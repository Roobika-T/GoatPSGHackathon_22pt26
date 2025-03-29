import json
import tkinter as tk
from tkinter import simpledialog, messagebox
from typing import Dict, List, Tuple, Set
from heapq import heappush, heappop
from math import sqrt
import random
import time
from dataclasses import dataclass

@dataclass
class RobotSpec:
    id: str
    start_vertex: str
    color: str
    speed: float
    battery: float = 100.0

class NavigationGraph:
    def __init__(self, json_path: str):
        with open(json_path, 'r') as f:
            data = json.load(f)
        level_data = data['levels']['level1']
        
        self.vertices = {}
        for idx, vertex in enumerate(level_data['vertices']):
            self.vertices[str(idx)] = {'x': vertex[0], 'y': vertex[1], 'meta': vertex[2]}
            
        self.lanes = [{'from': str(lane[0]), 'to': str(lane[1]), 'speed_limit': lane[2]['speed_limit']} 
                     for lane in level_data['lanes']]
        
        self.graph = {}
        self.reverse_graph = {}  # For checking opposite direction
        for lane in self.lanes:
            if lane['from'] not in self.graph:
                self.graph[lane['from']] = []
            if lane['to'] not in self.reverse_graph:
                self.reverse_graph[lane['to']] = []
            cost = self.calculate_distance(lane['from'], lane['to'])
            self.graph[lane['from']].append((lane['to'], cost))
            self.reverse_graph[lane['to']].append((lane['from'], cost))

    def calculate_distance(self, from_id: str, to_id: str) -> float:
        from_pos = self.vertices[from_id]
        to_pos = self.vertices[to_id]
        return sqrt((from_pos['x'] - to_pos['x'])**2 + (from_pos['y'] - to_pos['y'])**2)

    def heuristic(self, node: str, goal: str) -> float:
        return self.calculate_distance(node, goal)

    def get_robot_spawn_points(self) -> List[str]:
        return [vid for vid, vertex in self.vertices.items() 
                if vertex['meta']['name'] and vertex['meta']['name'].startswith('m')]

class TrafficManager:
    def __init__(self):
        self.lane_reservations = {}  # Lane reservations: {lane_id: [(robot_id, start_time, end_time)]}
        self.current_time = 0
        self.waiting_queues = {}  # Queues at vertices: {vertex_id: [(robot_id, arrival_time)]}
        self.lane_directions = {}  # Track direction of lane usage: {lane_id: robot_id}

    def reserve_lane(self, lane_id: str, robot_id: str, start_time: float, end_time: float) -> bool:
        if lane_id not in self.lane_reservations:
            self.lane_reservations[lane_id] = []

        # Check for head-on collision by looking at the opposite lane
        opposite_lane_id = lane_id.split('-')[1] + '-' + lane_id.split('-')[0]
        if opposite_lane_id in self.lane_reservations:
            for res_robot_id, res_start, res_end in self.lane_reservations[opposite_lane_id]:
                if max(start_time, res_start) < min(end_time, res_end):
                    return False  # Head-on collision detected

        # Check for same-direction conflicts
        for res_robot_id, res_start, res_end in self.lane_reservations[lane_id]:
            if res_robot_id == robot_id:
                continue
            if max(start_time, res_start) < min(end_time, res_end):
                return False  # Lane is occupied

        self.lane_reservations[lane_id].append((robot_id, start_time, end_time))
        self.lane_directions[lane_id] = robot_id  # Track which robot is using the lane
        return True

    def add_to_waiting_queue(self, vertex_id: str, robot_id: str, arrival_time: float):
        if vertex_id not in self.waiting_queues:
            self.waiting_queues[vertex_id] = []
        self.waiting_queues[vertex_id].append((robot_id, arrival_time))

    def process_waiting_queue(self, vertex_id: str) -> str:
        if vertex_id not in self.waiting_queues or not self.waiting_queues[vertex_id]:
            return None
        # Process the first robot in the queue
        robot_id, _ = self.waiting_queues[vertex_id][0]
        return robot_id

    def remove_from_waiting_queue(self, vertex_id: str, robot_id: str):
        if vertex_id in self.waiting_queues:
            self.waiting_queues[vertex_id] = [
                (rid, t) for rid, t in self.waiting_queues[vertex_id] if rid != robot_id
            ]
            if not self.waiting_queues[vertex_id]:
                del self.waiting_queues[vertex_id]

    def get_lane_id(self, from_vertex: str, to_vertex: str) -> str:
        return f"{from_vertex}-{to_vertex}"

    def update_time(self, new_time: float):
        self.current_time = new_time
        for lane_id in list(self.lane_reservations.keys()):
            self.lane_reservations[lane_id] = [
                res for res in self.lane_reservations[lane_id] 
                if res[2] > self.current_time 
            ]
            if not self.lane_reservations[lane_id]:
                del self.lane_reservations[lane_id]
                if lane_id in self.lane_directions:
                    del self.lane_directions[lane_id]

class RobotPathFinder:
    def __init__(self, nav_graph: NavigationGraph, traffic_manager: TrafficManager):
        self.nav_graph = nav_graph
        self.traffic_manager = traffic_manager

    def find_shortest_path(self, start: str, goal: str, robot_id: str, 
                           start_time: float = 0, speed: float = 1.0) -> Tuple[List[str], float, List[float]]:
        if start not in self.nav_graph.vertices or goal not in self.nav_graph.vertices:
            return [], 0.0, []
            
        open_set = [(0, start, [start], 0, [start_time])]  
        closed_set = set()
        g_scores = {start: 0}
        
        while open_set:
            f_score, current, path, cost, arrival_times = heappop(open_set)
            
            if current == goal:
                return path, cost, arrival_times
                
            if current in closed_set:
                continue
                
            closed_set.add(current)
            current_time = arrival_times[-1]
            
            neighbors = self.nav_graph.graph.get(current, [])
            for next_node, lane_distance in neighbors:
                if next_node in closed_set:
                    continue
                    
                travel_time = lane_distance / speed
                estimated_arrival = current_time + travel_time
                lane_id = self.traffic_manager.get_lane_id(current, next_node)
                
                if self.traffic_manager.reserve_lane(lane_id, robot_id, current_time, estimated_arrival):
                    new_cost = cost + lane_distance
                    if next_node not in g_scores or new_cost < g_scores[next_node]:
                        g_scores[next_node] = new_cost
                        h_score = self.nav_graph.heuristic(next_node, goal)
                        f_score = new_cost + h_score
                        new_path = path + [next_node]
                        new_arrival_times = arrival_times + [estimated_arrival]
                        heappush(open_set, (f_score, next_node, new_path, new_cost, new_arrival_times))
                        
        return [], 0.0, []  

class Robot:
    def __init__(self, robot_spec: RobotSpec, canvas: tk.Canvas, 
                 nav_graph: NavigationGraph, traffic_manager: TrafficManager):
        self.id = robot_spec.id
        self.current_vertex = robot_spec.start_vertex
        self.path = []
        self.arrival_times = []
        self.status = "idle"
        self.canvas = canvas
        self.nav_graph = nav_graph
        self.traffic_manager = traffic_manager
        self.speed = robot_spec.speed
        self.color = robot_spec.color
        self.battery = robot_spec.battery
        
        self.pos_x = nav_graph.vertices[robot_spec.start_vertex]['x'] * 50 + 200
        self.pos_y = -nav_graph.vertices[robot_spec.start_vertex]['y'] * 50 + 200
        self.obj = canvas.create_oval(self.pos_x-8, self.pos_y-8, self.pos_x+8, self.pos_y+8, 
                                    fill=self.color, outline="black", width=2)
        self.label = canvas.create_text(self.pos_x, self.pos_y-20, text=f"R{self.id}", font=("Arial", 10, "bold"))
        self.path_line = None
        
        self.battery_drain_rate = 0.5  
        self.battery_label = canvas.create_text(self.pos_x, self.pos_y+20, 
                                            text=f"{self.battery:.0f}%", fill="green", font=("Arial", 8))
        self.waiting_indicator = None  # For visualizing waiting status

    def move_to(self, goal: str):
        path_finder = RobotPathFinder(self.nav_graph, self.traffic_manager)
        current_time = self.traffic_manager.current_time
        self.path, _, self.arrival_times = path_finder.find_shortest_path(
            self.current_vertex, goal, self.id, current_time, self.speed)
            
        if self.path:
            self.status = "moving"
            self._update_label()
            
            if self.path_line:
                self.canvas.delete(self.path_line)
                
            path_coords = []
            for vertex_id in self.path:
                x = self.nav_graph.vertices[vertex_id]['x'] * 50 + 200
                y = -self.nav_graph.vertices[vertex_id]['y'] * 50 + 200
                path_coords.extend([x, y])
                
            if len(path_coords) >= 4:  
                self.path_line = self.canvas.create_line(path_coords, fill=self.color, dash=(4, 2), width=2)
                
            self._animate_movement(1)
        else:
            messagebox.showinfo("Path Finding", f"No available path found for Robot {self.id} to reach the destination.")

    def _animate_movement(self, path_idx: int):
        if path_idx >= len(self.path):
            self.status = "idle"
            self._update_label()
            self._stop_waiting_indicator()
            
            current_vertex_data = self.nav_graph.vertices[self.current_vertex]
            if current_vertex_data['meta'].get('is_charger', False):
                self.status = "charging"
                self._update_label()
                self._charge_battery()
            return
            
        next_vertex = self.path[path_idx]
        target_x = self.nav_graph.vertices[next_vertex]['x'] * 50 + 200
        target_y = -self.nav_graph.vertices[next_vertex]['y'] * 50 + 200
        
        distance = sqrt((target_x - self.pos_x)**2 + (target_y - self.pos_y)**2)
        steps = max(10, int(distance / 5))
        dx = (target_x - self.pos_x) / steps
        dy = (target_y - self.pos_y) / steps
        
        lane_id = self.traffic_manager.get_lane_id(self.current_vertex, next_vertex)
        lane_distance = self.nav_graph.calculate_distance(self.current_vertex, next_vertex)
        travel_time = lane_distance / self.speed
        
        # Check if the lane is available
        current_time = self.traffic_manager.current_time
        if not self.traffic_manager.reserve_lane(lane_id, self.id, current_time, current_time + travel_time):
            self.status = "waiting"
            self._update_label()
            self._start_waiting_indicator()
            self.traffic_manager.add_to_waiting_queue(self.current_vertex, self.id, current_time)
            
            def check_lane_availability():
                if self.traffic_manager.reserve_lane(lane_id, self.id, self.traffic_manager.current_time, 
                                                    self.traffic_manager.current_time + travel_time):
                    self.traffic_manager.remove_from_waiting_queue(self.current_vertex, self.id)
                    self.status = "moving"
                    self._update_label()
                    self._stop_waiting_indicator()
                    self._animate_movement(path_idx)
                else:
                    self.canvas.after(500, check_lane_availability)
                    
            self.canvas.after(500, check_lane_availability)
            return

        self.battery -= lane_distance * self.battery_drain_rate
        self.battery = max(0, self.battery)
        self._update_battery_display()
        
        if self.battery < 20 and self.status != "seeking_charger":
            closest_charger = self._find_closest_charger()
            if closest_charger and closest_charger != self.current_vertex:
                self.status = "seeking_charger"
                self._update_label()
                self.move_to(closest_charger)
                return
                
        def step(count=0):
            self.traffic_manager.update_time(time.time())
            
            if count >= steps:
                self.current_vertex = next_vertex
                self._animate_movement(path_idx + 1)
                return
                
            self.pos_x += dx
            self.pos_y += dy
            self.canvas.coords(self.obj, self.pos_x-8, self.pos_y-8, self.pos_x+8, self.pos_y+8)
            self.canvas.coords(self.label, self.pos_x, self.pos_y-20)
            self.canvas.coords(self.battery_label, self.pos_x, self.pos_y+20)
            
            delay = 50 / self.speed  
            self.canvas.after(int(delay), step, count + 1)
            
        step()

    def _start_waiting_indicator(self):
        if not self.waiting_indicator:
            self.waiting_indicator = self.canvas.create_text(
                self.pos_x, self.pos_y-30, text="⏳", font=("Arial", 12), fill="red"
            )
        def blink(state=1):
            if self.status != "waiting":
                return
            self.canvas.itemconfig(self.waiting_indicator, state="normal" if state else "hidden")
            self.canvas.after(500, blink, 1 - state)
        blink()

    def _stop_waiting_indicator(self):
        if self.waiting_indicator:
            self.canvas.delete(self.waiting_indicator)
            self.waiting_indicator = None

    def _update_label(self):
        status_text = f"R{self.id}"
        if self.status == "moving":
            status_text += " 🚀"
        elif self.status == "waiting":
            status_text += " ⏳"
        elif self.status == "charging":
            status_text += " ⚡"
        elif self.status == "seeking_charger":
            status_text += " 🔋"
        self.canvas.itemconfig(self.label, text=status_text)

    def _find_closest_charger(self) -> str:
        chargers = []
        for vid, vertex in self.nav_graph.vertices.items():
            if vertex['meta'].get('is_charger', False):
                distance = self.nav_graph.calculate_distance(self.current_vertex, vid)
                chargers.append((vid, distance))
                
        if chargers:
            chargers.sort(key=lambda x: x[1])
            return chargers[0][0]
        return None

    def _charge_battery(self):
        if self.battery < 100:
            self.battery = min(100, self.battery + 2.0)
            self._update_battery_display()
            
            if self.battery < 100:
                self.canvas.after(100, self._charge_battery)
            else:
                self.status = "idle"
                self._update_label()
    
    def _update_battery_display(self):
        if self.battery > 50:
            color = "green"
        elif self.battery > 20:
            color = "orange"
        else:
            color = "red"
            
        self.canvas.itemconfig(self.battery_label, text=f"{self.battery:.0f}%", fill=color)

class FleetManagementApp:
    def __init__(self, root: tk.Tk, json_path: str, max_robots: int = 5):
        self.root = root
        self.root.title("Fleet Management System")
        self.nav_graph = NavigationGraph(json_path)
        self.traffic_manager = TrafficManager()
        self.max_robots = max_robots
        
        main_frame = tk.Frame(root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        control_frame = tk.Frame(main_frame, width=250)
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5, pady=5)
        
        canvas_frame = tk.Frame(main_frame)
        canvas_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        self.canvas = tk.Canvas(canvas_frame, width=800, height=600, bg="white")
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        tk.Label(control_frame, text="Fleet Management System", font=("Arial", 12, "bold")).pack(pady=10)
        
        robot_frame = tk.LabelFrame(control_frame, text="Robot Management", padx=5, pady=5)
        robot_frame.pack(fill=tk.X, pady=5)
        
        self.robot_count_label = tk.Label(robot_frame, text=f"Robots: 0/{self.max_robots}")
        self.robot_count_label.pack()
        
        tk.Button(robot_frame, text="Add Robot", command=self.spawn_robot_ui).pack(fill=tk.X, pady=5)
        tk.Button(robot_frame, text="Remove Robot", command=self.remove_robot_ui).pack(fill=tk.X, pady=5)
        
        task_frame = tk.LabelFrame(control_frame, text="Task Assignment", padx=5, pady=5)
        task_frame.pack(fill=tk.X, pady=5)
        
        tk.Label(task_frame, text="Robot ID:").pack(anchor=tk.W)
        self.robot_id_var = tk.StringVar()
        tk.Entry(task_frame, textvariable=self.robot_id_var).pack(fill=tk.X)
        
        tk.Label(task_frame, text="Destination:").pack(anchor=tk.W)
        self.dest_var = tk.StringVar()
        tk.Entry(task_frame, textvariable=self.dest_var).pack(fill=tk.X)
        
        tk.Button(task_frame, text="Assign Task", command=self.assign_task).pack(fill=tk.X, pady=5)
        
        info_frame = tk.LabelFrame(control_frame, text="System Information", padx=5, pady=5)
        info_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        tk.Label(info_frame, text="Traffic Status", font=("Arial", 10, "bold")).pack()
        self.traffic_info = tk.Text(info_frame, height=8, width=25)
        self.traffic_info.pack(fill=tk.X, pady=2)
        
        tk.Label(info_frame, text="Robot Status", font=("Arial", 10, "bold")).pack()
        self.status_display = tk.Text(info_frame, height=12, width=25)
        self.status_display.pack(fill=tk.X, pady=2)
        
        self.robots = {}
        self.robot_counter = 0
        self.vertex_objects = {}
        
        self.draw_environment()
        self.canvas.bind("<Button-1>", self.on_canvas_click)
        
        self.update_displays()

    def draw_environment(self):
        for lane in self.nav_graph.lanes:
            x1 = self.nav_graph.vertices[lane['from']]['x'] * 50 + 200
            y1 = -self.nav_graph.vertices[lane['from']]['y'] * 50 + 200
            x2 = self.nav_graph.vertices[lane['to']]['x'] * 50 + 200
            y2 = -self.nav_graph.vertices[lane['to']]['y'] * 50 + 200
            self.canvas.create_line(x1, y1, x2, y2, fill="gray", width=2)

        for vid, vertex in self.nav_graph.vertices.items():
            x = vertex['x'] * 50 + 200
            y = -vertex['y'] * 50 + 200
            
            meta = vertex['meta']
            if meta.get('is_charger', False):
                self.vertex_objects[vid] = self.canvas.create_oval(
                    x-12, y-12, x+12, y+12, fill="yellow", outline="orange", width=2
                )
                self.canvas.create_text(x, y+25, text="⚡", font=("Arial", 14))
            elif meta.get('name', '').startswith('m'):
                self.vertex_objects[vid] = self.canvas.create_rectangle(
                    x-12, y-12, x+12, y+12, fill="lightgreen", outline="darkgreen", width=2
                )
            else:
                self.vertex_objects[vid] = self.canvas.create_oval(
                    x-10, y-10, x+10, y+10, fill="lightblue", outline="blue", width=1
                )
            
            name = meta.get('name', vid)
            self.canvas.create_text(x, y, text=name, font=("Arial", 8))

    def on_canvas_click(self, event):
        for vid, obj in self.vertex_objects.items():
            coords = self.canvas.coords(obj)
            if coords and len(coords) >= 4:
                x1, y1, x2, y2 = coords
                if x1 <= event.x <= x2 and y1 <= event.y <= y2:
                    vertex_data = self.nav_graph.vertices[vid]
                    if vertex_data['meta'].get('name', '').startswith('m'):
                        self.spawn_robot_ui(vid)
                    return

    def spawn_robot_ui(self, vertex_id=None):
        if len(self.robots) >= self.max_robots:
            messagebox.showwarning("Limit Reached", f"Maximum number of robots ({self.max_robots}) reached.")
            return
            
        if vertex_id is None:
            spawn_points = self.nav_graph.get_robot_spawn_points()
            if not spawn_points:
                messagebox.showerror("Error", "No robot spawn points available in the navigation graph.")
                return
                
            vertex_id = simpledialog.askstring("Spawn Robot", 
                                             f"Enter spawn vertex ID ({', '.join(spawn_points)}):",
                                             parent=self.root)
            if not vertex_id or vertex_id not in spawn_points:
                return
        
        self.robot_counter += 1
        robot_id = str(self.robot_counter)
        
        robot_spec = RobotSpec(
            id=robot_id,
            start_vertex=vertex_id,
            color=f"#{random.randint(0, 255):02x}{random.randint(0, 255):02x}{random.randint(0, 255):02x}",
            speed=1.0 + random.random() * 0.5,
            battery=100.0
        )
        
        robot = Robot(robot_spec, self.canvas, self.nav_graph, self.traffic_manager)
        self.robots[robot_id] = robot
        
        self.update_status_display()
        self.robot_count_label.config(text=f"Robots: {len(self.robots)}/{self.max_robots}")

    def remove_robot_ui(self):
        if not self.robots:
            messagebox.showinfo("Info", "No robots to remove.")
            return
            
        robot_id = simpledialog.askstring("Remove Robot", 
                                        f"Enter robot ID to remove ({', '.join(self.robots.keys())}):",
                                        parent=self.root)
        if robot_id and robot_id in self.robots:
            robot = self.robots[robot_id]
            self.canvas.delete(robot.obj)
            self.canvas.delete(robot.label)
            self.canvas.delete(robot.battery_label)
            if robot.path_line:
                self.canvas.delete(robot.path_line)
            if robot.waiting_indicator:
                self.canvas.delete(robot.waiting_indicator)
            del self.robots[robot_id]
            
            self.update_status_display()
            self.robot_count_label.config(text=f"Robots: {len(self.robots)}/{self.max_robots}")

    def assign_task(self):
        robot_id = self.robot_id_var.get().strip()
        dest_id = self.dest_var.get().strip()
        
        if not robot_id or not dest_id:
            messagebox.showwarning("Input Error", "Please enter both robot ID and destination.")
            return
            
        if robot_id not in self.robots:
            messagebox.showwarning("Input Error", f"Robot {robot_id} not found.")
            return
            
        if dest_id not in self.nav_graph.vertices:
            messagebox.showwarning("Input Error", f"Destination {dest_id} not found in navigation graph.")
            return
            
        self.robots[robot_id].move_to(dest_id)
        self.update_status_display()

    def update_displays(self):
        self.update_status_display()
        self.update_traffic_display()
        self.root.after(1000, self.update_displays)
        
    def update_status_display(self):
        self.status_display.delete(1.0, tk.END)
        for robot_id, robot in sorted(self.robots.items(), key=lambda x: int(x[0])):
            vertex_info = f"{robot.current_vertex}"
            if robot.current_vertex in self.nav_graph.vertices:
                name = self.nav_graph.vertices[robot.current_vertex]['meta'].get('name', '')
                if name:
                    vertex_info += f" ({name})"
                    
            self.status_display.insert(tk.END, 
                f"Robot {robot_id}:\n"
                f"Status: {robot.status.upper()}\n"
                f"Position: {vertex_info}\n"
                f"Battery: {robot.battery:.0f}%\n"
                f"Speed: {robot.speed:.1f}x\n\n"
            )
            
    def update_traffic_display(self):
        self.traffic_info.delete(1.0, tk.END)
        total_reservations = sum(len(reservations) for reservations in self.traffic_manager.lane_reservations.values())
        total_waiting = sum(len(queue) for queue in self.traffic_manager.waiting_queues.values())
        
        self.traffic_info.insert(tk.END, f"Active Robots: {len(self.robots)}\n")
        self.traffic_info.insert(tk.END, f"Active Lanes: {len(self.traffic_manager.lane_reservations)}\n")
        self.traffic_info.insert(tk.END, f"Total Reservations: {total_reservations}\n")
        self.traffic_info.insert(tk.END, f"Robots Waiting: {total_waiting}\n\n")
        
        if self.traffic_manager.waiting_queues:
            self.traffic_info.insert(tk.END, "Waiting Queues:\n")
            for vertex_id, queue in self.traffic_manager.waiting_queues.items():
                self.traffic_info.insert(tk.END, f" - Vertex {vertex_id}: {len(queue)} robots\n")

def main():
    root = tk.Tk()
    app = FleetManagementApp(root, '/Users/roobikatura/GoatPSGHackathon_22pt26/nav_graph_samples/nav_graph_1.json', max_robots=5)
    root.mainloop()

if __name__ == "__main__":
    main()