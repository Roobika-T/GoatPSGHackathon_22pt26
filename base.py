import json
import tkinter as tk
from tkinter import simpledialog, messagebox
from typing import Dict, List, Tuple, Set
from heapq import heappush, heappop
from math import sqrt
import random
import time

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
        for lane in self.lanes:
            if lane['from'] not in self.graph:
                self.graph[lane['from']] = []
            cost = self.calculate_distance(lane['from'], lane['to'])
            self.graph[lane['from']].append((lane['to'], cost))

    def calculate_distance(self, from_id: str, to_id: str) -> float:
        from_pos = self.vertices[from_id]
        to_pos = self.vertices[to_id]
        return sqrt((from_pos['x'] - to_pos['x'])**2 + (from_pos['y'] - to_pos['y'])**2)

    def heuristic(self, node: str, goal: str) -> float:
        return self.calculate_distance(node, goal)

class TrafficManager:
    def __init__(self):
        self.lane_reservations = {}
        self.current_time = 0
        
    def reserve_lane(self, lane_id: str, robot_id: str, start_time: float, end_time: float) -> bool:
        if lane_id not in self.lane_reservations:
            self.lane_reservations[lane_id] = []
            
        for res_robot_id, res_start, res_end in self.lane_reservations[lane_id]:
            if res_robot_id == robot_id:
                continue
            
            if max(start_time, res_start) < min(end_time, res_end):
                return False  
                
        self.lane_reservations[lane_id].append((robot_id, start_time, end_time))
        return True
        
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
    def __init__(self, robot_id: str, start_vertex: str, canvas: tk.Canvas, 
                 nav_graph: NavigationGraph, traffic_manager: TrafficManager):
        self.id = robot_id
        self.current_vertex = start_vertex
        self.path = []
        self.arrival_times = []
        self.status = "idle"
        self.canvas = canvas
        self.nav_graph = nav_graph
        self.traffic_manager = traffic_manager
        self.speed = 1.0 + random.random() * 0.5  
        
        self.color = f"#{random.randint(0, 255):02x}{random.randint(0, 255):02x}{random.randint(0, 255):02x}"
        self.pos_x = nav_graph.vertices[start_vertex]['x'] * 50 + 200
        self.pos_y = -nav_graph.vertices[start_vertex]['y'] * 50 + 200
        self.obj = canvas.create_oval(self.pos_x-5, self.pos_y-5, self.pos_x+5, self.pos_y+5, fill=self.color)
        self.label = canvas.create_text(self.pos_x, self.pos_y-15, text=f"R{robot_id} ({self.status})")
        self.path_line = None
        
        self.battery = 100.0
        self.battery_drain_rate = 0.5  
        self.battery_label = canvas.create_text(self.pos_x, self.pos_y+15, 
                                            text=f"Batt: {self.battery:.1f}%", fill="green")

    def move_to(self, goal: str):
        path_finder = RobotPathFinder(self.nav_graph, self.traffic_manager)
        current_time = self.traffic_manager.current_time
        self.path, _, self.arrival_times = path_finder.find_shortest_path(
            self.current_vertex, goal, self.id, current_time, self.speed)
            
        if self.path:
            self.status = "moving"
            self.canvas.itemconfig(self.label, text=f"R{self.id} ({self.status})")
            
            if self.path_line:
                self.canvas.delete(self.path_line)
                
            path_coords = []
            for vertex_id in self.path:
                x = self.nav_graph.vertices[vertex_id]['x'] * 50 + 200
                y = -self.nav_graph.vertices[vertex_id]['y'] * 50 + 200
                path_coords.extend([x, y])
                
            if len(path_coords) >= 4:  
                self.path_line = self.canvas.create_line(path_coords, fill=self.color, dash=(4, 2))
                
            self._animate_movement(1)
        else:
            messagebox.showinfo("Path Finding", f"No available path found for Robot {self.id} to reach the destination.")

    def _animate_movement(self, path_idx: int):
        if path_idx >= len(self.path):
            self.status = "idle"
            self.canvas.itemconfig(self.label, text=f"R{self.id} ({self.status})")
            
            current_vertex_data = self.nav_graph.vertices[self.current_vertex]
            if 'is_charger' in current_vertex_data['meta'] and current_vertex_data['meta']['is_charger']:
                self.status = "charging"
                self.canvas.itemconfig(self.label, text=f"R{self.id} ({self.status})")
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
        
        self.battery -= lane_distance * self.battery_drain_rate
        self.battery = max(0, self.battery)
        self._update_battery_display()
        
        if self.battery < 20 and self.status != "seeking_charger":
            closest_charger = self._find_closest_charger()
            if closest_charger and closest_charger != self.current_vertex:
                self.status = "seeking_charger"
                self.canvas.itemconfig(self.label, text=f"R{self.id} ({self.status})")
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
            self.canvas.coords(self.obj, self.pos_x-5, self.pos_y-5, self.pos_x+5, self.pos_y+5)
            self.canvas.coords(self.label, self.pos_x, self.pos_y-15)
            self.canvas.coords(self.battery_label, self.pos_x, self.pos_y+15)
            
            delay = 50 / self.speed  
            self.canvas.after(int(delay), step, count + 1)
            
        step()

    def _find_closest_charger(self) -> str:
       
        chargers = []
        for vid, vertex in self.nav_graph.vertices.items():
            if 'is_charger' in vertex['meta'] and vertex['meta']['is_charger']:
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
                self.canvas.itemconfig(self.label, text=f"R{self.id} ({self.status})")
    
    def _update_battery_display(self):
        if self.battery > 50:
            color = "green"
        elif self.battery > 20:
            color = "orange"
        else:
            color = "red"
            
        self.canvas.itemconfig(self.battery_label, text=f"Batt: {self.battery:.1f}%", fill=color)

class FleetManagementApp:
    def __init__(self, root: tk.Tk, json_path: str):
        self.root = root
        self.root.title("Fleet Management System")
        self.nav_graph = NavigationGraph(json_path)
        self.traffic_manager = TrafficManager()
        
        main_frame = tk.Frame(root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        control_frame = tk.Frame(main_frame, width=200)
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5, pady=5)
        
        canvas_frame = tk.Frame(main_frame)
        canvas_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        self.canvas = tk.Canvas(canvas_frame, width=800, height=600, bg="white")
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        tk.Label(control_frame, text="Fleet Management Controls", font=("Arial", 12, "bold")).pack(pady=10)
        
        tk.Label(control_frame, text="Select Start Vertex:").pack(pady=5)
        self.start_var = tk.StringVar()
        self.start_combobox = tk.OptionMenu(
            control_frame, 
            self.start_var,
            *[f"{vid} ({v['meta']['name'] or 'unnamed'})" for vid, v in self.nav_graph.vertices.items()]
        )
        self.start_combobox.pack(fill=tk.X, pady=2)
        
        tk.Button(control_frame, text="Spawn Robot", command=self.spawn_robot_from_ui).pack(fill=tk.X, pady=10)
        
        tk.Label(control_frame, text="Traffic Information", font=("Arial", 10, "bold")).pack(pady=10)
        self.traffic_info = tk.Text(control_frame, height=10, width=25)
        self.traffic_info.pack(fill=tk.X, pady=5)
        
        tk.Label(control_frame, text="Robot Status", font=("Arial", 10, "bold")).pack(pady=10)
        self.status_display = tk.Text(control_frame, height=15, width=25)
        self.status_display.pack(fill=tk.X, pady=5)
        
        self.robots = {}
        self.robot_counter = 0
        self.vertex_objects = {}
        
        self.draw_environment()
        self.canvas.bind("<Button-1>", self.on_click)
        
        self.update_displays()
        
    def draw_environment(self):
        for lane in self.nav_graph.lanes:
            x1 = self.nav_graph.vertices[lane['from']]['x'] * 50 + 200
            y1 = -self.nav_graph.vertices[lane['from']]['y'] * 50 + 200
            x2 = self.nav_graph.vertices[lane['to']]['x'] * 50 + 200
            y2 = -self.nav_graph.vertices[lane['to']]['y'] * 50 + 200
            self.canvas.create_line(x1, y1, x2, y2, fill="gray")

        for vid, vertex in self.nav_graph.vertices.items():
            x = vertex['x'] * 50 + 200
            y = -vertex['y'] * 50 + 200
            
            if 'is_charger' in vertex['meta'] and vertex['meta']['is_charger']:
                self.vertex_objects[vid] = self.canvas.create_oval(
                    x-12, y-12, x+12, y+12, fill="yellow", outline="orange", width=2
                )
                self.canvas.create_text(x, y+25, text="⚡", font=("Arial", 14))
            else:
                self.vertex_objects[vid] = self.canvas.create_oval(
                    x-10, y-10, x+10, y+10, fill="lightblue"
                )
                
            name = vertex['meta']['name'] or vid
            self.canvas.create_text(x, y, text=name)

    def on_click(self, event):
        for vid, obj in self.vertex_objects.items():
            coords = self.canvas.coords(obj)
            if coords and len(coords) >= 4:
                x1, y1, x2, y2 = coords
                if x1 <= event.x <= x2 and y1 <= event.y <= y2:
                    self.spawn_robot(vid)
                    break

    def spawn_robot_from_ui(self):
        selected = self.start_var.get()
        if selected:
            vid = selected.split()[0]  
            self.spawn_robot(vid)

    def spawn_robot(self, vertex_id: str):
        self.robot_counter += 1
        robot = Robot(str(self.robot_counter), vertex_id, self.canvas, 
                     self.nav_graph, self.traffic_manager)
        self.robots[str(self.robot_counter)] = robot
        
        self.update_status_display()
        
        goal = simpledialog.askstring("Destination", 
                                     f"Enter destination vertex ID for Robot R{self.robot_counter} (0-13):",
                                     parent=self.root)
        if goal and goal.strip() in self.nav_graph.vertices:
            robot.move_to(goal.strip())
        else:
            messagebox.showinfo("Invalid Input", "Invalid destination. Robot remains idle.")

    def update_displays(self):
        self.update_status_display()
        self.update_traffic_display()
        self.root.after(1000, self.update_displays)
        
    def update_status_display(self):
        self.status_display.delete(1.0, tk.END)
        for robot_id, robot in self.robots.items():
            vertex_info = f"{robot.current_vertex}"
            if robot.current_vertex in self.nav_graph.vertices:
                name = self.nav_graph.vertices[robot.current_vertex]['meta'].get('name', '')
                if name:
                    vertex_info += f" ({name})"
                    
            self.status_display.insert(tk.END, 
                f"Robot {robot_id}: {robot.status.upper()}\n" +
                f"  Position: {vertex_info}\n" +
                f"  Battery: {robot.battery:.1f}%\n\n"
            )
            
    def update_traffic_display(self):
        self.traffic_info.delete(1.0, tk.END)
        total_reservations = sum(len(reservations) for reservations in self.traffic_manager.lane_reservations.values())
        self.traffic_info.insert(tk.END, f"Active lanes: {len(self.traffic_manager.lane_reservations)}\n")
        self.traffic_info.insert(tk.END, f"Total reservations: {total_reservations}\n\n")
        
        if self.traffic_manager.lane_reservations:
            self.traffic_info.insert(tk.END, "Busiest lanes:\n")
            lanes = sorted(self.traffic_manager.lane_reservations.items(), 
                         key=lambda x: len(x[1]), reverse=True)
            for i, (lane_id, reservations) in enumerate(lanes[:3]):
                if i < 3: 
                    self.traffic_info.insert(tk.END, f" - {lane_id}: {len(reservations)} robots\n")

def main():
    root = tk.Tk()
    app = FleetManagementApp(root, '/Users/roobikatura/GoatPSGHackathon_22pt26/nav_graph_samples/nav_graph_1.json')
    root.mainloop()

if __name__ == "__main__":
    main()