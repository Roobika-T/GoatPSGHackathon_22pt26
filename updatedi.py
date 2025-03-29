import json
import tkinter as tk
from tkinter import simpledialog, messagebox
from typing import Dict, List, Tuple
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
        """Attempt to reserve lane for the robot in the specified time window"""
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
        self.start_vertex_var = tk.StringVar()
        self.start_vertex_menu = tk.OptionMenu(control_frame, self.start_vertex_var, *self.nav_graph.vertices.keys())
        self.start_vertex_menu.pack(pady=5)

        tk.Label(control_frame, text="Select Destination Vertex:").pack(pady=5)
        self.destination_vertex_var = tk.StringVar()
        self.destination_vertex_menu = tk.OptionMenu(control_frame, self.destination_vertex_var, *self.nav_graph.vertices.keys())
        self.destination_vertex_menu.pack(pady=5)

        self.spawn_button = tk.Button(control_frame, text="Spawn Robot", command=self.spawn_robot)
        self.spawn_button.pack(pady=10)

        self.start_navigation_button = tk.Button(control_frame, text="Start Navigation", command=self.start_navigation)
        self.start_navigation_button.pack(pady=10)

        self.robots = {}
        self.robot_count = 0

        self.draw_environment()

    def draw_environment(self):
        for vertex_id, vertex in self.nav_graph.vertices.items():
            x = vertex['x'] * 50 + 200
            y = -vertex['y'] * 50 + 200
            self.canvas.create_oval(x-5, y-5, x+5, y+5, fill="black")
            self.canvas.create_text(x, y-10, text=f"{vertex_id} ({vertex['meta']['name']})")

        for lane in self.nav_graph.lanes:
            from_vertex = self.nav_graph.vertices[lane['from']]
            to_vertex = self.nav_graph.vertices[lane['to']]
            from_x = from_vertex['x'] * 50 + 200
            from_y = -from_vertex['y'] * 50 + 200
            to_x = to_vertex['x'] * 50 + 200
            to_y = -to_vertex['y'] * 50 + 200
            self.canvas.create_line(from_x, from_y, to_x, to_y, fill="gray")

    def spawn_robot(self):
        start_vertex = self.start_vertex_var.get()
        if start_vertex not in self.nav_graph.vertices:
            messagebox.showerror("Error", "Please select a valid start vertex.")
            return

        self.robot_count += 1
        robot_id = str(self.robot_count)
        robot = Robot(robot_id, start_vertex, self.canvas, self.nav_graph, self.traffic_manager)
        self.robots[robot_id] = robot

    def start_navigation(self):
        destination_vertex = self.destination_vertex_var.get()
        if destination_vertex not in self.nav_graph.vertices:
            messagebox.showerror("Error", "Please select a valid destination vertex.")
            return

        # Start navigation for all robots
        for robot_id, robot in self.robots.items():
            robot.move_to(destination_vertex)

if __name__ == "__main__":
    root = tk.Tk()
    app = FleetManagementApp(root, '/Users/roobikatura/GoatPSGHackathon_22pt26/nav_graph_samples/nav_graph_1.json')
    root.mainloop()