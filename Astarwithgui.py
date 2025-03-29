import json
import tkinter as tk
from tkinter import simpledialog
from typing import Dict, List, Tuple
from heapq import heappush, heappop
from math import sqrt
import random

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

class RobotPathFinder:
    def __init__(self, nav_graph: NavigationGraph):
        self.nav_graph = nav_graph

    def find_shortest_path(self, start: str, goal: str) -> Tuple[List[str], float]:
        if start not in self.nav_graph.vertices or goal not in self.nav_graph.vertices:
            return [], 0.0
        open_set = [(0, start, [start], 0)]
        closed_set = set()
        g_scores = {start: 0}
        
        while open_set:
            f_score, current, path, cost = heappop(open_set)
            if current == goal:
                return path, cost
            if current in closed_set:
                continue
            closed_set.add(current)
            neighbors = self.nav_graph.graph.get(current, [])
            for next_node, lane_cost in neighbors:
                if next_node in closed_set:
                    continue
                new_cost = cost + lane_cost
                if next_node not in g_scores or new_cost < g_scores[next_node]:
                    g_scores[next_node] = new_cost
                    h_score = self.nav_graph.heuristic(next_node, goal)
                    f_score = new_cost + h_score
                    new_path = path + [next_node]
                    heappush(open_set, (f_score, next_node, new_path, new_cost))
        return [], 0.0

class Robot:
    def __init__(self, robot_id: str, start_vertex: str, canvas: tk.Canvas, nav_graph: NavigationGraph):
        self.id = robot_id
        self.current_vertex = start_vertex
        self.path = []
        self.status = "idle"
        self.canvas = canvas
        self.nav_graph = nav_graph
        self.color = f"#{random.randint(0, 255):02x}{random.randint(0, 255):02x}{random.randint(0, 255):02x}"
        self.pos_x = nav_graph.vertices[start_vertex]['x'] * 50 + 200
        self.pos_y = -nav_graph.vertices[start_vertex]['y'] * 50 + 200
        self.obj = canvas.create_oval(self.pos_x-5, self.pos_y-5, self.pos_x+5, self.pos_y+5, fill=self.color)
        self.label = canvas.create_text(self.pos_x, self.pos_y-15, text=f"R{robot_id} ({self.status})")

    def move_to(self, goal: str):
        path_finder = RobotPathFinder(self.nav_graph)
        self.path, _ = path_finder.find_shortest_path(self.current_vertex, goal)
        if self.path:
            self.status = "moving"
            self.canvas.itemconfig(self.label, text=f"R{self.id} ({self.status})")
            self._animate_movement(1)

    def _animate_movement(self, path_idx: int):
        if path_idx >= len(self.path):
            self.status = "idle"
            self.canvas.itemconfig(self.label, text=f"R{self.id} ({self.status})")
            return
        next_vertex = self.path[path_idx]
        target_x = self.nav_graph.vertices[next_vertex]['x'] * 50 + 200
        target_y = -self.nav_graph.vertices[next_vertex]['y'] * 50 + 200
        dx = (target_x - self.pos_x) / 10
        dy = (target_y - self.pos_y) / 10
        
        def step(count=0):
            if count > 10:
                self.current_vertex = next_vertex
                self._animate_movement(path_idx + 1)
                return
            self.pos_x += dx
            self.pos_y += dy
            self.canvas.coords(self.obj, self.pos_x-5, self.pos_y-5, self.pos_x+5, self.pos_y+5)
            self.canvas.coords(self.label, self.pos_x, self.pos_y-15)
            self.canvas.after(50, step, count + 1)
        
        step()

class FleetManagementApp:
    def __init__(self, root: tk.Tk, json_path: str):
        self.root = root
        self.root.title("Fleet Management System")
        self.nav_graph = NavigationGraph(json_path)
        self.canvas = tk.Canvas(root, width=800, height=600, bg="white")
        self.canvas.pack(pady=20)
        self.robots = {}
        self.robot_counter = 0
        
        self.draw_environment()
        self.canvas.bind("<Button-1>", self.on_click)

    def draw_environment(self):
        for lane in self.nav_graph.lanes:
            x1 = self.nav_graph.vertices[lane['from']]['x'] * 50 + 200
            y1 = -self.nav_graph.vertices[lane['from']]['y'] * 50 + 200
            x2 = self.nav_graph.vertices[lane['to']]['x'] * 50 + 200
            y2 = -self.nav_graph.vertices[lane['to']]['y'] * 50 + 200
            self.canvas.create_line(x1, y1, x2, y2, fill="gray")

        self.vertex_objects = {}
        for vid, vertex in self.nav_graph.vertices.items():
            x = vertex['x'] * 50 + 200
            y = -vertex['y'] * 50 + 200
            self.vertex_objects[vid] = self.canvas.create_oval(x-10, y-10, x+10, y+10, fill="lightblue")
            name = vertex['meta']['name'] or vid
            self.canvas.create_text(x, y, text=name)

    def on_click(self, event):
        for vid, obj in self.vertex_objects.items():
            x, y = self.nav_graph.vertices[vid]['x'] * 50 + 200, -self.nav_graph.vertices[vid]['y'] * 50 + 200
            if (x-10 <= event.x <= x+10) and (y-10 <= event.y <= y+10):
                self.spawn_robot(vid)
                break

    def spawn_robot(self, vertex_id: str):
        self.robot_counter += 1
        robot = Robot(str(self.robot_counter), vertex_id, self.canvas, self.nav_graph)
        self.robots[str(self.robot_counter)] = robot
        print(f"Spawned Robot R{self.robot_counter} at vertex {vertex_id}")
        
        goal = simpledialog.askstring("Destination", 
                                    f"Enter destination vertex ID for Robot R{self.robot_counter} (0-13):",
                                    parent=self.root)
        if goal and goal.strip() in self.nav_graph.vertices:
            robot.move_to(goal.strip())
        else:
            print("Invalid destination. Robot remains idle.")

def main():
    root = tk.Tk()
    app = FleetManagementApp(root, '/Users/roobikatura/GoatPSGHackathon_22pt26/nav_graph_samples/nav_graph_1.json')
    root.mainloop()

if __name__ == "__main__":
    main()