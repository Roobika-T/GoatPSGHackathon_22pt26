from heapq import heappush, heappop
from models.navigation_graph import NavigationGraph
from models.traffic_manager import TrafficManager
from models.robot_spec import RobotSpec
from utils.robot import Robot

import tkinter as tk
from tkinter import simpledialog, messagebox
import random

class FleetManagementApp:
    def __init__(self, root: tk.Tk, json_path: str, max_robots: int = 5, spawn_prefix: str = "m"):
        self.root = root
        self.root.title("Fleet Management System")
        self.nav_graph = NavigationGraph(json_path, spawn_prefix)
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
            elif meta.get('name', '').startswith(self.nav_graph.spawn_prefix):
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
                    if vertex_data['meta'].get('name', '').startswith(self.nav_graph.spawn_prefix):
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
