from models.robot_spec import RobotSpec
from models.navigation_graph import NavigationGraph
from models.traffic_manager import TrafficManager
from utils.robot_pathfinder import RobotPathFinder
import tkinter as tk
import time
from math import sqrt

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

        self.battery_drain_rate = 0.1 
        self.battery_label = canvas.create_text(self.pos_x, self.pos_y+20,
                                            text=f"{self.battery:.0f}%", fill="green", font=("Arial", 8))
        self.waiting_indicator = None

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
            print(f"Robot {self.id}: No path found to {goal}, retrying in 1 second...")
            self.status = "waiting"
            self._update_label()
            self._start_waiting_indicator()
            self.canvas.after(1000, lambda: self.move_to(goal))

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
            print(f"Robot {self.id}: Reached destination {self.current_vertex}, status: {self.status}")
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

        current_time = self.traffic_manager.current_time
        print(f"Robot {self.id}: Attempting to move from {self.current_vertex} to {next_vertex} (lane {lane_id})")
        if not self.traffic_manager.reserve_lane(lane_id, self.id, current_time, current_time + travel_time):
            self.status = "waiting"
            self._update_label()
            self._start_waiting_indicator()
            self.traffic_manager.add_to_waiting_queue(self.current_vertex, self.id, current_time)
            print(f"Robot {self.id}: Waiting at {self.current_vertex} for lane {lane_id}")

            def check_lane_availability():
                self.traffic_manager.update_time(time.time())
                current_time = self.traffic_manager.current_time
                print(f"Robot {self.id}: Checking lane {lane_id} availability at time {current_time}")
                if self.traffic_manager.reserve_lane(lane_id, self.id, current_time, current_time + travel_time):
                    self.traffic_manager.remove_from_waiting_queue(self.current_vertex, self.id)
                    self.status = "moving"
                    self._update_label()
                    self._stop_waiting_indicator()
                    print(f"Robot {self.id}: Lane {lane_id} now available, resuming movement")
                    self._animate_movement(path_idx)
                else:
                    print(f"Robot {self.id}: Lane {lane_id} still unavailable, waiting...")
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
                print(f"Robot {self.id}: Low battery ({self.battery:.0f}%), seeking charger at {closest_charger}")
                self.move_to(closest_charger)
                return

        def step(count=0):
            self.traffic_manager.update_time(time.time())

            if count >= steps:
                self.current_vertex = next_vertex
                print(f"Robot {self.id}: Moved to {self.current_vertex}, continuing to next vertex")
                self._animate_movement(path_idx + 1)
                return

            self.pos_x += dx
            self.pos_y += dy
            self.canvas.coords(self.obj, self.pos_x-8, self.pos_y-8, self.pos_x+8, self.pos_y+8)
            self.canvas.coords(self.label, self.pos_x, self.pos_y-20)
            self.canvas.coords(self.battery_label, self.pos_x, self.pos_y+20)

            delay = 50 / self.speed
            self.canvas.after(int(delay), step, count + 1)

        print(f"Robot {self.id}: Starting movement to {next_vertex}")
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
