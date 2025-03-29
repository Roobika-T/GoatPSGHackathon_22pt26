import json
from math import sqrt
from typing import List

class NavigationGraph:
    def __init__(self, json_path: str, spawn_prefix: str = "m"):
        self.spawn_prefix = spawn_prefix

        try:
            with open(json_path, 'r') as f:
                data = json.load(f)
        except FileNotFoundError:
            raise FileNotFoundError(f"JSON file not found at path: {json_path}")
        except json.JSONDecodeError:
            raise ValueError(f"Invalid JSON format in file: {json_path}")

        if 'levels' not in data:
            raise KeyError("JSON file must contain a 'levels' key")

        levels = data['levels']
        if not levels:
            raise KeyError("No levels found in the JSON file")
        level_name = next(iter(levels))
        level_data = levels[level_name]

        if 'vertices' not in level_data:
            raise KeyError("Level data must contain 'vertices'")
        if 'lanes' not in level_data:
            raise KeyError("Level data must contain 'lanes'")

        self.vertices = {}
        for idx, vertex in enumerate(level_data['vertices']):
            if not isinstance(vertex, list) or len(vertex) < 3:
                raise ValueError(f"Vertex {idx} is malformed: expected [x, y, meta]")
            if not isinstance(vertex[0], (int, float)) or not isinstance(vertex[1], (int, float)):
                raise ValueError(f"Vertex {idx} coordinates must be numbers: {vertex}")
            if not isinstance(vertex[2], dict):
                raise ValueError(f"Vertex {idx} meta must be a dictionary: {vertex}")
            self.vertices[str(idx)] = {
                'x': float(vertex[0]),
                'y': float(vertex[1]),
                'meta': vertex[2]
            }

        self.lanes = []
        for lane in level_data['lanes']:
            if not isinstance(lane, list) or len(lane) != 3:
                raise ValueError(f"Lane is malformed: expected [from, to, {{'speed_limit': value}}], got {lane}")
            if not isinstance(lane[0], int) or not isinstance(lane[1], int):
                raise ValueError(f"Lane vertex IDs must be integers: {lane}")
            if not isinstance(lane[2], dict) or 'speed_limit' not in lane[2]:
                raise ValueError(f"Lane must have a 'speed_limit' in metadata: {lane}")
            from_vertex = str(lane[0])
            to_vertex = str(lane[1])
            if from_vertex not in self.vertices or to_vertex not in self.vertices:
                raise ValueError(f"Lane references invalid vertex: {lane}")
            self.lanes.append({
                'from': from_vertex,
                'to': to_vertex,
                'speed_limit': float(lane[2]['speed_limit'])
            })

        self.graph = {}
        self.reverse_graph = {}
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
        spawn_points = [
            vid for vid, vertex in self.vertices.items()
            if vertex['meta'].get('name', '').startswith(self.spawn_prefix)
        ]
        if not spawn_points:
            spawn_points = list(self.vertices.keys())
        return spawn_points
