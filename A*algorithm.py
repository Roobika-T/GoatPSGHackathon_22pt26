import json
from typing import Dict, List, Tuple
from heapq import heappush, heappop
from math import sqrt

class NavigationGraph:
    def __init__(self, json_path: str):
        with open(json_path, 'r') as f:
            data = json.load(f)
        
        level_data = data['levels']['level1']
        
        self.vertices = {}
        for idx, vertex in enumerate(level_data['vertices']):
            self.vertices[str(idx)] = {
                'x': vertex[0],
                'y': vertex[1],
                'meta': vertex[2]
            }
            
        self.lanes = []
        for lane in level_data['lanes']:
            self.lanes.append({
                'from': str(lane[0]),
                'to': str(lane[1]),
                'speed_limit': lane[2]['speed_limit']
            })
            
        self.graph = {}
        for lane in self.lanes:
            if lane['from'] not in self.graph:
                self.graph[lane['from']] = []
            cost = self.calculate_distance(lane['from'], lane['to'])
            self.graph[lane['from']].append((lane['to'], cost))

    def calculate_distance(self, from_id: str, to_id: str) -> float:
        from_pos = self.vertices[from_id]
        to_pos = self.vertices[to_id]
        return sqrt((from_pos['x'] - to_pos['x'])**2 + 
                   (from_pos['y'] - to_pos['y'])**2)

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

def display_robots_and_destinations(nav_graph: NavigationGraph):
    """Display available robots and destinations"""
    print("\nAvailable Robots (named vertices with potential charger stations):")
    robots = {}
    for vid, vertex in nav_graph.vertices.items():
        name = vertex['meta']['name']
        if name:  
            charger = " (Charger)" if vertex['meta'].get('is_charger', False) else ""
            robots[vid] = f"{name}{charger}"
            print(f"ID: {vid} - {name}{charger}")
    
    print("\nPossible Destinations (all vertices):")
    for vid, vertex in nav_graph.vertices.items():
        name = vertex['meta']['name'] or "unnamed"
        charger = " (Charger)" if vertex['meta'].get('is_charger', False) else ""
        print(f"ID: {vid} - {name}{charger}")
    
    return robots

def main():
    nav_graph = NavigationGraph('./data/nav_graph_samples/nav_graph_1.json')
    path_finder = RobotPathFinder(nav_graph)
    
    robots = display_robots_and_destinations(nav_graph)
    
    while True:
        print("\nEnter 'q' to quit")
        start = input("Enter robot ID to start from (see list above): ").strip()
        if start.lower() == 'q':
            break
            
        if start not in robots:
            print("Invalid robot ID. Please select from the available robots.")
            continue
            
        goal = input("Enter destination vertex ID (see list above): ").strip()
        if goal.lower() == 'q':
            break
            
        if goal not in nav_graph.vertices:
            print("Invalid destination ID. Please select from the available vertices.")
            continue
            
        path, total_cost = path_finder.find_shortest_path(start, goal)
        
        if path:
            readable_path = []
            for vertex_id in path:
                name = nav_graph.vertices[vertex_id]['meta']['name'] or 'unnamed'
                readable_path.append(f"{vertex_id} ({name})")
            print(f"\nShortest path from {robots[start]} to vertex {goal}:")
            print(f"{' -> '.join(readable_path)}")
            print(f"Total distance: {total_cost:.2f} units")
            
            print("\nPath coordinates:")
            for vertex_id in path:
                v = nav_graph.vertices[vertex_id]
                print(f"Vertex {vertex_id} ({v['meta']['name'] or 'unnamed'}): ({v['x']}, {v['y']})")
        else:
            print(f"\nNo path found from {robots[start]} (ID: {start}) to vertex {goal}")
        
        print("\n" + "-"*50)

if __name__ == "__main__":
    main()