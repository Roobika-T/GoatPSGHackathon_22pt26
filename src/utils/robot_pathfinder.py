from typing import Dict, List, Tuple, Set
from heapq import heappush, heappop
from models.navigation_graph import NavigationGraph
from models.traffic_manager import TrafficManager
import logging 

logging.basicConfig(
    filename='src/logs/fleet_logs.txt',
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

class RobotPathFinder:
    def __init__(self, nav_graph: NavigationGraph, traffic_manager: TrafficManager):
        self.nav_graph = nav_graph
        self.traffic_manager = traffic_manager

    def find_shortest_path(self, start: str, goal: str, robot_id: str,
                           start_time: float = 0, speed: float = 1.0) -> Tuple[List[str], float, List[float]]:
        if start not in self.nav_graph.vertices or goal not in self.nav_graph.vertices:
            logging.info(f"Robot {robot_id}: Start {start} or goal {goal} not in vertices.")
            return [], 0.0, []

        logging.info(f"Robot {robot_id}: Finding path from {start} to {goal} at time {start_time}")
        open_set = [(0, start, [start], 0, [start_time])]
        closed_set = set()
        g_scores = {start: 0}

        while open_set:
            f_score, current, path, cost, arrival_times = heappop(open_set)

            if current == goal:
                logging.info(f"Robot {robot_id}: Path found: {path}")
                return path, cost, arrival_times

            if current in closed_set:
                continue

            closed_set.add(current)
            current_time = arrival_times[-1]

            neighbors = self.nav_graph.graph.get(current, [])
            logging.info(f"Robot {robot_id}: At vertex {current}, neighbors: {neighbors}")
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
                else:
                    logging.info(f"Robot {robot_id}: Cannot reserve lane {lane_id} from {current} to {next_node} at time {current_time}")

        logging.info(f"Robot {robot_id}: No path found from {start} to {goal}")
        return [], 0.0, []