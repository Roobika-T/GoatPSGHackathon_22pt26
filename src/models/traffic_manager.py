class TrafficManager:
    def __init__(self):
        self.lane_reservations = {}
        self.current_time = 0
        self.waiting_queues = {}
        self.lane_directions = {}

    def reserve_lane(self, lane_id: str, robot_id: str, start_time: float, end_time: float) -> bool:
        if lane_id not in self.lane_reservations:
            self.lane_reservations[lane_id] = []

        opposite_lane_id = lane_id.split('-')[1] + '-' + lane_id.split('-')[0]
        if opposite_lane_id in self.lane_reservations:
            for res_robot_id, res_start, res_end in self.lane_reservations[opposite_lane_id]:
                if max(start_time, res_start) < min(end_time, res_end):
                    print(f"Robot {robot_id}: Lane {lane_id} blocked by opposite lane {opposite_lane_id} (Robot {res_robot_id})")
                    return False

        for res_robot_id, res_start, res_end in self.lane_reservations[lane_id]:
            if res_robot_id == robot_id:
                continue
            if max(start_time, res_start) < min(end_time, res_end):
                print(f"Robot {robot_id}: Lane {lane_id} already reserved by Robot {res_robot_id} from {res_start} to {res_end}")
                return False

        print(f"Robot {robot_id}: Reserved lane {lane_id} from {start_time} to {end_time}")
        self.lane_reservations[lane_id].append((robot_id, start_time, end_time))
        self.lane_directions[lane_id] = robot_id
        return True

    def add_to_waiting_queue(self, vertex_id: str, robot_id: str, arrival_time: float):
        if vertex_id not in self.waiting_queues:
            self.waiting_queues[vertex_id] = []
        self.waiting_queues[vertex_id].append((robot_id, arrival_time))

    def process_waiting_queue(self, vertex_id: str) -> str:
        if vertex_id not in self.waiting_queues or not self.waiting_queues[vertex_id]:
            return None
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
                print(f"Lane {lane_id} reservations cleared at time {self.current_time}")
                del self.lane_reservations[lane_id]
                if lane_id in self.lane_directions:
                    del self.lane_directions[lane_id]
