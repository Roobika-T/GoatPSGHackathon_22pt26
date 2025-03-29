from dataclasses import dataclass

@dataclass
class RobotSpec:
    id: str
    start_vertex: str
    color: str
    speed: float
    battery: float = 100.0
