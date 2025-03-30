# Fleet Management System

## Overview
The Fleet Management System is a multi-robot coordination framework designed to efficiently navigate a structured environment. It prevents collisions using a reservation-based system, dynamically allocates tasks to robots, and ensures smooth traffic flow within a finite robot fleet.

### Demo Video
Watch the system in action: [Demo Video](https://drive.google.com/file/d/1ERsoPKPq1699kIbkUajCJkFb7umSyccb/view?usp=sharing)

## Features
- **Path Optimization** – Uses the A* algorithm to compute the shortest paths.
- **Collision Avoidance** – Prevents head-on collisions with a lane reservation mechanism.
- **Dynamic Task Allocation** – Assigns and reallocates tasks in real-time.
- **Finite Robot Management** – Optimized for handling a limited number of robots.
- **Traffic Management** – Dynamically reserves lanes to prevent congestion and deadlocks.
- **Graphical User Interface (GUI)** – A Tkinter-based visualization tool to monitor robot movements and traffic conditions.
- **Logging & Debugging** – Logs key events and robot activities for analysis and troubleshooting.

## Project Structure
```
├── data
│   └── nav_graph_samples
│       ├── nav_graph_1.json
│       ├── nav_graph_2.json
│       └── nav_graph_3.json
│
├── src
│   ├── gui
│   │   └── fleetmanagement.py  # Main GUI application
│   │
│   ├── logs
│   │   └── fleet_logs.txt  # Log file for system events
│   │
│   ├── main.py  # Entry point for the application
│   │
│   ├── models
│   │   ├── navigation_graph.py  # Handles navigation graph and JSON parsing
│   │   ├── robot_spec.py  # Defines robot specifications
│   │   └── traffic_manager.py  # Manages lane reservations and collision handling
│   │
│   ├── utils
│   │   └── robot_pathfinder.py  # A* algorithm for pathfinding
│
└── README.md
```

## Installation

### Prerequisites
- Python 3.8+ (Recommended: Python 3.13)
- Tkinter (Pre-installed with Python on most systems)
- Pip package manager

### Setup
Clone the repository and install the dependencies:
```sh
git clone https://github.com/your-repository/fleet-management-system.git
cd fleet-management-system
pip install -r requirements.txt
```

### Required Dependencies
Ensure the following dependencies are installed (listed in `requirements.txt`):
```sh
tkinter
```

## Usage

### Configuring the Navigation Graph
- Place JSON-based navigation graphs inside `data/nav_graph_samples/`.
- Modify `src/main.py` to use a different graph file if needed.

### Running the Application
```sh
python3 src/main.py
```
This will launch the GUI, where you can:
- View the navigation graph (nodes, lanes, and connectivity)
- Add and remove robots dynamically
- Assign tasks and observe real-time path updates
- Monitor traffic reservations to prevent deadlocks

## Logging & Debugging
All system activities, including robot movements and lane reservations, are logged in:
```sh
src/logs/fleet_logs.txt
```
These logs help analyze system behavior and troubleshoot potential issues.

