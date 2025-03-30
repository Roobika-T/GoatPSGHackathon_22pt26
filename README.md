Fleet Management System
The Fleet Management System is a multi-robot coordination framework designed to navigate a structured environment efficiently. It ensures collision-free movement using lane reservations, queue-based waiting mechanisms, and dynamic task allocation. The system is built with a focus on scalability, real-time monitoring, and optimal pathfinding for a finite number of robots.

Demo Video
Watch the system in action:
Demo Video

Features
Path Optimization – Uses the A* algorithm to determine the shortest path.

Collision Avoidance – Prevents head-on collisions with a reservation-based system.

Dynamic Task Allocation – Assigns and reallocates tasks to robots in real time.

Finite Robot Management – Efficiently handles a limited number of robots without overloading computational resources.

Traffic Management – Dynamically reserves lanes to optimize movement and prevent deadlocks.

Graphical User Interface (GUI) – Uses Tkinter to provide a real-time visualization of robot movement and traffic.

Logging & Debugging – Tracks key events, system actions, and robot movements for debugging and analysis.

Project Structure
The repository is organized as follows:

data/nav_graph_samples/ – Contains sample JSON navigation graphs.

src/gui/fleetmanagement.py – Main GUI application for visualization.

src/logs/fleet_logs.txt – Stores system event logs.

src/main.py – Entry point for the application.

src/models/navigation_graph.py – Handles navigation graph logic.

src/models/robot_spec.py – Defines robot specifications.

src/models/traffic_manager.py – Manages lane reservations and collision avoidance.

src/utils/robot_pathfinder.py – Implements the A* pathfinding algorithm.

Installation
Prerequisites
Python 3.8 or later (recommended: Python 3.13)

Tkinter (pre-installed with most Python distributions)

Pip package manager

Setup
To install and run the system:

Clone the repository:
git clone https://github.com/your-repository/fleet-management-system.git

Navigate to the project directory:
cd fleet-management-system

Install dependencies:
pip install -r requirements.txt

Configuration
The navigation graphs should be placed in the data/nav_graph_samples/ directory.

If needed, modify src/main.py to specify a different graph file.

Running the Application
To start the system, use:

python3 src/main.py

Once the GUI opens, users can:

View the navigation graph with nodes, lanes, and connectivity.

Dynamically add or remove robots.

Assign tasks and monitor real-time updates.

Observe traffic management and lane reservations to prevent deadlocks.

Logging & Debugging
All important system events, including robot movements and lane reservations, are logged in src/logs/fleet_logs.txt. These logs can be useful for debugging and analyzing system behavior.
