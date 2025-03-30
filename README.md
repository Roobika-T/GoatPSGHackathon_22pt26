# Fleet Management System

The **Fleet Management System** is a multi-robot coordination framework designed to navigate a structured environment efficiently. It ensures collision-free movement through **lane reservations**, **queue-based waiting mechanisms**, and **dynamic task allocation**. The system is built with a focus on **scalability**, **real-time monitoring**, and **optimal pathfinding** for a finite number of robots.

## **Demo Video**  
[Watch the demo here](https://drive.google.com/file/d/1ERsoPKPq1699kIbkUajCJkFb7umSyccb/view?usp=sharing)

## **Features**

- **Path Optimization** – Implements the **A* algorithm** for computing the shortest path.
- **Collision Avoidance** – Head-on collisions are prevented through a **reservation-based system**.
- **Dynamic Task Allocation** – Robots are assigned tasks in real-time, with dynamic reallocation when necessary.
- **Finite Robot Management** – The system efficiently handles a limited number of robots without overloading computational resources.
- **Traffic Management** – Lanes are reserved dynamically to ensure efficient movement and prevent deadlocks.
- **Graphical User Interface (GUI)** – A Tkinter-based visualization tool displays real-time robot movement and traffic conditions.
- **Logging & Debugging** – Key events, system actions, and robot movements are logged for analysis and debugging.

---

## **Project Structure**

```
.
├── data
│   └── nav_graph_samples
│       ├── nav_graph_1.json
│       ├── nav_graph_2.json
│       └── nav_graph_3.json
├── src
│   ├── gui
│   │   └── fleetmanagement.py      # Main GUI application
│   ├── logs
│   │   └── fleet_logs.txt          # Log file for system events
│   ├── main.py                     # Entry point for the application
│   ├── models
│   │   ├── navigation_graph.py     # Handles navigation graph and JSON parsing
│   │   ├── robot_spec.py           # Robot specification module
│   │   └── traffic_manager.py      # Manages lane reservations and collision handling
│   └── utils
│       └── robot_pathfinder.py     # A* algorithm for pathfinding
└── README.md
```

---

## **Installation**

### **Prerequisites**

- Python **3.8+** (Recommended: Python 3.13)
- Tkinter (Pre-installed with Python on most systems)
- Pip package manager

### **Setup**

Clone the repository and install the required dependencies:

```bash
git clone https://github.com/your-repository/fleet-management-system.git
cd fleet-management-system
pip install -r requirements.txt
```

#### **Example `requirements.txt`**

```
tkinter
```



---

## **Usage**

### **1. Configure the Navigation Graph**
- Place the required JSON-based navigation graphs in `data/nav_graph_samples/`.
- Modify `src/main.py` if a different graph file needs to be used.

### **2. Run the Application**

```bash
python3 src/main.py
```

This will launch the **GUI**, where users can:

- View the **navigation graph** (nodes, lanes, and connectivity).
- Add and remove robots dynamically.
- Assign tasks and observe **real-time path updates**.
- Monitor **traffic reservations** to prevent deadlocks.

---

## **Logging & Debugging**

All **robot movements, lane reservations, and system events** are logged in:

`src/logs/fleet_logs.txt`

These logs provide valuable insights into system behavior and can be used for debugging and performance analysis.

---

