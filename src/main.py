from gui.fleetmanagement import FleetManagementApp
import tkinter as tk
import logging 
import os

logging.basicConfig(
    filename='src/logs/fleet_logs.txt',
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

def main(json_path: str, spawn_prefix: str = "m"):
    root = tk.Tk()
    app = FleetManagementApp(root, json_path, max_robots=5, spawn_prefix=spawn_prefix)
    root.mainloop()

if __name__ == "__main__":

    json_files = [
        'nav_graph_1.json',
        'nav_graph_2.json',
        'nav_graph_3.json'
    ]
    base_path = 'data/nav_graph_samples'
    selected_file = json_files[0]
    json_path = os.path.join(base_path, selected_file)

    spawn_prefix = "m" if "nav_graph_1" in selected_file or "nav_graph_2" in selected_file else "p"
    
    print(f"Attempting to load JSON file: {json_path}")
    main(json_path, spawn_prefix)