"""
EPFL Semester project
Author: Joao Pedro RODRIGUES LOPES // Laboratory of Intelligent Systems 
Date: January 5, 2024
Description: GeneticAlgorithm-based path planner for a morphing wing drone.
"""


import tkinter as tk
from tkinter import filedialog, messagebox
from obstacles import Circles
from plot import draw_everything
from genetic_algo import run_algo
import numpy as np

# Parameters
num_circles = 15
radius_range = (0.6 ,2.0)
map_size = (30,30)

# Global variables to store current map
current_map = None
current_map_obj = None
save_map_var = None  # Will be created later
last_simulation_results = None  # Store results from multi-simulations
last_circles = None
last_x_init = None
last_x_goal = None

def launch(circles, x_init, x_goal, num_simulations=1):
    params = [int(entry1.get()), int(entry2.get()), var2.get()]
    root.destroy()
    
    if num_simulations == 1:
        # Single simulation 
        bestIndv, perf = run_algo(x_init, x_goal, circles, params)
        if bestIndv:
            final_score = bestIndv.score
            flight_time = bestIndv.flight_time
            
            collision_penalty = bestIndv.obsCrossed
            print("Tangency sequence:", bestIndv.tangency)
            print('Obstacles crossed ' + str(len(bestIndv.obsCrossed))) 
        else:
            final_score = float('inf')
            flight_time = 0
            collision_penalty = 0
        print(f"Final Score: {final_score:.2f}")
        print(f"Flight Time: {flight_time:.2f} seconds")
        print(f"Collision Penalty: {collision_penalty}")


        draw_everything(map_size, circles, x_init, x_goal, bestIndv, perf)

    else:
        # Multiple simulations
        print("\n" + "="*80)
        print(f"Running {num_simulations} simulations on the same map")
        print("="*80 + "\n")
        
        results = []
        for sim_num in range(1, num_simulations + 1):
            print(f"\n--- Simulation {sim_num}/{num_simulations} ---")
            bestIndv, perf = run_algo(x_init, x_goal, circles, params)
            
            # Extract final performance metrics
            if bestIndv:
                final_score = bestIndv.score
                flight_time = bestIndv.flight_time
                collision_penalty = len(bestIndv.obsCrossed)
                print("Tangency sequence:", bestIndv.tangency)
            else:
                final_score = float('inf')
                flight_time = 0
                collision_penalty = 0
            results.append({
                'sim_num': sim_num,
                'final_score': final_score,
                'flight_time': flight_time,
                'collision_penalty': collision_penalty,
                'bestIndv': bestIndv,
                'perf': perf
            })
            print(f"Final Score: {final_score:.2f}")
            print(f"Flight Time: {flight_time:.2f} seconds")
            print(f"Collision Penalty: {collision_penalty}")

        
        # Print summary statistics
        print("\n" + "="*80)
        print("SIMULATION SUMMARY")
        print("="*80)
        print(f"{'Sim #':<6} {'Score':<12} {'Flight Time':<15} {'Collisions':<12}")
        print("-"*80)
        
        scores = []
        times = []
        collisions = []
        
        for result in results:
            print(f"{result['sim_num']:<6} {result['final_score']:<12.2f} {result['flight_time']:<15.2f} {result['collision_penalty']:<12}")
            scores.append(result['final_score'])
            times.append(result['flight_time'])
            collisions.append(result['collision_penalty'])
        
        print("-"*80)
        print(f"{'BEST:':<6} {min(scores):<12.2f} {min(times):<15.2f} {min(collisions):<12}")
        print(f"{'WORST:':<6} {max(scores):<12.2f} {max(times):<15.2f} {max(collisions):<12}")
        print(f"{'MEAN:':<6} {np.mean(scores):<12.2f} {np.mean(times):<15.2f} {np.mean(collisions):<12.2f}")
        print(f"{'STDEV:':<6} {np.std(scores):<12.2f} {np.std(times):<15.2f} {np.std(collisions):<12.2f}")
        print("="*80 + "\n")
        
        # Store results globally for later plotting
        global last_simulation_results, last_circles, last_x_init, last_x_goal
        last_simulation_results = results
        last_circles = circles
        last_x_init = x_init
        last_x_goal = x_goal
        
        # Create a window to select which simulation to plot
        show_simulation_selector(results)

    # Save if checkbox is enabled
    if save_map_var.get():
        circles_to_save, x_init_to_save, x_goal_to_save = current_map
        current_map_obj.save_map("exported_map.txt", circles_to_save, x_init_to_save, x_goal_to_save)
        print("Map saved to exported_map.txt")

def saveMap():
    """Save the current map to a file."""
    global current_map, current_map_obj
    if current_map is None:
        messagebox.showwarning("No Map", "Please generate or load a map first.")
        return
    
    circles, x_init, x_goal = current_map
    file_path = filedialog.asksaveasfilename(
        title="Save Map As",
        defaultextension=".txt",
        filetypes=[("Map files", "*.txt"), ("All files", "*.*")]
    )
    
    if file_path:
        current_map_obj.save_map(file_path, circles, x_init, x_goal)
        messagebox.showinfo("Success", f"Map saved to:\n{file_path}")


def openMap(root):
    """Load map from file and run multiple simulations"""
    global current_map, current_map_obj
    map_obj = Circles()
    file_path = filedialog.askopenfilename(title="Select a File", filetypes=[("Map files", "*.txt"), ("All files", "*.*")])
    if file_path:
        circles, x_init, x_goal = map_obj.open_map(file_path)
        current_map = (circles, x_init, x_goal)
        current_map_obj = map_obj
        num_sims = int(entry4.get())
        launch(circles, x_init, x_goal, num_simulations=num_sims)

def randomMap(root):
    """Generate random map and run multiple simulations"""
    global current_map, current_map_obj
    map_obj = Circles()
    circles = map_obj.generate_non_overlapping_circles(int(entry3.get()), radius_range, map_size, var1.get())
    x_init = map_obj.point_on_circle(circles[0],  20)
    x_goal = map_obj.point_on_circle(circles[-1], 45)
    
    # Store the current map
    current_map = (circles, x_init, x_goal)
    current_map_obj = map_obj
    
    num_sims = int(entry4.get())
    launch(circles, x_init, x_goal, num_simulations=num_sims)

def show_simulation_selector(results):
    """Create a window to select which simulation to plot"""
    selector_window = tk.Tk()
    selector_window.title("Select Simulation to Plot")
    selector_window.geometry("400x300")
    
    title_label = tk.Label(selector_window, text="Select a Simulation to Plot", font=("Arial", 12, "bold"))
    title_label.pack(pady=10)
    
    # Create frame for listbox
    frame = tk.Frame(selector_window)
    frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
    
    # Create listbox with scrollbar
    scrollbar = tk.Scrollbar(frame)
    scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
    
    listbox = tk.Listbox(frame, yscrollcommand=scrollbar.set)
    listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
    scrollbar.config(command=listbox.yview)
    
    # Populate listbox
    for result in results:
        sim_num = result['sim_num']
        score = result['final_score']
        flight_time = result['flight_time']
        collision_penalty = result['collision_penalty']
        listbox.insert(tk.END, f"Sim {sim_num}: Score={score:.2f}  Time={flight_time:.2f}s  Collisions={collision_penalty}")
    
    # Pre-select best simulation
    best_idx = min(range(len(results)), key=lambda i: results[i]['final_score'])
    listbox.selection_set(best_idx)
    listbox.see(best_idx)
    
    def plot_selected():
        """Plot the selected simulation"""
        selection = listbox.curselection()
        if selection:
            idx = selection[0]
            result = results[idx]
            print("Tangency sequence:", result['bestIndv'].tangency)
            draw_everything(map_size, last_circles, last_x_init, last_x_goal, 
                          result['bestIndv'], result['perf'])
    
    def plot_best():
        """Plot the best simulation"""
        best_idx = min(range(len(results)), key=lambda i: results[i]['final_score'])
        result = results[best_idx]
        draw_everything(map_size, last_circles, last_x_init, last_x_goal, 
                      result['bestIndv'], result['perf'])
    
    # Create buttons
    button_frame = tk.Frame(selector_window)
    button_frame.pack(pady=10)
    
    plot_btn = tk.Button(button_frame, text="Plot Selected", command=plot_selected)
    plot_btn.pack(side=tk.LEFT, padx=5)
    
    best_btn = tk.Button(button_frame, text="Plot Best", command=plot_best)
    best_btn.pack(side=tk.LEFT, padx=5)
    
    close_btn = tk.Button(button_frame, text="Close", command=selector_window.destroy)
    close_btn.pack(side=tk.LEFT, padx=5)
    
    selector_window.mainloop()



# Create the main window
root = tk.Tk()
root.title("Genetic path planning algorithm")

# Create and configure the radio buttons for the first set
var1 = tk.StringVar(value="Crescent")
radio_button1_1 = tk.Radiobutton(root, text="Crescent", variable=var1, value="Crescent")
radio_button1_2 = tk.Radiobutton(root, text="Distance", variable=var1, value="Distance")
radio_button1_3 = tk.Radiobutton(root, text="Random", variable=var1, value="Random")

# Create and configure the radio buttons for the second set
var2 = tk.StringVar(value="Rank")
radio_button2_2 = tk.Radiobutton(root, text="Roulette", variable=var2, value="Roulette")
radio_button2_1 = tk.Radiobutton(root, text="Best half", variable=var2, value="BestHalf")
radio_button2_3 = tk.Radiobutton(root, text="Tournament", variable=var2, value="Tournament")
radio_button2_4 = tk.Radiobutton(root, text="Rank", variable=var2, value="Rank")

# Create labels and entry widgets for numerical inputs
label1 = tk.Label(root, text="Population:")
entry1 = tk.Entry(root)
entry1.insert(0, "100") 
label2 = tk.Label(root, text="Generations:")
entry2 = tk.Entry(root)
entry2.insert(0, "100") 
label3 = tk.Label(root, text="N_Obstacles:")
entry3 = tk.Entry(root)
entry3.insert(0, "25")
label4 = tk.Label(root, text="Simulations:")
entry4 = tk.Entry(root)
entry4.insert(0, "1")

# Create checkbox for saving
save_map_var = tk.BooleanVar(value=False)
checkbutton_save = tk.Checkbutton(root, text="Save as exported_map", variable=save_map_var)

# Create buttons
button1 = tk.Button(root, text="Open Map", command = lambda:openMap(root))
button2 = tk.Button(root, text="Random Map", command = lambda:randomMap(root))

# Create labels for radio button groups
label_group1 = tk.Label(root, text="Obstacles sorting method")
label_group2 = tk.Label(root, text="Selection strategy")

# Arrange widgets in a column
label_group1.grid(row=0, column=0, sticky="w")
radio_button1_1.grid(row=1, column=0, sticky="w")
radio_button1_2.grid(row=2, column=0, sticky="w") 
radio_button1_3.grid(row=3, column=0, sticky="w")

label_group2.grid(row=0, column=1, sticky="w")
radio_button2_1.grid(row=1, column=1, sticky="w")
radio_button2_2.grid(row=2, column=1, sticky="w")
radio_button2_3.grid(row=3, column=1, sticky="w")
radio_button2_4.grid(row=4, column=1, sticky="w")

label1.grid(row=6, column=0, sticky="w")
entry1.grid(row=7, column=0, sticky="w")

label2.grid(row=6, column=1, sticky="w")
entry2.grid(row=7, column=1, sticky="w")
label3.grid(row=6, column=2, sticky="w")
entry3.grid(row=7, column=2, sticky="w")
label4.grid(row=8, column=0, sticky="w")
entry4.grid(row=9, column=0, sticky="w")
checkbutton_save.grid(row=9, column=2, columnspan=2, sticky="w")

button1.grid(row=1, column=2)
button2.grid(row=2, column=2)

# Start the main loop
root.mainloop()


