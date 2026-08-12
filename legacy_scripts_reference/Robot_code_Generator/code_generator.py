import tkinter as tk
from tkinter import ttk, filedialog, messagebox

def generate_commands():
    try:
        # Get values from GUI
        initial_pose = [
            float(x_entry.get()),
            float(y_entry.get()),
            float(z_entry.get()),
            float(a_entry.get()),
            float(b_entry.get()),
            float(c_entry.get())
        ]
        rectangle_size = float(size_entry.get())
        z_step = float(zstep_entry.get())
        num_layers = int(layers_entry.get())
        
        # Generate commands
        commands = []
        x, y, z, a, b, c = initial_pose
        
        for layer in range(num_layers):
            current_z = z + (layer * z_step)
            
            # Rectangle path
            commands.append(f"MovePose({x:.3f}, {y:.3f}, {current_z:.3f}, {a:.3f}, {b:.3f}, {c:.3f})")
            commands.append(f"MovePose({x + rectangle_size:.3f}, {y:.3f}, {current_z:.3f}, {a:.3f}, {b:.3f}, {c:.3f})")
            commands.append(f"MovePose({x + rectangle_size:.3f}, {y + rectangle_size:.3f}, {current_z:.3f}, {a:.3f}, {b:.3f}, {c:.3f})")
            commands.append(f"MovePose({x:.3f}, {y + rectangle_size:.3f}, {current_z:.3f}, {a:.3f}, {b:.3f}, {c:.3f})")
            commands.append(f"MovePose({x:.3f}, {y:.3f}, {current_z:.3f}, {a:.3f}, {b:.3f}, {c:.3f})")
        
        # Save to file
        filepath = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("Text files", "*.txt")],
            title="Save MECA500 Commands"
        )
        
        if filepath:
            with open(filepath, 'w') as f:
                f.write("\n".join(commands))
            messagebox.showinfo("Success", f"Generated {len(commands)} commands\nSaved to:\n{filepath}")
            
    except ValueError as e:
        messagebox.showerror("Error", f"Invalid input: {e}")

# Create GUI
root = tk.Tk()
root.title("MECA500 Rectangle Path Generator")

frame = ttk.Frame(root, padding="20")
frame.grid()

# Initial Pose
ttk.Label(frame, text="Initial Pose (mm/deg)").grid(column=0, row=0, columnspan=2, pady=5)
ttk.Label(frame, text="X:").grid(column=0, row=1, sticky=tk.E)
x_entry = ttk.Entry(frame, width=10)
x_entry.grid(column=1, row=1)
x_entry.insert(0, "212.563")

ttk.Label(frame, text="Y:").grid(column=0, row=2, sticky=tk.E)
y_entry = ttk.Entry(frame, width=10)
y_entry.grid(column=1, row=2)
y_entry.insert(0, "-61.575")

ttk.Label(frame, text="Z:").grid(column=0, row=3, sticky=tk.E)
z_entry = ttk.Entry(frame, width=10)
z_entry.grid(column=1, row=3)
z_entry.insert(0, "70.599")

ttk.Label(frame, text="A:").grid(column=0, row=4, sticky=tk.E)
a_entry = ttk.Entry(frame, width=10)
a_entry.grid(column=1, row=4)
a_entry.insert(0, "-94.998")

ttk.Label(frame, text="B:").grid(column=0, row=5, sticky=tk.E)
b_entry = ttk.Entry(frame, width=10)
b_entry.grid(column=1, row=5)
b_entry.insert(0, "79.029")

ttk.Label(frame, text="C:").grid(column=0, row=6, sticky=tk.E)
c_entry = ttk.Entry(frame, width=10)
c_entry.grid(column=1, row=6)
c_entry.insert(0, "33.056")

# Parameters
ttk.Label(frame, text="Parameters").grid(column=0, row=7, columnspan=2, pady=(15,5))

ttk.Label(frame, text="Rectangle Size (mm):").grid(column=0, row=8, sticky=tk.E)
size_entry = ttk.Entry(frame, width=10)
size_entry.grid(column=1, row=8)
size_entry.insert(0, "40.0")

ttk.Label(frame, text="Z Step per Layer (mm):").grid(column=0, row=9, sticky=tk.E)
zstep_entry = ttk.Entry(frame, width=10)
zstep_entry.grid(column=1, row=9)
zstep_entry.insert(0, "2.0")

ttk.Label(frame, text="Number of Layers:").grid(column=0, row=10, sticky=tk.E)
layers_entry = ttk.Entry(frame, width=10)
layers_entry.grid(column=1, row=10)
layers_entry.insert(0, "20")

# Generate Button
generate_btn = ttk.Button(frame, text="Generate Commands", command=generate_commands)
generate_btn.grid(column=0, row=11, columnspan=2, pady=20)

root.mainloop()