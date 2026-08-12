import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider, Button
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

class Meca5003DPrintingSimulator:
    def __init__(self):
        # Meca500 specifications (updated with correct dimensions)
        self.joint_limits = [
            (-np.pi, np.pi),    # Joint 1 (base rotation)
            (-2.6, 2.6),        # Joint 2 (shoulder)
            (-np.pi, np.pi),    # Joint 3 (elbow)
            (-np.pi, np.pi),    # Joint 4 (wrist 1)
            (-2.6, 2.6),        # Joint 5 (wrist 2)
            (-np.pi, np.pi)     # Joint 6 (wrist rotation)
        ]
        
        # Correct link lengths (in mm)
        self.link_lengths = {
            'base_height': 135,    # From base to joint 2
            'upper_arm': 135,      # Joint 2 to joint 3
            'forearm': 175,         # Joint 3 to joint 4 (includes wrist length)
            'wrist': 72            # From joint 6 to tool flange
        }
        
        # Initial joint angles
        self.joint_angles = [0, 0, 0, 0, 0, 0]
        
        # Printing parameters
        self.cube_size = 50  # mm
        self.layer_height = 2  # mm
        self.print_speed = 10  # mm/s
        
        # Create figure and 3D axis
        self.fig = plt.figure(figsize=(14, 10))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Set up UI controls
        self.setup_controls()
        
        # Initialize printing path
        self.generate_print_path()
        
        # Visualization objects
        self.robot_line = None
        self.end_effector_arrows = []
        self.extruder_visual = None
        self.printed_layers = []
        self.current_layer = None
        
        # Initial plot
        self.update_plot()
        
        plt.show()
    
    def dh_matrix(self, theta, d, a, alpha):
        """Denavit-Hartenberg transformation matrix"""
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        
        return np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])
    
    def forward_kinematics(self):
        """Calculate forward kinematics with correct extruder position"""
        # Corrected DH parameters for Meca500 with extruder at tool flange
        dh_params = [
            # theta, d, a, alpha
            [self.joint_angles[0], self.link_lengths['base_height'], 0, -np.pi/2],
            [self.joint_angles[1] + np.pi/2, 0, self.link_lengths['upper_arm'], 0],
            [self.joint_angles[2], 0, self.link_lengths['forearm'], -np.pi/2],
            [self.joint_angles[3], 0, 0, np.pi/2],
            [self.joint_angles[4], 0, 0, -np.pi/2],
            [self.joint_angles[5], self.link_lengths['wrist'], 0, 0]
        ]
        
        T = np.eye(4)
        transformations = [T.copy()]
        
        for params in dh_params:
            theta, d, a, alpha = params
            T = T @ self.dh_matrix(theta, d, a, alpha)
            transformations.append(T.copy())
        
        # Add extruder offset (50mm down from tool flange)
        extruder_offset = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, -50],  # Negative Z in tool frame
            [0, 0, 0, 1]
        ])
        T_extruder = T @ extruder_offset
        transformations.append(T_extruder)
        
        positions = [T[:3, 3] for T in transformations]
        return np.array(positions), T_extruder
    
    def generate_print_path(self):
        """Generate path for printing a cube"""
        self.path_points = []
        self.path_angles = []
        
        # Cube dimensions
        cube_x, cube_y, cube_z = self.cube_size, self.cube_size, self.cube_size
        
        # Number of layers
        num_layers = int(cube_z / self.layer_height)
        
        # Generate path for each layer
        for layer in range(num_layers):
            z = layer * self.layer_height + 5  # Start 5mm above bed
            
            # Outer perimeter
            path = [
                [0, 0, z], [cube_x, 0, z], 
                [cube_x, cube_y, z], [0, cube_y, z], [0, 0, z]
            ]
            
            # Infill (zig-zag pattern)
            spacing = 10  # mm between infill lines
            num_lines = int(cube_y / spacing) - 1
            
            for i in range(1, num_lines + 1):
                y = i * spacing
                if i % 2 == 1:
                    path.append([0, y, z])
                    path.append([cube_x, y, z])
                else:
                    path.append([cube_x, y, z])
                    path.append([0, y, z])
            
            # Add to full path
            self.path_points.extend(path)
            
            # Orientation (nozzle pointing down)
            for _ in path:
                self.path_angles.append([0, np.pi/2, 0])  # Roll, Pitch, Yaw
    
    def inverse_kinematics(self, position, orientation):
        """Improved IK for Meca500 with correct extruder position"""
        x, y, z = position
        roll, pitch, yaw = orientation
        
        # Base rotation (joint 1)
        theta1 = np.arctan2(y, x)
        
        # Wrist position relative to base (accounting for wrist length)
        wrist_pos = np.array([x, y, z]) - np.array([
            self.link_lengths['wrist'] * np.cos(theta1) * np.cos(pitch),
            self.link_lengths['wrist'] * np.sin(theta1) * np.cos(pitch),
            self.link_lengths['wrist'] * np.sin(pitch)
        ])
        
        # Arm IK (joints 2-3)
        L1 = self.link_lengths['upper_arm']
        L2 = self.link_lengths['forearm']
        
        # Target in arm coordinate system
        x_arm = np.sqrt(wrist_pos[0]**2 + wrist_pos[1]**2)
        z_arm = wrist_pos[2] - self.link_lengths['base_height']
        
        D = np.sqrt(x_arm**2 + z_arm**2)
        
        # Law of cosines for elbow angle
        try:
            theta3 = np.pi - np.arccos((L1**2 + L2**2 - D**2)/(2*L1*L2))
        except:
            theta3 = 0  # If position unreachable
        
        # Shoulder angle
        alpha = np.arctan2(z_arm, x_arm)
        beta = np.arccos((L1**2 + D**2 - L2**2)/(2*L1*D))
        theta2 = alpha + beta - np.pi/2  # Adjusted for DH parameters
        
        # Wrist orientation (joints 4-6)
        theta4 = roll
        theta5 = pitch
        theta6 = yaw - theta1  # Compensate for base rotation
        
        return [theta1, theta2, theta3, theta4, theta5, theta6]
    
    def update_plot(self, frame=None):
        """Update the 3D plot with correct extruder position"""
        self.ax.clear()
        
        # Plot robot arm
        positions, end_effector = self.forward_kinematics()
        
        # Plot arm links (skip the extruder position for main arm)
        self.robot_line, = self.ax.plot(positions[:-1, 0], positions[:-1, 1], positions[:-1, 2], 
                                      'o-', markersize=8, linewidth=3, color='blue')
        
        # Plot extruder
        nozzle_length = 50
        nozzle_dir = end_effector[:3, 2]  # Z-axis of end effector
        nozzle_end = end_effector[:3, 3] + nozzle_dir * nozzle_length
        
        self.extruder_visual, = self.ax.plot(
            [end_effector[0, 3], nozzle_end[0]], 
            [end_effector[1, 3], nozzle_end[1]], 
            [end_effector[2, 3], nozzle_end[2]], 
            linewidth=3, color='black')
        
        # Plot end effector orientation
        length = 30
        x_axis = end_effector[:3, 0] * length
        y_axis = end_effector[:3, 1] * length
        z_axis = end_effector[:3, 2] * length
        ee_pos = end_effector[:3, 3]
        
        self.ax.quiver(ee_pos[0], ee_pos[1], ee_pos[2], 
                      x_axis[0], x_axis[1], x_axis[2], color='r')
        self.ax.quiver(ee_pos[0], ee_pos[1], ee_pos[2], 
                      y_axis[0], y_axis[1], y_axis[2], color='g')
        self.ax.quiver(ee_pos[0], ee_pos[1], ee_pos[2], 
                      z_axis[0], z_axis[1], z_axis[2], color='b')
        
        # Plot printing bed
        bed_size = 100
        bed_verts = [
            [-bed_size/2, -bed_size/2, 0],
            [bed_size/2, -bed_size/2, 0],
            [bed_size/2, bed_size/2, 0],
            [-bed_size/2, bed_size/2, 0]
        ]
        bed = Poly3DCollection([bed_verts], alpha=0.2, color='gray')
        self.ax.add_collection3d(bed)
        
        # Plot cube model
        cube_verts = [
            [0, 0, 0], [self.cube_size, 0, 0],
            [self.cube_size, self.cube_size, 0], [0, self.cube_size, 0],
            [0, 0, self.cube_size], [self.cube_size, 0, self.cube_size],
            [self.cube_size, self.cube_size, self.cube_size], [0, self.cube_size, self.cube_size]
        ]
        cube_edges = [
            [cube_verts[0], cube_verts[1], cube_verts[2], cube_verts[3]],
            [cube_verts[4], cube_verts[5], cube_verts[6], cube_verts[7]],
            [cube_verts[0], cube_verts[4]], [cube_verts[1], cube_verts[5]],
            [cube_verts[2], cube_verts[6]], [cube_verts[3], cube_verts[7]]
        ]
        cube = Poly3DCollection(cube_edges, linewidths=1, edgecolor='blue', alpha=0.2)
        self.ax.add_collection3d(cube)
        
        # Plot printed layers
        if self.printed_layers:
            for layer in self.printed_layers:
                self.ax.plot(layer[:, 0], layer[:, 1], layer[:, 2], 
                            linewidth=2, color='green')
        
        # Plot current layer being printed
        if self.current_layer is not None and frame is not None:
            if frame > 0:
                self.ax.plot(self.current_layer[:frame, 0], 
                            self.current_layer[:frame, 1], 
                            self.current_layer[:frame, 2], 
                            linewidth=2, color='red')
        
        # Plot settings
        max_range = 300
        self.ax.set_xlim([-max_range, max_range])
        self.ax.set_ylim([-max_range, max_range])
        self.ax.set_zlim([0, max_range])
        self.ax.set_xlabel('X (mm)')
        self.ax.set_ylabel('Y (mm)')
        self.ax.set_zlabel('Z (mm)')
        self.ax.set_title('Meca500 3D Printing Simulation (Corrected Kinematics)')
        self.ax.view_init(elev=30, azim=45)
    
    def setup_controls(self):
        """Create UI controls"""
        plt.subplots_adjust(bottom=0.25)
        
        # Create sliders
        self.sliders = []
        for i in range(6):
            ax_slider = plt.axes([0.25, 0.15 - 0.03*i, 0.6, 0.02])
            slider = Slider(
                ax_slider, f'J{i+1}', 
                self.joint_limits[i][0], self.joint_limits[i][1], 
                valinit=self.joint_angles[i], valfmt='%1.2f rad'
            )
            slider.on_changed(self.make_slider_update_func(i))
            self.sliders.append(slider)
        
        # Add control buttons
        reset_ax = plt.axes([0.7, 0.9, 0.1, 0.04])
        self.reset_button = Button(reset_ax, 'Reset', color='lightgoldenrodyellow')
        self.reset_button.on_clicked(self.reset_joints)
        
        print_ax = plt.axes([0.55, 0.9, 0.1, 0.04])
        self.print_button = Button(print_ax, 'Print Cube', color='lightgreen')
        self.print_button.on_clicked(self.start_printing)
    
    def make_slider_update_func(self, joint_idx):
        """Create slider update function"""
        def update(val):
            self.joint_angles[joint_idx] = val
            self.update_plot()
        return update
    
    def reset_joints(self, event):
        """Reset all joints to zero position"""
        for i in range(6):
            self.joint_angles[i] = 0
            self.sliders[i].set_val(0)
        self.update_plot()
    
    def start_printing(self, event):
        """Animate the cube printing process"""
        self.printed_layers = []
        self.current_layer = None
        
        # Group path points into layers
        layers = []
        current_layer = []
        last_z = None
        
        for point in self.path_points:
            if last_z is None:
                last_z = point[2]
            
            if point[2] != last_z:
                layers.append(np.array(current_layer))
                current_layer = []
                last_z = point[2]
            
            current_layer.append(point)
        
        if current_layer:
            layers.append(np.array(current_layer))
        
        # Animate each layer
        for i, layer in enumerate(layers):
            self.current_layer = layer
            
            # Calculate joint angles for each point in layer
            joint_angles = []
            for j, point in enumerate(layer):
                orientation = self.path_angles[j + sum(len(l) for l in layers[:i])]
                angles = self.inverse_kinematics(point, orientation)
                joint_angles.append(angles)
            
            # Create animation for this layer
            for point_idx in range(len(layer)):
                angles = joint_angles[point_idx]
                for j in range(6):
                    self.joint_angles[j] = angles[j]
                    self.sliders[j].set_val(angles[j])
                
                self.update_plot(point_idx)
                plt.pause(0.05)
            
            # Add completed layer to printed layers
            self.printed_layers.append(layer)
            self.current_layer = None
            self.update_plot()
        
        print("Cube printing complete!")

# Run the simulator
print("Starting Meca500 3D Printing Simulator with Corrected Kinematics...")
simulator = Meca5003DPrintingSimulator()