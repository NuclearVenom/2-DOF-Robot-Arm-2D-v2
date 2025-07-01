"""
Getting slower as it aproches the target.
add another arm. (tried earlier but failed).
"""

import tkinter as tk
from tkinter import ttk
import math

class RobotArmSimulator:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Arm Simulator")
        # self.root.geometry("800x600")
        self.root.state("zoomed")
        
        # Arm parameters
        self.L1 = 150  # Upper arm length
        self.L2 = 120  # Forearm length
        self.base_x = 400  # Base position
        self.base_y = 350
        
        # Current joint angles (radians)
        self.theta1 = math.pi/4  # Shoulder angle
        self.theta2 = math.pi/3  # Elbow angle
        
        # Target angles
        self.target_theta1 = self.theta1
        self.target_theta2 = self.theta2
        
        # Target position
        self.target_x = None
        self.target_y = None
        
        # Animation parameters
        self.speed = 0.05  # Animation speed
        self.moving = False
        self.tolerance = 0.05  # Angle tolerance for reaching target
        
        self.setup_gui()
        self.update_display()
        self.animate()
    
    def setup_gui(self):
        # Main frame
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Canvas for arm visualization
        self.canvas = tk.Canvas(main_frame, width=1200, height=1000, bg='black', relief=tk.SUNKEN, bd=2)
        self.canvas.pack(side=tk.LEFT)
        
        # Bind mouse click to canvas
        self.canvas.bind("<Button-1>", self.on_canvas_click)
        
        # Control panel
        control_frame = ttk.Frame(main_frame)
        control_frame.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Title
        title_label = ttk.Label(control_frame, text="Robot Arm Control", font=('Arial', 14, 'bold'))
        title_label.pack(pady=(0, 20))
        
        # Speed control
        speed_frame = ttk.LabelFrame(control_frame, text="Speed Control", padding=10)
        speed_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Label(speed_frame, text="Animation Speed:").pack()
        self.speed_var = tk.DoubleVar(value=self.speed * 100)
        speed_scale = ttk.Scale(speed_frame, from_=1, to=20, variable=self.speed_var, orient=tk.HORIZONTAL)
        speed_scale.pack(fill=tk.X, pady=5)
        speed_scale.bind("<Motion>", self.update_speed)
        
        self.speed_label = ttk.Label(speed_frame, text=f"Speed: {self.speed*100:.1f}%")
        self.speed_label.pack()
        
        # Joint angles display
        angles_frame = ttk.LabelFrame(control_frame, text="Joint Angles", padding=10)
        angles_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.theta1_label = ttk.Label(angles_frame, text="Shoulder: 0.0°")
        self.theta1_label.pack(anchor=tk.W)
        
        self.theta2_label = ttk.Label(angles_frame, text="Elbow: 0.0°")
        self.theta2_label.pack(anchor=tk.W)
        
        # End effector position
        position_frame = ttk.LabelFrame(control_frame, text="End Effector Position", padding=10)
        position_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.pos_x_label = ttk.Label(position_frame, text="X: 0.0")
        self.pos_x_label.pack(anchor=tk.W)
        
        self.pos_y_label = ttk.Label(position_frame, text="Y: 0.0")
        self.pos_y_label.pack(anchor=tk.W)
        
        # Target position
        target_frame = ttk.LabelFrame(control_frame, text="Target Position", padding=10)
        target_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.target_x_label = ttk.Label(target_frame, text="X: None")
        self.target_x_label.pack(anchor=tk.W)
        
        self.target_y_label = ttk.Label(target_frame, text="Y: None")
        self.target_y_label.pack(anchor=tk.W)
        
        # Status
        status_frame = ttk.LabelFrame(control_frame, text="Status", padding=10)
        status_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.status_label = ttk.Label(status_frame, text="Ready", foreground="green")
        self.status_label.pack()
        
        # Instructions
        instructions_frame = ttk.LabelFrame(control_frame, text="Instructions", padding=10)
        instructions_frame.pack(fill=tk.X)
        
        instructions_text = """Click anywhere on the canvas 
to set a target position.

The robot arm will use inverse 
kinematics to reach the target.

Red circle = Target
Green circle = End effector
Blue lines = Arm segments"""
        
        ttk.Label(instructions_frame, text=instructions_text, justify=tk.LEFT, 
                 font=('Arial', 8)).pack(anchor=tk.W)
    
    def update_speed(self, event=None):
        self.speed = self.speed_var.get() / 100
        self.speed_label.config(text=f"Speed: {self.speed*100:.1f}%")
    
    def on_canvas_click(self, event):
        # Convert canvas coordinates to arm coordinates
        x = event.x - self.base_x
        y = self.base_y - event.y  # Flip Y axis
        
        # Check if target is reachable
        distance = math.sqrt(x*x + y*y)
        if distance > self.L1 + self.L2:
            self.status_label.config(text="Target unreachable!", foreground="red")
            return
        
        if distance < abs(self.L1 - self.L2):
            self.status_label.config(text="Target too close!", foreground="red")
            return
        
        # Set target position
        self.target_x = x
        self.target_y = y
        
        # Calculate inverse kinematics
        target_angles = self.inverse_kinematics(x, y)
        if target_angles:
            self.target_theta1, self.target_theta2 = target_angles
            self.moving = True
            self.status_label.config(text="Moving to target...", foreground="orange")
        else:
            self.status_label.config(text="Invalid target!", foreground="red")
    
    def inverse_kinematics(self, x, y):
        """Calculate joint angles for given end effector position"""
        try:
            # Distance from base to target
            r = math.sqrt(x*x + y*y)
            
            # Check reachability
            if r > self.L1 + self.L2 or r < abs(self.L1 - self.L2):
                return None
            
            # Calculate theta2 (elbow angle)
            cos_theta2 = (r*r - self.L1*self.L1 - self.L2*self.L2) / (2 * self.L1 * self.L2)
            cos_theta2 = max(-1, min(1, cos_theta2))  # Clamp to valid range
            theta2 = math.acos(cos_theta2)
            
            # Calculate theta1 (shoulder angle)
            alpha = math.atan2(y, x)
            beta = math.atan2(self.L2 * math.sin(theta2), self.L1 + self.L2 * math.cos(theta2))
            theta1 = alpha - beta
            
            return theta1, theta2
            
        except (ValueError, ZeroDivisionError):
            return None
    
    def forward_kinematics(self, theta1, theta2):
        """Calculate end effector position from joint angles"""
        # Elbow position
        elbow_x = self.L1 * math.cos(theta1)
        elbow_y = self.L1 * math.sin(theta1)
        
        # End effector position
        end_x = elbow_x + self.L2 * math.cos(theta1 + theta2)
        end_y = elbow_y + self.L2 * math.sin(theta1 + theta2)
        
        return elbow_x, elbow_y, end_x, end_y
    
    def animate(self):
        if self.moving:
            # Calculate angle differences
            diff1 = self.target_theta1 - self.theta1
            diff2 = self.target_theta2 - self.theta2
            
            # Check if we've reached the target
            if abs(diff1) < self.tolerance and abs(diff2) < self.tolerance:
                self.theta1 = self.target_theta1
                self.theta2 = self.target_theta2
                self.moving = False
                self.status_label.config(text="Target reached!", foreground="green")
            else:
                # Move towards target
                self.theta1 += diff1 * self.speed
                self.theta2 += diff2 * self.speed
        
        self.update_display()
        self.root.after(50, self.animate)  # 20 FPS
    
    def update_display(self):
        # Clear canvas
        self.canvas.delete("all")
        
        # Draw coordinate system
        self.canvas.create_line(0, self.base_y, 600, self.base_y, fill="lightgray", width=1)
        self.canvas.create_line(self.base_x, 0, self.base_x, 400, fill="lightgray", width=1)
        
        # Draw workspace boundary
        workspace_radius = self.L1 + self.L2
        self.canvas.create_oval(self.base_x - workspace_radius, self.base_y - workspace_radius,
                               self.base_x + workspace_radius, self.base_y + workspace_radius,
                               outline="lightblue", width=1, dash=(5, 5))
        
        # Calculate arm positions
        elbow_x, elbow_y, end_x, end_y = self.forward_kinematics(self.theta1, self.theta2)
        
        # Convert to canvas coordinates
        elbow_canvas_x = self.base_x + elbow_x
        elbow_canvas_y = self.base_y - elbow_y
        end_canvas_x = self.base_x + end_x
        end_canvas_y = self.base_y - end_y
        
        # Draw arm segments
        # Upper arm
        self.canvas.create_line(self.base_x, self.base_y, elbow_canvas_x, elbow_canvas_y,
                               fill="blue", width=6, capstyle=tk.ROUND)
        # Forearm
        self.canvas.create_line(elbow_canvas_x, elbow_canvas_y, end_canvas_x, end_canvas_y,
                               fill="blue", width=6, capstyle=tk.ROUND)
        
        # Draw joints
        # Base joint
        self.canvas.create_oval(self.base_x-8, self.base_y-8, self.base_x+8, self.base_y+8,
                               fill="red", outline="darkred", width=2)
        # Elbow joint
        self.canvas.create_oval(elbow_canvas_x-6, elbow_canvas_y-6, 
                               elbow_canvas_x+6, elbow_canvas_y+6,
                               fill="orange", outline="darkorange", width=2)
        # End effector
        self.canvas.create_oval(end_canvas_x-8, end_canvas_y-8, 
                               end_canvas_x+8, end_canvas_y+8,
                               fill="green", outline="darkgreen", width=2)
        
        # Draw target if set
        if self.target_x is not None and self.target_y is not None:
            target_canvas_x = self.base_x + self.target_x
            target_canvas_y = self.base_y - self.target_y
            self.canvas.create_oval(target_canvas_x-10, target_canvas_y-10,
                                   target_canvas_x+10, target_canvas_y+10,
                                   outline="red", width=3, dash=(3, 3))
            # Draw crosshair
            self.canvas.create_line(target_canvas_x-15, target_canvas_y, 
                                   target_canvas_x+15, target_canvas_y, fill="red", width=2)
            self.canvas.create_line(target_canvas_x, target_canvas_y-15, 
                                   target_canvas_x, target_canvas_y+15, fill="red", width=2)
        
        # Update labels
        self.theta1_label.config(text=f"Shoulder: {math.degrees(self.theta1):.1f}°")
        self.theta2_label.config(text=f"Elbow: {math.degrees(self.theta2):.1f}°")
        self.pos_x_label.config(text=f"X: {end_x:.1f}")
        self.pos_y_label.config(text=f"Y: {end_y:.1f}")
        
        if self.target_x is not None:
            self.target_x_label.config(text=f"X: {self.target_x:.1f}")
            self.target_y_label.config(text=f"Y: {self.target_y:.1f}")

def main():
    root = tk.Tk()
    app = RobotArmSimulator(root)
    root.mainloop()

if __name__ == "__main__":
    main()