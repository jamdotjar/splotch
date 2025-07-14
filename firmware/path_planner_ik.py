import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.widgets as widgets
import math
import numpy as np
import serial
import time
import sys
import glob
import signal
import atexit

# For copying to clipboard - try to import but provide fallback
try:
    import pyperclip
    HAS_PYPERCLIP = True
except ImportError:
    HAS_PYPERCLIP = False
    print("pyperclip not available, will print commands to console instead.")
    print("To install: pip install pyperclip")

# Track application state for clean exit
app_running = True

def signal_handler(sig, frame):
    """Handle termination signals gracefully"""
    global app_running
    print("\nReceived termination signal. Cleaning up...")
    app_running = False
    plt.close('all')
    
# Register signal handlers
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

class IKVisualizer:
    def __init__(self):
        # Same parameters as your Plotter class
        self.l1 = 110  # first link length
        self.l2 = 110  # second link length
        self.d = 25.5  # distance between arm origins
        
        # Safe zone parameters (configurable paper area)
        self.safe_zone_enabled = True
        self.safe_zone_width = 100  # mm
        self.safe_zone_height = 100  # mm
        self.safe_zone_x = -50+self.d/2  # mm (left edge position)
        self.safe_zone_y = -100  # mm (bottom edge position)
        
        # Path planning feature
        self.path_points = []  # [(x, y, pen_state, alpha, beta), ...]
        self.command_sequence = []  # List of formatted commands
        self.planning_mode = True  # We're always in planning mode now
        
        # Keep track of current position and pen state
        self.current_x = 0
        self.current_y = 0
        self.pen_down = False
        
        # Set up the plot with fixed size and axes limits
        self.fig = plt.figure(figsize=(12, 10))
        
        # Create a grid layout for our plot and controls
        self.gs = self.fig.add_gridspec(3, 1, height_ratios=[5, 1, 1])
        
        # Main plot area
        self.ax = self.fig.add_subplot(self.gs[0])
        self.ax.set_xlim(-150, 150)
        self.ax.set_ylim(-200, 50)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_title('SCARA Path Planner - Click to Add Points')
        
        # Controls area
        self.controls_ax = self.fig.add_subplot(self.gs[1])
        self.controls_ax.set_axis_off()
        
        # Command list area (for path planning)
        self.command_list_ax = self.fig.add_subplot(self.gs[2])
        self.command_list_ax.set_axis_off()
        
        # Arm origins
        self.left_origin = np.array([0, 0])
        self.right_origin = np.array([self.d, 0])
        
        # Add control buttons
        self.setup_buttons()
        
        # Initialize command list
        self.update_command_list()
        
    def setup_buttons(self):
        """Set up UI control buttons"""
        # Use the controls axis for button placement
        self.controls_ax.set_position([0.1, 0.05, 0.8, 0.1])  # [left, bottom, width, height]
        
        # Position buttons within the controls axis
        button_width = 0.12
        button_height = 0.6
        button_spacing = 0.02
        start_x = 0.02
        start_y = 0.2
        
        # Pen up button
        pen_up_pos = [start_x, start_y, button_width, button_height]
        self.btn_pen_up = widgets.Button(
            self.controls_ax.inset_axes(pen_up_pos),
            'Pen Up',
            color='lightgreen',
            hovercolor='lime'
        )
        self.btn_pen_up.on_clicked(self.on_pen_up)
        
        # Pen down button
        pen_down_pos = [start_x + (button_width + button_spacing), start_y, button_width, button_height]
        self.btn_pen_down = widgets.Button(
            self.controls_ax.inset_axes(pen_down_pos),
            'Pen Down',
            color='salmon',
            hovercolor='tomato'
        )
        self.btn_pen_down.on_clicked(self.on_pen_down)
        
        # Add dot button
        dot_pos = [start_x + 2 * (button_width + button_spacing), start_y, button_width, button_height]
        self.btn_dot = widgets.Button(
            self.controls_ax.inset_axes(dot_pos),
            'Add Dot',
            color='skyblue', 
            hovercolor='deepskyblue'
        )
        self.btn_dot.on_clicked(self.on_add_dot)
        
        # Toggle safe zone button
        toggle_pos = [start_x + 3 * (button_width + button_spacing), start_y, button_width, button_height]
        self.btn_toggle_safe = widgets.Button(
            self.controls_ax.inset_axes(toggle_pos),
            'Toggle Safe',
            color='lightgray',
            hovercolor='darkgray'
        )
        self.btn_toggle_safe.on_clicked(self.on_toggle_safe)
        
        # Clear path button
        clear_pos = [start_x + 4 * (button_width + button_spacing), start_y, button_width, button_height]
        self.btn_clear = widgets.Button(
            self.controls_ax.inset_axes(clear_pos),
            'Clear Path',
            color='pink',
            hovercolor='hotpink'
        )
        self.btn_clear.on_clicked(self.on_clear_path)
        
        # Copy commands button
        copy_pos = [start_x + 5 * (button_width + button_spacing), start_y, button_width, button_height]
        self.btn_copy = widgets.Button(
            self.controls_ax.inset_axes(copy_pos),
            'Copy Angles',
            color='lightyellow',
            hovercolor='yellow'
        )
        self.btn_copy.on_clicked(self.on_copy_commands)
        
    def on_pen_up(self, event):
        """Add pen up command to the sequence"""
        self.pen_down = False
        self.add_command("pen:up", None)
        self.update_status("Added: Pen UP command")
        self.draw_path()
    
    def on_pen_down(self, event):
        """Add pen down command to the sequence"""
        self.pen_down = True
        self.add_command("pen:down", None)
        self.update_status("Added: Pen DOWN command")
        self.draw_path()
    
    def on_add_dot(self, event):
        """Add a dot command (pen down then up at current location)"""
        # Add commands for the dot (down and up at the current point)
        if len(self.path_points) > 0:
            last_x, last_y, _, last_alpha, last_beta = self.path_points[-1]
            self.add_command("pen:down", None)
            self.add_command("pen:up", None)
            self.update_status(f"Added: Dot at ({last_x:.1f}, {last_y:.1f})")
            self.draw_path()
        else:
            self.update_status("Can't add dot: No position set")
    
    def on_toggle_safe(self, event):
        """Toggle the safe zone visibility"""
        self.toggle_safe_zone()
        self.draw_path()
        status = "visible" if self.safe_zone_enabled else "hidden"
        self.update_status(f"Safe zone: {status}")
    
    def on_clear_path(self, event):
        """Clear the current path"""
        self.path_points = []
        self.command_sequence = []
        self.update_command_list()
        self.update_status("Path cleared")
        self.draw_path()
    
    def on_copy_commands(self, event):
        """Copy all commands to clipboard"""
        if not self.command_sequence:
            self.update_status("No commands to copy")
            return
            
        command_text = ';'.join(self.command_sequence)
        
        if HAS_PYPERCLIP:
            pyperclip.copy(command_text)
            self.update_status("Commands copied to clipboard!")
        else:
            print("\n=== SERVO ANGLES COMMAND SEQUENCE ===")
            print(command_text)
            print("=== END COMMAND SEQUENCE ===\n")
            self.update_status("Commands printed to console (pyperclip not available)")
        
    def add_command(self, cmd_str, point=None):
        """Add a command to the sequence"""
        if cmd_str.startswith("xy:"):
            _, coords = cmd_str.split(":", 1)
            x, y = map(float, coords.split(","))
            
            # Calculate servo angles for this point
            alpha, beta = self.calcIK(x, y)
            if alpha is None or beta is None:
                self.update_status(f"Point ({x:.1f}, {y:.1f}) unreachable!")
                return
                
            # Format the servo angles command
            angle_cmd = f"{alpha:.1f},{beta:.1f}"
            
            # Store the command with actual servo angles
            self.command_sequence.append(angle_cmd)
            
            # Record the point with its pen state and angles
            if point is not None:  # User clicked
                self.path_points.append((x, y, self.pen_down, alpha, beta))
                self.current_x = x
                self.current_y = y
                
        elif cmd_str.startswith("pen:"):
            _, state = cmd_str.split(":", 1)
            
            # Store the pen command
            self.command_sequence.append(f"pen:{state}")
            
            # Update pen state for future points
            self.pen_down = (state == "down")
        
        else:
            self.update_status(f"Unknown command: {cmd_str}")
            return
            
        # Update the command list display
        self.update_command_list()
    
    def update_command_list(self):
        """Update the command list display"""
        self.command_list_ax.clear()
        
        if not self.command_sequence:
            self.command_list_ax.text(0.5, 0.5, "No commands yet. Click on plot to add points.",
                                     ha='center', va='center', fontsize=12)
        else:
            # Show the last several commands
            commands_to_show = self.command_sequence[-10:] if len(self.command_sequence) > 10 else self.command_sequence
            
            # Format the command text
            cmd_texts = []
            for i, cmd in enumerate(commands_to_show):
                if cmd.startswith("pen:"):
                    cmd_texts.append(f"{i+1}. PEN {cmd.split(':')[1].upper()}")
                else:
                    cmd_texts.append(f"{i+1}. MOVE TO ANGLES [{cmd}]")
            
            # Display as multi-line text
            cmd_str = "\n".join(cmd_texts)
            count_str = f"Commands: {len(self.command_sequence)} total"
            self.command_list_ax.text(0.02, 0.9, count_str, fontsize=12, va='top')
            self.command_list_ax.text(0.02, 0.8, cmd_str, fontsize=10, va='top', fontfamily='monospace')
            
            if len(self.command_sequence) > 10:
                self.command_list_ax.text(0.02, 0.1, f"...and {len(self.command_sequence) - 10} more", fontsize=10)
                
        self.command_list_ax.set_axis_off()
        self.fig.canvas.draw_idle()
    
    def update_status(self, message):
        """Update status text in the title"""
        self.ax.set_title(f'SCARA Path Planner - {message}')
        self.fig.canvas.draw_idle()

    def calcIK(self, x1, y1):
        """
        Calculate inverse kinematics for parallel dual arm SCARA system
        Both arms are connected at their endpoints (closed chain)
        """
        x = x1
        y = y1
        a1 = self.l1
        a2 = self.l2
        d = self.d

        try:
            c = math.sqrt((x**2) + (y**2))
            e = math.sqrt(((d-x)**2) + (y**2))
            if c > a1 + a2 or c < abs(a1 - a2):
                return None, None
            if e > a1 + a2 or e < abs(a1 - a2):
                return None, None
            cos_q2_left = (c**2 - a1**2 - a2**2) / (2 * a1 * a2)
            if abs(cos_q2_left) > 1:
                return None, None
            q2_left = math.acos(cos_q2_left)
            if x == 0:
                q1_left = math.pi/2 if y > 0 else -math.pi/2
            else:
                k1 = a1 + a2 * math.cos(q2_left)
                k2 = a2 * math.sin(q2_left)
                q1_left = math.atan2(y, x) - math.atan2(k2, k1)
            x_right = x - d
            cos_q2_right = (e**2 - a1**2 - a2**2) / (2 * a1 * a2)
            if abs(cos_q2_right) > 1:
                return None, None
            q2_right = -math.acos(cos_q2_right)
            if x_right == 0:
                q1_right = math.pi/2 if y > 0 else -math.pi/2
            else:
                k1_right = a1 + a2 * math.cos(q2_right)
                k2_right = a2 * math.sin(q2_right)
                q1_right = math.atan2(y, x_right) - math.atan2(k2_right, k1_right)
            alpha_deg = math.degrees(q1_left)
            beta_deg = math.degrees(q1_right)
            self.left_elbow_angle = math.degrees(q2_left)
            self.right_elbow_angle = math.degrees(q2_right)
            return alpha_deg, beta_deg
        except (ValueError, ZeroDivisionError) as e:
            return None, None
    
    def calcFK(self, t1, t2):
        """Calculate forward kinematics to verify IK"""
        try:
            t1_rad = math.radians(t1)
            t2_rad = math.radians(t2)
            
            l1 = self.l1
            l2 = self.l2
            dj = self.d
            
            a = l1 * math.cos(t1_rad)
            b = l1 * math.sin(t1_rad)
            c = dj + (l1 * math.cos(t2_rad))
            d = l1 * math.sin(t2_rad)
            
            k1 = c - a
            k2 = d - b
            k3 = c + a
            k4 = d + b
            
            j1 = c**2 + d**2
            j2 = a**2 + b**2
            
            db = math.sqrt(k1**2 + k2**2)
            
            if db > 2 * l2:
                return None, None
            
            phi = math.acos(db / (2 * l2))
            phi1 = math.acos((db**2 + l1**2 - j1) / (2 * db * l1))
            
            s = l1**2 - 2 * l1 * l2 * math.cos(phi + phi1) + ((j1 + j2) / 2)
            
            denominator = 2 * (k1 * k4 - k2 * k3)
            if abs(denominator) < 1e-10:
                return None, None
                
            y = (2 * k1 * s - (k3 * (j1 - j2))) / denominator
            x = (j1 - j2 - 2 * k2 * y) / (2 * k1) if abs(k1) > 1e-10 else 0
            
            return x, y
            
        except (ValueError, ZeroDivisionError):
            return None, None
    
    def draw_arm_config(self, target_x, target_y, point_only=False):
        """Draw the arm configuration for a specific point"""
        # Calculate IK
        alpha, beta = self.calcIK(target_x, target_y)
        
        if alpha is None or beta is None:
            self.update_status(f"Target ({target_x:.1f}, {target_y:.1f}) unreachable!")
            return False
        
        # Convert to radians for drawing
        alpha_rad = math.radians(alpha)
        beta_rad = math.radians(beta)

        # Calculate elbow positions
        left_elbow = self.left_origin + self.l1 * np.array([math.cos(alpha_rad), math.sin(alpha_rad)])
        right_elbow = self.right_origin + self.l1 * np.array([math.cos(beta_rad), math.sin(beta_rad)])
        
        # Draw the arms and end effector
        if not point_only:
            # Draw first links (shoulder to elbow)
            self.ax.plot([self.left_origin[0], left_elbow[0]], [self.left_origin[1], left_elbow[1]], 
                        'b-', linewidth=2, alpha=0.5)
            self.ax.plot([self.right_origin[0], right_elbow[0]], [self.right_origin[1], right_elbow[1]], 
                        'r-', linewidth=2, alpha=0.5)

            # Draw second links (elbow to common end effector)
            self.ax.plot([left_elbow[0], target_x], [left_elbow[1], target_y], 
                        'b-', linewidth=1.5, alpha=0.5)
            self.ax.plot([right_elbow[0], target_x], [right_elbow[1], target_y], 
                        'r-', linewidth=1.5, alpha=0.5)

            # Draw joints
            self.ax.plot(left_elbow[0], left_elbow[1], 'bo', markersize=4, alpha=0.5)
            self.ax.plot(right_elbow[0], right_elbow[1], 'ro', markersize=4, alpha=0.5)
        
        return True
    
    def draw_path(self):
        """Draw the current path with all points and connections"""
        # Clear the plot
        self.ax.clear()
        self.ax.set_xlim(-150, 150)
        self.ax.set_ylim(-200, 50)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_title('SCARA Path Planner - Click to Add Points')
        
        # Draw the safe zone (paper area) if enabled
        if self.safe_zone_enabled:
            safe_rect = patches.Rectangle(
                (self.safe_zone_x, self.safe_zone_y), 
                self.safe_zone_width, -self.safe_zone_height,  # Negative height because y is inverted
                linewidth=2, edgecolor='green', facecolor='lightgreen', alpha=0.2,
                label='Safe Zone (Paper)'
            )
            self.ax.add_patch(safe_rect)
            
            # Add text showing the safe zone dimensions
            self.ax.text(
                self.safe_zone_x + self.safe_zone_width/2, 
                self.safe_zone_y - self.safe_zone_height/2,
                f"{self.safe_zone_width}x{self.safe_zone_height}mm",
                ha='center', va='center', color='green', fontsize=10
            )
        
        # Draw arm origins
        self.ax.plot(self.left_origin[0], self.left_origin[1], 'ko', markersize=6, label='Left Origin')
        self.ax.plot(self.right_origin[0], self.right_origin[1], 'ko', markersize=6, label='Right Origin')
        
        # Draw the connecting line between origins to show the base
        self.ax.plot([self.left_origin[0], self.right_origin[0]], 
                    [self.left_origin[1], self.right_origin[1]], 
                    'k-', linewidth=2, alpha=0.5)
        
        # Draw workspace (intersection of both arm workspaces)
        left_workspace = plt.Circle(self.left_origin, self.l1 + self.l2, 
                                  fill=False, color='blue', alpha=0.2, linestyle='--')
        right_workspace = plt.Circle(self.right_origin, self.l1 + self.l2, 
                                   fill=False, color='red', alpha=0.2, linestyle='--')
        self.ax.add_patch(left_workspace)
        self.ax.add_patch(right_workspace)
        
        # Draw the path points and connecting lines
        if len(self.path_points) > 0:
            # Draw the path lines based on pen state
            for i in range(1, len(self.path_points)):
                prev_x, prev_y, prev_pen, _, _ = self.path_points[i-1]
                curr_x, curr_y, curr_pen, _, _ = self.path_points[i]
                
                # Line style depends on pen state at previous point
                if prev_pen:  # Pen was down for this segment
                    self.ax.plot([prev_x, curr_x], [prev_y, curr_y], 'g-', linewidth=2)
                else:  # Pen was up for this segment
                    self.ax.plot([prev_x, curr_x], [prev_y, curr_y], 'g--', linewidth=1, alpha=0.6)
            
            # Draw all path points with different markers based on pen state
            for i, (x, y, pen_state, _, _) in enumerate(self.path_points):
                # Point marker style based on pen state
                marker = 'D' if pen_state else 'o'  # Diamond for pen down, circle for pen up
                color = 'darkgreen' if pen_state else 'green'
                # Larger marker for first and last points
                size = 10 if (i == 0 or i == len(self.path_points)-1) else 6
                self.ax.plot(x, y, marker=marker, color=color, markersize=size)
                
                # Add small index number next to each point
                self.ax.annotate(str(i+1), (x, y), 
                                xytext=(3, 3), 
                                textcoords='offset points', 
                                fontsize=8)
            
            # Show the current arm configuration for the last point
            last_x, last_y, _, _, _ = self.path_points[-1]
            self.draw_arm_config(last_x, last_y)
            
            # Highlight the last point as the current position
            self.ax.plot(last_x, last_y, 'go', markersize=12, alpha=0.7)
            
            # Show the coordinates and angles of the last point
            last_x, last_y, _, last_alpha, last_beta = self.path_points[-1]
            self.ax.annotate(f"({last_x:.1f}, {last_y:.1f})\n[{last_alpha:.1f}°, {last_beta:.1f}°]",
                            (last_x, last_y),
                            xytext=(10, 10),
                            textcoords='offset points',
                            fontsize=9,
                            bbox=dict(boxstyle='round,pad=0.5', fc='white', alpha=0.7))
        
        # Add a legend
        self.ax.legend(loc='upper left')
        
        # Refresh the plot
        self.fig.canvas.draw_idle()
    
    def is_point_in_safe_zone(self, x, y):
        """Check if a point is within the safe zone"""
        return (self.safe_zone_x <= x <= self.safe_zone_x + self.safe_zone_width and
                self.safe_zone_y - self.safe_zone_height <= y <= self.safe_zone_y)
                
    def toggle_safe_zone(self):
        """Toggle the visibility of the safe zone"""
        self.safe_zone_enabled = not self.safe_zone_enabled
    
    def interactive_planner(self):
        """Run the interactive path planner"""
        def on_click(event):
            if event.inaxes != self.ax:
                return
                
            # Get the clicked coordinates
            x = event.xdata
            y = event.ydata
            
            # Check if point is within workspace bounds
            alpha, beta = self.calcIK(x, y)
            if alpha is None or beta is None:
                self.update_status(f"Point ({x:.1f}, {y:.1f}) unreachable!")
                return
                
            # Check if point is in safe zone when enabled
            if self.safe_zone_enabled and not self.is_point_in_safe_zone(x, y):
                self.update_status(f"Warning: Point ({x:.1f}, {y:.1f}) is outside safe zone!")
            
            # Add the command
            cmd_str = f"xy:{x:.1f},{y:.1f}"
            self.add_command(cmd_str, (x, y, self.pen_down, alpha, beta))
            
            # Update the display
            self.update_status(f"Added: [{alpha:.1f}, {beta:.1f}] for pos ({x:.1f}, {y:.1f})")
            self.draw_path()
        
        # Connect click handler
        self.fig.canvas.mpl_connect('button_press_event', on_click)
        
        # Draw initial state
        self.draw_path()
        
        print("Interactive Path Planner for SCARA Plotter")
        print("-----------------------------------------")
        print("Click on the plot to add points to your path")
        print("Use the buttons to control pen state:")
        print("  - Pen Up: Move without drawing")
        print("  - Pen Down: Move while drawing")
        print("  - Add Dot: Add a dot at the current position (pen down then up)")
        print("  - Toggle Safe: Show/hide the safe drawing area")
        print("  - Clear Path: Remove all commands")
        print("  - Copy Angles: Copy the servo angles command sequence to clipboard")
        print("\nOutput will be in the format of servo angles (left_angle,right_angle)")
        
        plt.tight_layout()
        plt.subplots_adjust(hspace=0.3)
        
        # Show the plot and handle cleanup
        try:
            plt.show(block=True)
        except Exception as e:
            print(f"Exception in matplotlib event loop: {e}")
            import traceback
            traceback.print_exc()

if __name__ == "__main__":
    # Register cleanup function
    def cleanup():
        print("Program exiting, cleaning up resources...")
    
    atexit.register(cleanup)
    
    planner = None
    try:
        print("Starting Path Planner...")
        planner = IKVisualizer()
        planner.interactive_planner()
    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
    except Exception as e:
        import traceback
        print(f"Unexpected error: {e}")
        traceback.print_exc()
    finally:
        # Close all matplotlib figures
        plt.close('all')
        print("Path planning terminated.")
