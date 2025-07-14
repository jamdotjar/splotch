import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.widgets as widgets
import math
import numpy as np
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

class IKPathPlanner:
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
        
        # Keep track of current position and pen state
        self.current_x = 0
        self.current_y = 0
        self.pen_down = False
        
        # Command sequence
        self.commands = []
        self.path_points = []  # [(x, y, pen_state, alpha, beta), ...]
        self.current_command_id = 0
        
        # Set up the plot with fixed size and axes limits
        self.fig = plt.figure(figsize=(12, 10))
        
        # Create a grid layout for our plot and controls
        self.gs = self.fig.add_gridspec(3, 2, height_ratios=[4, 1, 1], width_ratios=[4, 1])
        
        # Main plot area
        self.ax = self.fig.add_subplot(self.gs[0, :])
        self.ax.set_xlim(-150, 150)
        self.ax.set_ylim(-200, 50)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_title('SCARA Plotter Path Planner with IK')
        
        # Controls area
        self.controls_ax = self.fig.add_subplot(self.gs[1, 0])
        self.controls_ax.set_axis_off()
        
        # Status area
        self.status_ax = self.fig.add_subplot(self.gs[1, 1])
        self.status_ax.set_axis_off()
        
        # Commands list area
        self.command_list_ax = self.fig.add_subplot(self.gs[2, :])
        self.command_list_ax.set_axis_off()
        
        # Arm origins
        self.left_origin = np.array([0, 0])
        self.right_origin = np.array([self.d, 0])
        
        # Add control buttons
        self.setup_buttons()
        
        # Display initial command list
        self.update_command_list()
    
    def setup_buttons(self):
        """Set up UI control buttons"""
        # Use the controls axis for button placement
        self.controls_ax.set_position([0.1, 0.05, 0.8, 0.1])  # [left, bottom, width, height]
        
        # Position buttons within the controls axis
        button_width = 0.15
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
            'Toggle Safe Zone',
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
            'Copy Commands',
            color='lightyellow',
            hovercolor='yellow'
        )
        self.btn_copy.on_clicked(self.on_copy_commands)
    
    def on_pen_up(self, event):
        """Add pen up command to the queue"""
        self.pen_down = False
        self.add_command("pen:up", None)
        self.update_status("Added: Pen UP command")
        self.draw_path()
    
    def on_pen_down(self, event):
        """Add pen down command to the queue"""
        self.pen_down = True
        self.add_command("pen:down", None)
        self.update_status("Added: Pen DOWN command")
        self.draw_path()
    
    def on_add_dot(self, event):
        """Add a dot command (pen down then up at current location)"""
        # Add commands for the dot (down and up at the current point)
        if len(self.path_points) > 0:
            last_x, last_y, _, last_alpha, last_beta = self.path_points[-1]
            self.add_command("pen:dot", None)
            self.update_status(f"Added: Dot at ({last_x:.1f}, {last_y:.1f}) -> angles [{last_beta:.1f}°, {last_alpha:.1f}°]")
            self.draw_path()
        else:
            self.update_status("Can't add dot: No position set")
    
    def on_toggle_safe(self, event):
        """Toggle the safe zone visibility"""
        self.safe_zone_enabled = not self.safe_zone_enabled
        status = "visible" if self.safe_zone_enabled else "hidden"
        self.update_status(f"Safe zone: {status}")
        self.draw_path()
    
    def on_clear_path(self, event):
        """Clear the current path"""
        self.commands = []
        self.path_points = []
        self.current_command_id = 0
        self.update_command_list()
        self.update_status("Path cleared")
        self.draw_path()
    
    def on_copy_commands(self, event):
        """Copy all commands to clipboard"""
        if not self.commands:
            self.update_status("No commands to copy")
            return
            
        command_text = self.format_commands()
        
        if HAS_PYPERCLIP:
            pyperclip.copy(command_text)
            self.update_status("Commands copied to clipboard!")
        else:
            print("\n=== COMMANDS TO COPY ===")
            print(command_text)
            print("=== END COMMANDS ===\n")
            self.update_status("Commands printed to console (pyperclip not available)")
    
    def format_commands(self):
        """Format the commands for output"""
        lines = []
        for cmd in self.commands:
            if cmd['type'] == 'move':
                # Format as servo angles for the plotter
                lines.append(f"{cmd['beta']:.1f},{cmd['alpha']:.1f}")
            elif cmd['type'] == 'pen':
                lines.append(f"pen:{cmd['state']}")
                
        return ';'.join(lines)
    
    def add_command(self, cmd_str, point=None):
        """Add a command to the sequence"""
        if cmd_str.startswith("xy:"):
            _, coords = cmd_str.split(":", 1)
            x, y = map(float, coords.split(","))
            
            # Calculate IK for the point
            alpha, beta = self.calcIK(x, y)
            if alpha is None or beta is None:
                self.update_status(f"Cannot add point: ({x:.1f}, {y:.1f}) - unreachable")
                return
            # this fixes the weird IK stuff that made angles negative, it's easier to invert than fix the IK itself
            alpha = 180 - -alpha
            beta = 180 - -beta
                
            cmd = {
                'id': self.current_command_id,
                'type': 'move',
                'x': x,
                'y': y,
                'alpha': alpha,
                'beta': beta
            }
            self.current_x = x
            self.current_y = y
            
            # If point is provided, it means user clicked directly
            if point is not None:
                self.path_points.append((x, y, self.pen_down, alpha, beta))
        
        elif cmd_str.startswith("pen:"):
            _, state = cmd_str.split(":", 1)
            cmd = {
                'id': self.current_command_id,
                'type': 'pen',
                'state': state
            }
            
            # Update the pen state for future path points
            self.pen_down = (state == "down")
        else:
            self.update_status(f"Unknown command: {cmd_str}")
            return
            
        self.commands.append(cmd)
        self.current_command_id += 1
        self.update_command_list()
    
    def update_status(self, message):
        """Update status display"""
        self.status_ax.clear()
        self.status_ax.text(0.5, 0.5, message, 
                           ha='center', va='center', fontsize=12)
        self.status_ax.set_axis_off()
        self.fig.canvas.draw_idle()
    
    def update_command_list(self):
        """Update the command list display"""
        self.command_list_ax.clear()
        
        if not self.commands:
            self.command_list_ax.text(0.5, 0.5, "No commands yet. Click on the plot to add points.",
                                     ha='center', va='center', fontsize=12)
        else:
            # Show the last several commands
            commands_to_show = self.commands[-10:] if len(self.commands) > 10 else self.commands
            
            # Format the command text
            cmd_texts = []
            for i, cmd in enumerate(commands_to_show):
                if cmd['type'] == 'move':
                    cmd_texts.append(f"{i+1}. MOVE to ({cmd['x']:.1f}, {cmd['y']:.1f}) → [{cmd['beta']:.1f}°, {cmd['alpha']:.1f}°]")
                elif cmd['type'] == 'pen':
                    cmd_texts.append(f"{i+1}. PEN {cmd['state'].upper()}")
            
            # Display as multi-line text
            cmd_str = "\n".join(cmd_texts)
            count_str = f"Commands: {len(self.commands)} total"
            self.command_list_ax.text(0.02, 0.9, count_str, fontsize=12, va='top')
            self.command_list_ax.text(0.02, 0.8, cmd_str, fontsize=10, va='top', fontfamily='monospace')
            
            if len(self.commands) > 10:
                self.command_list_ax.text(0.02, 0.1, f"...and {len(self.commands) - 10} more", fontsize=10)
                
        self.command_list_ax.set_axis_off()
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
    
    def is_point_in_safe_zone(self, x, y):
        """Check if a point is within the safe zone"""
        return (self.safe_zone_x <= x <= self.safe_zone_x + self.safe_zone_width and
                self.safe_zone_y - self.safe_zone_height <= y <= self.safe_zone_y)
    
    def draw_path(self):
        """Draw the current path with all points and connecting lines"""
        # Clear the plot
        self.ax.clear()
        self.ax.set_xlim(-150, 150)
        self.ax.set_ylim(-250, 50)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_title('SCARA Plotter Path Planner with IK')
        
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
        self.ax.plot(self.left_origin[0], self.left_origin[1], 'ko', markersize=10, label='Left Origin')
        self.ax.plot(self.right_origin[0], self.right_origin[1], 'ko', markersize=10, label='Right Origin')
        
        # Draw the connecting line between origins to show the base
        self.ax.plot([self.left_origin[0], self.right_origin[0]], 
                    [self.left_origin[1], self.right_origin[1]], 
                    'k-', linewidth=2, alpha=0.5, label='Base')
        
        # Draw workspace (intersection of both arm workspaces)
        left_workspace = plt.Circle(self.left_origin, self.l1 + self.l2, 
                                  fill=False, color='blue', alpha=0.2, linestyle='--')
        right_workspace = plt.Circle(self.right_origin, self.l1 + self.l2, 
                                   fill=False, color='red', alpha=0.2, linestyle='--')
        self.ax.add_patch(left_workspace)
        self.ax.add_patch(right_workspace)
        
        # Draw the path points and lines connecting them
        if len(self.path_points) > 0:
            # Separate points by pen state to draw segments
            pen_up_points = []
            pen_down_points = []
            
            for i, (x, y, pen_state, alpha, beta) in enumerate(self.path_points):
                if i > 0:
                    prev_x, prev_y, prev_pen, _, _ = self.path_points[i-1]
                    
                    # Draw line segment based on current pen state
                    if pen_state:  # Pen is down for this segment
                        self.ax.plot([prev_x, x], [prev_y, y], 'b-', linewidth=2)
                    else:  # Pen is up for this segment
                        self.ax.plot([prev_x, x], [prev_y, y], 'b--', linewidth=1, alpha=0.5)
                
                # For the current point, we want to draw the arm configuration
                if i == len(self.path_points) - 1:
                    self.draw_arm_config(x, y, alpha, beta, clear_plot=False)
                
                # Collect points for later plotting
                if pen_state:
                    pen_down_points.append((x, y))
                else:
                    pen_up_points.append((x, y))
            
            # Draw points with different markers based on pen state
            if pen_up_points:
                x_up, y_up = zip(*pen_up_points)
                self.ax.plot(x_up, y_up, 'bo', markersize=6, alpha=0.7, label='Pen Up')
                
            if pen_down_points:
                x_down, y_down = zip(*pen_down_points)
                self.ax.plot(x_down, y_down, 'ro', markersize=6, label='Pen Down')
            
            # Highlight the last point as the current position
            last_x, last_y, last_pen, last_alpha, last_beta = self.path_points[-1]
            self.ax.plot(last_x, last_y, 'go', markersize=10, label='Current Position')
            
            # Show the coordinates and angles of the last point
            self.ax.annotate(f"({last_x:.1f}, {last_y:.1f})\n[{last_beta:.1f}°, {last_alpha:.1f}°]",
                            (last_x, last_y),
                            xytext=(10, 10),
                            textcoords='offset points',
                            fontsize=9)
        
        # Add a legend
        self.ax.legend(loc='upper left')
        
        # Refresh the plot
        self.fig.canvas.draw_idle()
    
    def draw_arm_config(self, x, y, alpha, beta, clear_plot=False):
        """Draw the arm configuration for a given point"""
        # Convert to radians for drawing
        alpha_rad = math.radians(-180 +alpha)
        
        beta_rad = math.radians(-180  +beta)

        # Calculate elbow positions
        left_elbow = self.left_origin + self.l1 * np.array([math.cos(alpha_rad), math.sin(alpha_rad)])
        right_elbow = self.right_origin + self.l1 * np.array([math.cos(beta_rad), math.sin(beta_rad)])
        
        # Draw first links (shoulder to elbow)
        self.ax.plot([self.left_origin[0], left_elbow[0]], [self.left_origin[1], left_elbow[1]], 
                    'b-', linewidth=4, label='Left Arm L1')
        self.ax.plot([self.right_origin[0], right_elbow[0]], [self.right_origin[1], right_elbow[1]], 
                    'r-', linewidth=4, label='Right Arm L1')

        # Draw second links (elbow to common end effector)
        self.ax.plot([left_elbow[0], x], [left_elbow[1], y], 
                    'b-', linewidth=3, alpha=0.8, label='Left Arm L2')
        self.ax.plot([right_elbow[0], x], [right_elbow[1], y], 
                    'r-', linewidth=3, alpha=0.8, label='Right Arm L2')

        # Draw joints
        self.ax.plot(left_elbow[0], left_elbow[1], 'bo', markersize=8, label='Left Elbow')
        self.ax.plot(right_elbow[0], right_elbow[1], 'ro', markersize=8, label='Right Elbow')
        
    def on_click(self, event):
        """Handle mouse click to add points to the path"""
        if event.inaxes != self.ax:
            return
            
        x = event.xdata
        y = event.ydata
        
        # Check if point is within workspace bounds
        alpha, beta = self.calcIK(x, y)
        if alpha is None or beta is None:
            self.update_status(f"Point ({x:.1f}, {y:.1f}) is unreachable!")
            return
            
        # Check if point is in safe zone when enabled
        if self.safe_zone_enabled and not self.is_point_in_safe_zone(x, y):
            self.update_status(f"Warning: Point ({x:.1f}, {y:.1f}) is outside safe zone!")
        
        # Add the command
        cmd_str = f"xy:{x:.1f},{y:.1f}"
        self.add_command(cmd_str, (x, y, self.pen_down, alpha, beta))
        
        # Update the display
        self.update_status(f"Added: ({x:.1f}, {y:.1f}) → [{beta:.1f}°, {alpha:.1f}°], Pen {'DOWN' if self.pen_down else 'UP'}")
        self.draw_path()
    
    def start_planning(self):
        """Start the interactive path planning session"""
        # Connect click handler
        self.cid = self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        
        # Draw initial state
        self.draw_path()
        
        print("Interactive IK Path Planner")
        print("-------------------------")
        print("Click on the plot to add points to your path")
        print("Use the buttons to control pen state:")
        print("  - Pen Up: Move without drawing")
        print("  - Pen Down: Move while drawing")
        print("  - Add Dot: Add a dot at the current position (pen down then up)")
        print("  - Toggle Safe Zone: Show/hide the safe drawing area")
        print("  - Clear Path: Remove all commands")
        print("  - Copy Commands: Copy the servo angle commands to clipboard")
        print("\nOutputs servo angle commands in format: alpha,beta")
        print("For example: 120.5,80.2")
        print("Pen commands: pen:up, pen:down")
        
        plt.tight_layout()
        plt.subplots_adjust(bottom=0.05, top=0.95, hspace=0.3)
        
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
        print("Starting IK Path Planner...")
        planner = IKPathPlanner()
        planner.start_planning()
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
