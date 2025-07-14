import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, TextBox
import math
import time
import sys
import os
from serial_comm import SerialComm
import matplotlib.patches as patches
from matplotlib import widgets


class IKVisualizer:
    def __init__(self):
        # Same parameters as your Plotter class
        self.l1 = 110  # first link length
        self.l2 = 110  # second link length
        self.d = 25.5  # distance between arm origins
        
        # Set up the plot
        self.fig, self.ax = plt.subplots(1, 1, figsize=(10, 8))
        self.ax.set_xlim(-150, 150)
        self.ax.set_ylim(-50, 200)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_title('SCARA Dual Arm IK Visualization')
        
        # Arm origins (ORIGINAL)
        self.left_origin = np.array([0, 0])
        self.right_origin = np.array([self.d, 0])
        
        # Initialize serial communication (None initially)
        self.serial_comm = None
        self.connected = False
        
        # Current target position and angles
        self.current_target = (0, 0)
        self.current_angles = (90, 90)
        
    def connect_serial(self, port, baudrate=115200):
        """Establish serial connection to the Arduino"""
        try:
            self.serial_comm = SerialComm(port, baudrate)
            self.connected = True
            print(f"Connected to {port} at {baudrate} baud.")
        except Exception as e:
            print(f"Error connecting to serial port: {e}")
            self.connected = False
    
    def close_serial(self):
        """Close the serial connection"""
        if self.serial_comm is not None:
            self.serial_comm.close()
            self.connected = False
            print("Serial connection closed.")
    
    def send_command(self, command):
        """Send a command to the Arduino over serial"""
        if self.connected and self.serial_comm is not None:
            self.serial_comm.send(command)
        else:
            print("Not connected to any serial port.")
    
    def connect_to_mcu(self, port='/dev/tty.usbmodem1101'):
        """Connect to the microcontroller"""
        try:
            self.serial_comm = SerialComm(port=port)
            self.connected = self.serial_comm.connected
            if self.connected:
                print(f"Connected to microcontroller on {port}")
                return True
            else:
                print("Failed to connect to microcontroller")
                return False
        except Exception as e:
            print(f"Error connecting to microcontroller: {e}")
            self.connected = False
            return False
    
    def disconnect_from_mcu(self):
        """Disconnect from the microcontroller"""
        if self.serial_comm is not None:
            self.serial_comm.close()
            self.connected = False
            self.serial_comm = None
            print("Disconnected from microcontroller")
    
    def send_angles_to_mcu(self, alpha, beta):
        """Send calculated angles to the microcontroller"""
        if not self.connected or self.serial_comm is None:
            print("Not connected to microcontroller")
            return False
        
        print(f"Sending angles to MCU: α={alpha:.2f}°, β={beta:.2f}°")
        return self.serial_comm.send_servo_angles(alpha, beta)
    
    def pen_up(self):
        """Send pen up command to microcontroller"""
        if not self.connected or self.serial_comm is None:
            print("Not connected to microcontroller")
            return False
        
        return self.serial_comm.pen_up()
    
    def pen_down(self):
        """Send pen down command to microcontroller"""
        if not self.connected or self.serial_comm is None:
            print("Not connected to microcontroller")
            return False
        
        return self.serial_comm.pen_down()
    
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

        # Servo angle limits removed

        try:
            c = math.sqrt((x**2) + (y**2))
            e = math.sqrt(((d-x)**2) + (y**2))
            if c > a1 + a2 or c < abs(a1 - a2):
                print(f"Position ({x}, {y}) is out of reach for left arm")
                return None, None
            if e > a1 + a2 or e < abs(a1 - a2):
                print(f"Position ({x}, {y}) is out of reach for right arm")
                return None, None
            cos_q2_left = (c**2 - a1**2 - a2**2) / (2 * a1 * a2)
            if abs(cos_q2_left) > 1:
                print(f"Left arm: Invalid cosine value {cos_q2_left}")
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
                print(f"Right arm: Invalid cosine value {cos_q2_right}")
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
            # Servo limits removed
            # Arms crossing check removed
            return alpha_deg, beta_deg
        except (ValueError, ZeroDivisionError) as e:
            print(f"Error calculating IK for ({x}, {y}): {e}")
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
            
        except (ValueError, ZeroDivisionError) as e:
            print(f"Error in FK calculation: {e}")
            return None, None
    
    def draw_arm_config(self, target_x, target_y, clear_plot=True):
        # --- Display angle visualization (relative to the back as front) ---
        # Mirror the target if y > 0 (behind), for display only
        display_y = -target_y if target_y > 0 else target_y
        display_alpha, display_beta = self.calcIK(target_x, display_y)
        if display_alpha is not None and display_beta is not None:
            display_alpha_rad = math.radians(display_alpha)
            display_beta_rad = math.radians(display_beta)
            # Draw dashed lines for display angles
            left_disp_line = self.left_origin + self.l1 * 0.5 * np.array([math.cos(display_alpha_rad), math.sin(display_alpha_rad)])
            right_disp_line = self.right_origin + self.l1 * 0.5 * np.array([math.cos(display_beta_rad), math.sin(display_beta_rad)])
            self.ax.plot([self.left_origin[0], left_disp_line[0]], [self.left_origin[1], left_disp_line[1]], 'b--', linewidth=1.5, alpha=0.5)
            self.ax.plot([self.right_origin[0], right_disp_line[0]], [self.right_origin[1], right_disp_line[1]], 'r--', linewidth=1.5, alpha=0.5)
            # Draw display angle text
            self.ax.text(self.left_origin[0]-10, self.left_origin[1]-15, f"Display: {display_alpha:.1f}°", color='blue', fontsize=9)
            self.ax.text(self.right_origin[0]+10, self.right_origin[1]-15, f"Display: {display_beta:.1f}°", color='red', fontsize=9)
            # Draw arc to indicate shoulder angle (mirrored/display angle)
            arc_radius = 22
            arc_width = 3
            arc_theta1_left = 270  # Start at downward (negative y)
            arc_theta2_left = 270 + display_alpha  # End at display angle
            arc_theta1_right = 270
            arc_theta2_right = 270 + display_beta
            left_arc = patches.Arc(self.left_origin, arc_radius, arc_radius, angle=0,
                                  theta1=arc_theta1_left, theta2=arc_theta2_left,
                                  color='blue', linewidth=arc_width)
            right_arc = patches.Arc(self.right_origin, arc_radius, arc_radius, angle=0,
                                   theta1=arc_theta1_right, theta2=arc_theta2_right,
                                   color='red', linewidth=arc_width)
            self.ax.add_patch(left_arc)
            self.ax.add_patch(right_arc)
        """Draw the current arm configuration with proper parallel mechanism display"""
        if clear_plot:
            self.ax.clear()
            self.ax.set_xlim(-150, 150)
            self.ax.set_ylim(-200, 50)
            self.ax.set_aspect('equal')
            self.ax.grid(True, alpha=0.3)
            self.ax.set_title('SCARA Parallel Dual Arm IK Visualization')
        
        # Calculate IK
        alpha, beta = self.calcIK(target_x, target_y)
        
        if alpha is None or beta is None:
            self.ax.text(0, 150, f"Target ({target_x}, {target_y}) unreachable or invalid!", 
                        fontsize=12, color='red', ha='center')
            self.ax.text(0, 130, "Reason: Out of reach, arms cross, or servo limits.", fontsize=10, color='red', ha='center')
            return
        
        # Store current angles
        self.current_angles = (alpha, beta)
        self.current_target = (target_x, target_y)
        
        # Convert to radians for drawing
        alpha_rad = math.radians(alpha)
        beta_rad = math.radians(beta)

        # Use original elbow calculations (no swap)
        left_elbow = self.left_origin + self.l1 * np.array([math.cos(alpha_rad), math.sin(alpha_rad)])
        right_elbow = self.right_origin + self.l1 * np.array([math.cos(beta_rad), math.sin(beta_rad)])
        
        # Second links connect elbows to the common end effector (target)
        # Both should reach the same target point
        
        # Draw arm origins
        self.ax.plot(self.left_origin[0], self.left_origin[1], 'ko', markersize=10, label='Left Origin')
        self.ax.plot(self.right_origin[0], self.right_origin[1], 'ko', markersize=10, label='Right Origin')
        
        # Draw first links (shoulder to elbow)
        self.ax.plot([self.left_origin[0], left_elbow[0]], [self.left_origin[1], left_elbow[1]], 
                    'b-', linewidth=4, label='Left Arm L1')
        self.ax.plot([self.right_origin[0], right_elbow[0]], [self.right_origin[1], right_elbow[1]], 
                    'r-', linewidth=4, label='Right Arm L1')

        # Draw second links (elbow to common end effector)
        self.ax.plot([left_elbow[0], target_x], [left_elbow[1], target_y], 
                    'b-', linewidth=3, alpha=0.8, label='Left Arm L2')
        self.ax.plot([right_elbow[0], target_x], [right_elbow[1], target_y], 
                    'r-', linewidth=3, alpha=0.8, label='Right Arm L2')

        # Draw joints
        self.ax.plot(left_elbow[0], left_elbow[1], 'bo', markersize=8, label='Left Elbow')
        self.ax.plot(right_elbow[0], right_elbow[1], 'ro', markersize=8, label='Right Elbow')
        
        # Draw the common end effector (where both arms connect)
        self.ax.plot(target_x, target_y, 'go', markersize=12, label='End Effector')
        
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
        
        # Verify that both second links have correct length
        left_l2_length = math.sqrt((target_x - left_elbow[0])**2 + (target_y - left_elbow[1])**2)
        right_l2_length = math.sqrt((target_x - right_elbow[0])**2 + (target_y - right_elbow[1])**2)
        
    
    
        # Show link lengths and verification
        self.ax.text(-140, 140, f'L1 = {self.l1}mm', fontsize=10)
        self.ax.text(-140, 130, f'L2 = {self.l2}mm', fontsize=10)
        self.ax.text(-140, 120, f'Arm separation = {self.d}mm', fontsize=10)
        # Show actual shoulder angles (for debug)
        display_y = -target_y if target_y > 0 else target_y
        display_alpha, display_beta = self.calcIK(target_x, display_y)
        if display_alpha is not None and display_beta is not None:
            self.ax.text(-140, 110, f' angle left: {-display_alpha:.2f}°', fontsize=10, color='blue')
            self.ax.text(-140, 105, f' angle right: {-display_beta:.2f}°', fontsize=10, color='red')
        self.ax.text(-140, 100, f'Left L2 actual: {left_l2_length:.1f}mm', fontsize=10, color='blue')
        self.ax.text(-140, 90, f'Right L2 actual: {right_l2_length:.1f}mm', fontsize=10, color='red')
        l2_error_left = abs(left_l2_length - self.l2)
        l2_error_right = abs(right_l2_length - self.l2)
        self.ax.text(-140, 80, f'L2 error left: {l2_error_left:.2f}mm', fontsize=10, color='blue')
        self.ax.text(-140, 70, f'L2 error right: {l2_error_right:.2f}mm', fontsize=10, color='red')
        
        # Show if the configuration is valid (both L2 links have correct length)
        if l2_error_left < 0.1 and l2_error_right < 0.1:
            self.ax.text(-140, 50, 'Configuration: VALID', fontsize=10, color='green')
        else:
            self.ax.text(-140, 50, 'Configuration: INVALID', fontsize=10, color='red')
        
        self.ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        plt.tight_layout()
        plt.draw()
    
    def interactive_test(self):
        """Interactive test - click to set target positions"""
        def on_click(event):
            if event.inaxes != self.ax:
                return
            target_x = event.xdata
            target_y = event.ydata
            self.current_target = (target_x, target_y)
            
            # Calculate angles for this position
            alpha, beta = self.calcIK(target_x, target_y)
            if alpha is not None and beta is not None:
                self.current_angles = (alpha, beta)
            
            self.draw_arm_config(target_x, target_y)
            plt.draw()
        
        # Add button for connecting to MCU
        connect_ax = plt.axes([0.8, 0.9, 0.15, 0.05])
        connect_button = widgets.Button(connect_ax, 'Connect to MCU')
        
        def on_connect(event):
            if not self.connected:
                if self.connect_to_mcu():
                    connect_button.label.set_text('Disconnect')
            else:
                self.disconnect_from_mcu()
                connect_button.label.set_text('Connect to MCU')
            plt.draw()
        
        connect_button.on_clicked(on_connect)
        
        # Add button for sending angles
        send_ax = plt.axes([0.8, 0.8, 0.15, 0.05])
        send_button = widgets.Button(send_ax, 'Send Angles to MCU')
        
        def on_send_angles(event):
            if self.connected and self.current_angles != (None, None):
                alpha, beta = self.current_angles
                self.send_angles_to_mcu(alpha, beta)
            else:
                print("Cannot send angles: not connected or no valid angles calculated")
        
        send_button.on_clicked(on_send_angles)
        
        # Add pen up/down buttons
        pen_up_ax = plt.axes([0.8, 0.7, 0.15, 0.05])
        pen_up_button = widgets.Button(pen_up_ax, 'Pen Up')
        
        def on_pen_up(event):
            if self.connected:
                self.pen_up()
        
        pen_up_button.on_clicked(on_pen_up)
        
        pen_down_ax = plt.axes([0.8, 0.65, 0.15, 0.05])
        pen_down_button = widgets.Button(pen_down_ax, 'Pen Down')
        
        def on_pen_down(event):
            if self.connected:
                self.pen_down()
        
        pen_down_button.on_clicked(on_pen_down)
        
        # Connect click event
        self.fig.canvas.mpl_connect('button_press_event', on_click)
        
        # Draw initial configuration
        self.draw_arm_config(50, 100)
        
        print("Click anywhere on the plot to test IK at that position!")
        print("Use the buttons to connect to the MCU and send angles.")
        print("Close the window to exit.")
        plt.show()

def test_specific_points():
    """Test specific points to verify IK"""
    visualizer = IKVisualizer()
    
    # Test points from your code
    test_points = [
        (1, 7),      # Your test case
        (50, 100),   # Center-ish
        (0, 150),    # Straight up
        (100, 50),   # Right side
        (-50, 100),  # Left side
        (80, 100),   # Your servo test point equivalent
        (60, 120),   # Your other servo test point equivalent
    ]
    
    for i, (x, y) in enumerate(test_points):
        plt.figure(figsize=(8, 6))
        visualizer.ax = plt.gca()
        visualizer.draw_arm_config(x, y)
        plt.title(f'Test Point {i+1}: ({x}, {y})')
        plt.show()

if __name__ == "__main__":
    print("Choose test mode:")
    print("1. Interactive test (click to set targets)")
    print("2. Test specific points")
    print("3. Interactive test with MCU control")
    
    choice = input("Enter choice (1, 2, or 3): ").strip()
    
    visualizer = IKVisualizer()
    
    if choice == "1":
        visualizer.interactive_test()
    elif choice == "3":
        # Ask for the serial port
        default_port = '/dev/cu.usbmodem14101'  # Default port
        port = input(f"Enter serial port (default: {default_port}): ").strip()
        if not port:
            port = default_port
        
        # Connect to MCU
        if visualizer.connect_to_mcu(port):
            print(f"Connected to MCU on {port}")
        
        # Start interactive test
        visualizer.interactive_test()
    else:
        test_specific_points()