import matplotlib.pyplot as plt
import matplotlib.patches as patches
import math
import numpy as np

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
        """Draw the current arm configuration with proper parallel mechanism display"""
        if clear_plot:
            self.ax.clear()
            self.ax.set_xlim(-150, 150)
            self.ax.set_ylim(-50, 200)
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
        
        # Add text info
        self.ax.text(-140, 180, f'Target: ({target_x:.1f}, {target_y:.1f})', fontsize=10)
        self.ax.text(-140, 170, f'Left shoulder angle: {alpha:.1f}°', fontsize=10, color='blue')
        self.ax.text(-140, 160, f'Right shoulder angle: {beta:.1f}°', fontsize=10, color='red')
        
        # Show link lengths and verification
        self.ax.text(-140, 140, f'L1 = {self.l1}mm', fontsize=10)
        self.ax.text(-140, 130, f'L2 = {self.l2}mm', fontsize=10)
        self.ax.text(-140, 120, f'Arm separation = {self.d}mm', fontsize=10)
        
        # Verify link lengths
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
            self.draw_arm_config(target_x, target_y)
            plt.draw()
        
        self.fig.canvas.mpl_connect('button_press_event', on_click)
        
        # Draw initial configuration
        self.draw_arm_config(50, 100)
        
        print("Click anywhere on the plot to test IK at that position!")
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
    
    choice = input("Enter choice (1 or 2): ").strip()
    
    if choice == "1":
        visualizer = IKVisualizer()
        visualizer.interactive_test()
    else:
        test_specific_points()