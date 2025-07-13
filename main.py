from servo import Servo
import math
import time
import machine
from machine import Pin

class Plotter:
    def __init__(self):
        # pen lift servo angles
        self.penZUp = 23
        self.penZDown = 40

        self.C2 = 12.75  # C/2, where C is the distance between the two servos
        self.totalDist = 56.5  # distance btwn origin and servos midpoint
        self.v = 57.295  # 180/pi

        # SCARA IK parameters
        self.l1 = 110  # first link length
        self.l2 = 110  # second link length
        self.d = 25.5  # distance between arm origins

        # limits
        self.Xmax = 100
        self.Xmin = -100
        self.Ymax = 100
        self.Ymin = -100
        
        # Servo angle limits (degrees)
        self.servo_min = 0
        self.servo_max = 180

        # Initialize servos on GPIO pins (change pins as needed)
        self.xservo = Servo(6)
        self.yservo = Servo(7)
        self.penservo = Servo(8)

        self.actuatorPos = {'x': 0.0, 'y': 0.0}
        
        # Track current servo positions for smooth movement
        self.current_a = 90
        self.current_b = 90
        
        # Set default movement speed
        self.setMovementSpeed('normal')

        self.penUp()
        self.xservo.move(90)
        self.yservo.move(90)
        time.sleep(0.5)

    def penUp(self):
        self.penservo.move(self.penZUp)
        time.sleep(0.5)

    def penDown(self):
        self.penservo.move(self.penZDown)
        time.sleep(0.5)

    def servowrite(self, a, b, smooth=True):
        """
        Move servos to target angles with optional smooth movement
        """
        target_a = round(a)
        target_b = round(b)
        
        # Update current positions first
        if not hasattr(self, 'current_a'):
            self.current_a = 90
        if not hasattr(self, 'current_b'):
            self.current_b = 90
            
        current_a = self.current_a
        current_b = self.current_b
        
        if not smooth:
            # Direct movement (fast)
            self.xservo.move(target_a)
            self.yservo.move(target_b)
            self.current_a = target_a
            self.current_b = target_b
            time.sleep(0.05)
            return
        
        # Calculate movement distance
        diff_a = abs(target_a - current_a)
        diff_b = abs(target_b - current_b)
        max_diff = max(diff_a, diff_b)
        
        # For any movement, just move directly but with appropriate delay
        self.xservo.move(target_a)
        self.yservo.move(target_b)
        self.current_a = target_a
        self.current_b = target_b
        
        # Scale delay based on movement distance
        movement_delay = max(0.1, max_diff * 0.008)  # 8ms per degree of movement
        time.sleep(movement_delay)

    def ease_in_out(self, t):
        """
        Easing function for smoother servo movement
        """
        return t * t * (3.0 - 2.0 * t)

    def setMovementSpeed(self, speed='normal'):
        """
        Set movement speed for servo operations
        speed: 'slow', 'normal', 'fast'
        """
        if speed == 'slow':
            self.servo_delay = 0.1
            self.servo_steps_per_degree = 3
        elif speed == 'fast':
            self.servo_delay = 0.02
            self.servo_steps_per_degree = 0.5
        else:  # normal
            self.servo_delay = 0.05
            self.servo_steps_per_degree = 1.5
    def calcIK(self, x1, y1):
        """
        Calculate inverse kinematics for dual arm SCARA system
        Based on the provided IK equations from the diagram
        """
        # Convert input coordinates to the SCARA coordinate system
        x = x1
        y = y1
        
        # SCARA parameters
        a1 = self.l1  # first link length
        a2 = self.l2  # second link length
        d = self.d  # distance between arm origins

        try:
            # Calculate distances
            c = math.sqrt((x**2) + (y**2))  # distance from left arm origin to target
            e = math.sqrt(((d-x)**2) + (y**2))  # distance from right arm origin to target
            
            # Check if target is within reach
            if c > a1 + a2 or e > a1 + a2:
                print(f"Position ({x}, {y}) is out of reach")
                return None, None
            
            # Calculate angles directly (similar to the example)
            # Left arm angle calculation
            try:
                t1 = math.atan(y/x) + math.acos(((a1**2) + (c**2) - (a2**2))/(2*a1*c))
            except ZeroDivisionError:
            # Handle case where x = 0
                t1 = math.pi/2 + math.acos(((a1**2) + (c**2) - (a2**2))/(2*a1*c))
            
            # Right arm angle calculation
            try:
                t2 = math.atan(y/(d-x)) + math.acos(((a1**2) + (e**2) - (a2**2))/(2*a1*e))
            except ZeroDivisionError:
            # Handle case where d-x = 0
                t2 = math.pi/2 + math.acos(((a1**2) + (e**2) - (a2**2))/(2*a1*e))
            
            # Convert to degrees
            alpha_deg = math.degrees(t1)
            beta_deg = 180 - math.degrees(t2)  # As per the example
            
            return alpha_deg, beta_deg
            
        except (ValueError, ZeroDivisionError) as e:
            print(f"Error calculating IK for ({x}, {y}): {e}")
            return None, None

    def drawLine(self, x1, y1):
        """
        Move to target position using SCARA inverse kinematics
        """
        # Clamp coordinates
        x1 = max(min(x1, self.Xmax), self.Xmin)
        y1 = max(min(y1, self.Ymax), self.Ymin)

        # Calculate servo angles using IK
        alpha, beta = self.calcIK(x1, y1)
        
        if alpha is not None and beta is not None:
            # Update current position
            self.actuatorPos['x'] = x1
            self.actuatorPos['y'] = y1
            
            # Move servos to calculated angles
            self.servowrite(alpha, beta)
            print(f"Moving to ({x1}, {y1}) -> Servo angles: α={alpha:.1f}°, β={beta:.1f}°")
        else:
            print(f"Cannot reach position ({x1}, {y1})")

    def testIK(self):
        """
        Test the IK implementation with known values
        """
        print("Testing IK implementation...")
        
        # Test with the example values from the provided code
        test_x, test_y = 1, 7
        alpha, beta = self.calcIK(test_x, test_y)
        
        if alpha is not None and beta is not None:
            print(f"Test position ({test_x}, {test_y}):")
            print(f"Left arm angle (α): {alpha:.2f}°")
            print(f"Right arm angle (β): {beta:.2f}°")
            
            # Verify against the expected results from your code
            # Expected: t1 ≈ 55.77°, t2 ≈ 127.38°
            print(f"Expected results: α ≈ 55.77°, β ≈ 127.38°")
        else:
            print(f"Failed to calculate IK for test position ({test_x}, {test_y})")
        
        print("IK test complete.\n")
def main():
    # Setup IO25 as an input with a pull-up resistor
    stop_button = Pin(25, Pin.IN, Pin.PULL_UP)
    
    plotter = Plotter()
    plotter.penUp()
    
    # Set both servos to 90 degrees
    plotter.servowrite(90, 90, smooth=False)
    time.sleep(1)
    
    print("Ready to receive servo angle pairs")
    print("Format: 'angle_a,angle_b' (e.g. '120,60')")
    print("Type 'exit' to quit")

    try:
        while True:
            # Wait for input from computer
            input_data = input("Enter angles (a,b): ")
            
            # Check for exit command
            if input_data.lower() == 'exit':
                print("Exiting program...")
                break
                
            # Parse input
            try:
                parts = input_data.strip().split(',')
                if len(parts) == 2:
                    angle_a = float(parts[0])
                    angle_b = float(parts[1])
                    
                    # Clamp angles to safe limits
                    angle_a = max(min(angle_a, plotter.servo_max), plotter.servo_min)
                    angle_b = max(min(angle_b, plotter.servo_max), plotter.servo_min)
                    
                    print(f"Moving to angles: a={angle_a}, b={angle_b}")
                    plotter.servowrite(angle_a, angle_b, smooth=True)
                else:
                    print("Error: Invalid format. Use 'angle_a,angle_b'")
            except ValueError:
                print("Error: Invalid angles. Please enter numbers only.")
                
    except KeyboardInterrupt:
        print("Program interrupted by user")
    except Exception as e:
        print(f"Error occurred: {e}")
    finally:
        # Make sure pen is up when exiting
        plotter.servowrite(90, 90, smooth=False)  # Reset to neutral position
        plotter.penUp()
        print("Program ended")


if __name__ == "__main__":
    main()
