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

    def calcFK(self, t1, t2):
        """
        Calculate forward kinematics - find position from servo angles
        Based on the provided FK equations
        """
        try:
            # Convert angles to radians
            t1_rad = math.radians(t1)
            t2_rad = math.radians(t2)
            
            # Use the same parameters as IK
            l1 = self.l1  # first link length
            l2 = self.l2  # second link length
            dj = self.d   # distance between joints
            
            # Calculate intermediate values
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
            
            # Check if configuration is valid
            if db > 2 * l2:
                return None, None
            
            phi = math.acos(db / (2 * l2))
            phi1 = math.acos((db**2 + l1**2 - j1) / (2 * db * l1))
            
            s = l1**2 - 2 * l1 * l2 * math.cos(phi + phi1) + ((j1 + j2) / 2)
            
            # Calculate final position
            denominator = 2 * (k1 * k4 - k2 * k3)
            if abs(denominator) < 1e-10:  # Avoid division by zero
                return None, None
                
            y = (2 * k1 * s - (k3 * (j1 - j2))) / denominator
            x = (j1 - j2 - 2 * k2 * y) / (2 * k1) if abs(k1) > 1e-10 else 0
            
            return x, y
            
        except (ValueError, ZeroDivisionError) as e:
            print(f"Error in FK calculation: {e}")
            return None, None

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

    def drawLineOld(self, x1, y1):
        """
        Original drawLine method (kept for reference)
        """
        # Clamp coordinates
        x1 = max(min(x1, self.Xmax), self.Xmin)
        y1 = max(min(y1, self.Ymax), self.Ymin)

        C2 = self.C2
        totalDist = self.totalDist
        v = self.v

        if x1 < -C2:
            tx1 = math.atan((abs(x1) - C2) / (totalDist - y1))
            dx1 = math.sqrt((abs(x1) - C2) ** 2 + (totalDist - y1) ** 2)
            cx1 = math.acos(dx1 / 180)
            anglex1 = 135 - v * (tx1 + cx1)

            ty1 = math.atan((abs(x1) + C2) / (totalDist - y1))
            dy1 = math.sqrt((abs(x1) + C2) ** 2 + (totalDist - y1) ** 2)
            cy1 = math.acos(dy1 / 180)
            angley1 = 45 + v * (cy1 - ty1)

            self.servowrite(anglex1, angley1)

        elif -C2 <= x1 < 0:
            tx1 = math.atan((C2 - abs(x1)) / (totalDist - y1))
            dx1 = math.sqrt((C2 - abs(x1)) ** 2 + (totalDist - y1) ** 2)
            cx1 = math.acos(dx1 / 180)
            anglex1 = 135 - v * (cx1 - tx1)

            ty1 = math.atan((abs(x1) + C2) / (totalDist - y1))
            dy1 = math.sqrt((abs(x1) + C2) ** 2 + (totalDist - y1) ** 2)
            cy1 = math.acos(dy1 / 180)
            angley1 = 45 + v * (cy1 - ty1)

            self.servowrite(anglex1, angley1)

        elif 0 <= x1 < C2:
            tx1 = math.atan((x1 + C2) / (totalDist - y1))
            dx1 = math.sqrt((C2 + x1) ** 2 + (totalDist - y1) ** 2)
            cx1 = math.acos(dx1 / 180)
            anglex1 = 135 - v * (cx1 - tx1)

            ty1 = math.atan((C2 - x1) / (totalDist - y1))
            dy1 = math.sqrt((C2 - x1) ** 2 + (totalDist - y1) ** 2)
            cy1 = math.acos(dy1 / 180)
            angley1 = 45 + v * (cy1 - ty1)

            self.servowrite(anglex1, angley1)

        elif x1 >= C2:
            tx1 = math.atan((x1 + C2) / (totalDist - y1))
            dx1 = math.sqrt((x1 + C2) ** 2 + (totalDist - y1) ** 2)
            cx1 = math.acos(dx1 / 180)
            anglex1 = 135 - v * (cx1 - tx1)

            ty1 = math.atan((x1 - C2) / (totalDist - y1))
            dy1 = math.sqrt((x1 - C2) ** 2 + (totalDist - y1) ** 2)
            cy1 = math.acos(dy1 / 180)
            angley1 = 45 + v * (cy1 + ty1)

            self.servowrite(anglex1, angley1)

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
    time.sleep(5)
    
    # print("Press the button on IO25 to start the program...")
    # # Wait for button press to start
    # while stop_button.value():
    #     time.sleep(0.1)
    # print("Starting program...")
    # time.sleep(0.5)  # Debounce delay
    
    # # Test the IK implementation first
    # plotter.testIK()

    try:
        # Set movement speed to normal for better control
        plotter.setMovementSpeed('normal')
        
        while True:
            # Move to starting position first (pen up)
            print("Moving to start position...")
            plotter.servowrite(80, 100, smooth=True)
            time.sleep(0.8)  # Let servos settle
            
            # Put pen down and draw
            plotter.penDown()
            print("Drawing vertical line...")
            plotter.servowrite(60, 120, smooth=True)  # Draw the line 
            time.sleep(0.8)  # Let the line finish drawing
            
            # Lift pen
            plotter.penUp()
            time.sleep(0.3)
            
            # Return to center position
            print("Returning to center...")
            plotter.servowrite(90, 90, smooth=True)
            
            print("Cycle complete, waiting...")
            time.sleep(2.0)  # Wait before next cycle
    except KeyboardInterrupt:
        print("Program interrupted by user")
    except Exception as e:
        print(f"Error occurred: {e}")
    finally:
        # Make sure pen is up when exiting
        plotter.servowrite(90, 90, smooth=False)  # Quick reset to neutral position
        plotter.penUp()
        print("Program ended")


if __name__ == "__main__":
    main()