import sys
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
        
        # Safe zone parameters (configurable paper area)
        self.safe_zone_enabled = True
        self.safe_zone_width = 100  # mm
        self.safe_zone_height = 100  # mm
        self.safe_zone_x = -50  # mm (left edge position)
        self.safe_zone_y = -200  # mm (bottom edge position)
        
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
        x = x1
        y = y1
        a1 = self.l1  # first link length
        a2 = self.l2  # second link length
        d = self.d  # distance between arm origins

        try:
            # Calculate distances
            c = math.sqrt((x**2) + (y**2))  # distance from left arm origin to target
            e = math.sqrt(((d-x)**2) + (y**2))  # distance from right arm origin to target
            
            # Check if target is within reach for both arms
            if c > a1 + a2 or c < abs(a1 - a2):
                print(f"Position ({x}, {y}) is out of reach for left arm")
                return None, None
            if e > a1 + a2 or e < abs(a1 - a2):
                print(f"Position ({x}, {y}) is out of reach for right arm")
                return None, None
                
            # Calculate elbow angles
            cos_q2_left = (c**2 - a1**2 - a2**2) / (2 * a1 * a2)
            if abs(cos_q2_left) > 1:
                print(f"Left arm: Invalid cosine value {cos_q2_left}")
                return None, None
            q2_left = math.acos(cos_q2_left)
            
            cos_q2_right = (e**2 - a1**2 - a2**2) / (2 * a1 * a2)
            if abs(cos_q2_right) > 1:
                print(f"Right arm: Invalid cosine value {cos_q2_right}")
                return None, None
            q2_right = -math.acos(cos_q2_right)
            
            # Calculate shoulder angles
            if x == 0:
                q1_left = math.pi/2 if y > 0 else -math.pi/2
            else:
                k1 = a1 + a2 * math.cos(q2_left)
                k2 = a2 * math.sin(q2_left)
                q1_left = math.atan2(y, x) - math.atan2(k2, k1)
                
            x_right = x - d
            if x_right == 0:
                q1_right = math.pi/2 if y > 0 else -math.pi/2
            else:
                k1_right = a1 + a2 * math.cos(q2_right)
                k2_right = a2 * math.sin(q2_right)
                q1_right = math.atan2(y, x_right) - math.atan2(k2_right, k1_right)
            
            # Convert to degrees for servo control
            alpha_deg = math.degrees(q1_left)
            beta_deg = math.degrees(q1_right)
            
            # IMPORTANT: For display purposes, we want to use the mirrored angles
            # similar to what's shown in the visualizer
            display_y = -y if y > 0 else y
            display_alpha, display_beta = self.calcDisplayAngles(x, display_y)
            
            # Return display angles for actual servo control
            return -display_alpha, -display_beta
            
        except (ValueError, ZeroDivisionError) as e:
            print(f"Error calculating IK for ({x}, {y}): {e}")
            return None, None
            
    def calcDisplayAngles(self, x, y):
        """
        Helper function to calculate display angles (mirrored angles)
        These are the angles shown in the visualizer
        """
        a1 = self.l1
        a2 = self.l2
        d = self.d
        
        try:
            c = math.sqrt((x**2) + (y**2))
            e = math.sqrt(((d-x)**2) + (y**2))
            
            if c > a1 + a2 or c < abs(a1 - a2) or e > a1 + a2 or e < abs(a1 - a2):
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
                
            return math.degrees(q1_left), math.degrees(q1_right)
            
        except (ValueError, ZeroDivisionError):
            return None, None

    def drawLine(self, x1, y1):
        """
        Move to target position using SCARA inverse kinematics
        """
        # Clamp coordinates
        x1 = max(min(x1, self.Xmax), self.Xmin)
        y1 = max(min(y1, self.Ymax), self.Ymin)
        
        # Check if the point is within the safe zone (paper area)
        in_safe_zone = self.is_point_in_safe_zone(x1, y1)
        if not in_safe_zone and self.safe_zone_enabled:
            print(f"WARNING: Position ({x1}, {y1}) is outside the safe zone (paper area)")
            # Continue anyway, but give a warning

        # Calculate servo angles using IK
        alpha, beta = self.calcIK(x1, y1)
        
        if alpha is not None and beta is not None:
            # Update current position
            self.actuatorPos['x'] = x1
            self.actuatorPos['y'] = y1
            
            # Move servos to calculated angles
            self.servowrite(alpha, beta)
            print(f"Moving to ({x1}, {y1}) -> Servo angles: α={alpha:.1f}°, β={beta:.1f}°")
            if not in_safe_zone and self.safe_zone_enabled:
                print("CAUTION: Point is outside the paper area!")
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
    
    def is_point_in_safe_zone(self, x, y):
        """
        Check if a point is within the safe zone (paper area)
        Returns True if the point is within the safe zone, False otherwise
        """
        return (self.safe_zone_enabled and
                self.safe_zone_x <= x <= self.safe_zone_x + self.safe_zone_width and
                self.safe_zone_y - self.safe_zone_height <= y <= self.safe_zone_y)
def read_serial_input():
    """Read input from the serial port with timeout"""
    import machine
    import utime
    
    # Use the hardware UART connected to your USB/Serial adapter
    # Adjust UART number (0 or 1) and baud rate as needed for your board
    uart = machine.UART(0, baudrate=115200)
    uart.init(115200, bits=8, parity=None, stop=1)
    
    # Clear any pending data
    uart.read()
    
    # Wait for a line of input with timeout
    data = b''
    start_time = utime.time()
    timeout = 0.1  # 100ms timeout
    
    while not data.endswith(b'\n') and (utime.time() - start_time) < timeout:
        if uart.any():
            char = uart.read(1)
            if char:
                data += char
        utime.sleep(0.01)
    
    return data.decode().strip()
def main():
    # Setup IO25 as an input with a pull-up resistor
    stop_button = Pin(25, Pin.IN, Pin.PULL_UP)
    
    plotter = Plotter()
    plotter.penUp()
    
    # Set both servos to 90 degrees
    plotter.servowrite(90, 90, smooth=False)
    time.sleep(1)
    
    print("\n\n***** PLOTTER READY *****")
    print("Ready to receive commands")
    print("Commands:")
    print("  'angle_a,angle_b' - Move to servo angles (e.g. '120,60')")
    print("  'xy:x,y' - Move to XY coordinates (e.g. 'xy:50,80')")
    print("  'pen:up' or 'pen:down' - Move pen up or down")
    print("  'safe:x,y,w,h' - Set safe zone (paper) parameters (e.g. 'safe:-50,100,100,100')")
    print("  'safe:on' or 'safe:off' - Enable or disable safe zone checks")
    print("  'exit' - Quit program")
    print("**************************")

    try:
        while True:
            # Print a prompt
            print("\nEnter command > ", end="")
            
            # Wait for input using a simpler approach
            input_data = ""
            try:
                # Try to use regular input first (works in REPL and many serial terminals)
                input_data = input().strip()
            except Exception:
                # If input() fails, fall back to reading from stdin directly
                input_data = sys.stdin.readline().strip()
                
            # Print confirmation of received command
            print(f"\nReceived command: '{input_data}'")
            
            # Check for exit command
            if input_data.lower() == 'exit':
                print("Exiting program...")
                break
                
            # Parse input
            try:
                # Check if it's a coordinate command
                if input_data.startswith('xy:'):
                    coords = input_data[3:].strip().split(',')
                    if len(coords) == 2:
                        x = float(coords[0])
                        y = float(coords[1])
                        print(f"Moving to coordinates: ({x}, {y})")
                        plotter.drawLine(x, y)
                    else:
                        print("Error: Invalid format. Use 'xy:x,y'")
                
                # Check if it's a pen command
                elif input_data.startswith('pen:'):
                    pen_action = input_data[4:].strip().lower()
                    if pen_action == 'up':
                        plotter.penUp()
                        print("Pen moved up")
                    elif pen_action == 'down':
                        plotter.penDown()
                        print("Pen moved down")
                    else:
                        print("Error: Invalid pen command. Use 'pen:up' or 'pen:down'")
                
                # Check if it's a safe zone command
                elif input_data.startswith('safe:'):
                    safe_params = input_data[5:].strip()
                    if safe_params.lower() == 'on':
                        plotter.safe_zone_enabled = True
                        print(f"Safe zone checks enabled")
                    elif safe_params.lower() == 'off':
                        plotter.safe_zone_enabled = False
                        print(f"Safe zone checks disabled")
                    else:
                        try:
                            params = safe_params.split(',')
                            if len(params) == 4:
                                x = float(params[0])
                                y = float(params[1])
                                w = float(params[2])
                                h = float(params[3])
                                plotter.safe_zone_x = x
                                plotter.safe_zone_y = y
                                plotter.safe_zone_width = w
                                plotter.safe_zone_height = h
                                print(f"Safe zone set to: x={x}, y={y}, width={w}, height={h}")
                            else:
                                print("Error: Invalid format. Use 'safe:x,y,width,height'")
                        except ValueError:
                            print("Error: Invalid safe zone parameters. Use numbers only.")
                
                # Otherwise assume it's a servo angle pair
                else:
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
                print("Error: Invalid parameters. Please enter numbers correctly.")
                
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
