import serial
import time
import math

class SerialComm:
    """
    Class to handle serial communication with the SCARA plotter MCU
    """
    def __init__(self, port='/dev/tty.usbmodem110', baud=115200, timeout=10):
        """Initialize serial connection to the MCU"""
        self.port = port
        self.baud = baud
        self.serial = None
        self.connected = False
        
        try:
            self.serial = serial.Serial(port, baud, timeout=timeout)
            time.sleep(2)  # Wait for the connection to establish
            self.connected = True
            print(f"Connected to {port} at {baud} baud")
        except Exception as e:
            print(f"Error connecting to serial port: {e}")
    
    def send_command(self, command):
        """Send a command to the MCU and wait for acknowledgement"""
        if not self.connected or self.serial is None:
            print("Not connected to the MCU")
            return False
        
        try:
            self.serial.write(f"{command}\n".encode())
            response = self.serial.readline().decode().strip()
            if response == "ok":
                return True
            else:
                print(f"Unexpected response: {response}")
                return False
        except Exception as e:
            print(f"Error sending command: {e}")
            return False
    
    def move_to(self, x, y):
        """Send move command to position x,y"""
        return self.send_command(f"G1 X{x:.2f} Y{y:.2f}")
    
    def pen_up(self):
        """Lift the pen"""
        return self.send_command("U")
    
    def pen_down(self):
        """Lower the pen"""
        return self.send_command("D")
    
    def send_servo_angles(self, alpha, beta):
        """Send servo angles directly to the MCU"""
        return self.send_command(f"ANGLES {alpha:.2f} {beta:.2f}")
    
    def close(self):
        """Close the serial connection"""
        if self.serial:
            self.serial.close()
            self.connected = False
            print("Serial connection closed")
