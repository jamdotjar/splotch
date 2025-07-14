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
        self.safe_zone_y = -100  # mm (bxottom edge position)
        
        # Serial communication
        self.serial_port = None
        self.connected = False
        self.serial_baudrate = 115200  # Default baudrate
        self.last_serial_activity = 0  # Track when we last used the serial port
        self.serial_heartbeat_interval = 10  # Seconds between heartbeats
        
        # Keep track of current position and pen state
        self.current_x = 0
        self.current_y = 0
        self.pen_down = False
        
        # Set up the plot with fixed size and axes limits
        self.fig = plt.figure(figsize=(12, 10))
        
        # Create a grid layout for our plot and controls
        self.gs = self.fig.add_gridspec(2, 2, height_ratios=[5, 1], width_ratios=[4, 1])
        
        # Main plot area
        self.ax = self.fig.add_subplot(self.gs[0, :])
        self.ax.set_xlim(-150, 150)
        self.ax.set_ylim(-200, 50)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_title('SCARA Dual Arm IK Visualization')
        
        # Controls area
        self.controls_ax = self.fig.add_subplot(self.gs[1, 0])
        self.controls_ax.set_axis_off()
        
        # Status area
        self.status_ax = self.fig.add_subplot(self.gs[1, 1])
        self.status_ax.set_axis_off()
        
        # Arm origins
        self.left_origin = np.array([0, 0])
        self.right_origin = np.array([self.d, 0])
        
        # Add control buttons
        self.setup_buttons()
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
        
        # Connect button
        self.btn_connect_pos = [start_x, start_y, button_width, button_height]
        self.btn_connect = widgets.Button(
            self.controls_ax.inset_axes(self.btn_connect_pos),
            'Connect',
            color='yellow',
            hovercolor='gold'
        )
        self.btn_connect.on_clicked(self.on_connect)
        
        # Send position button
        send_pos = [start_x + (button_width + button_spacing), start_y, button_width, button_height]
        self.btn_send = widgets.Button(
            self.controls_ax.inset_axes(send_pos),
            'Send Position',
            color='lightblue',
            hovercolor='skyblue'
        )
        self.btn_send.on_clicked(self.on_send_position)
        
        # Pen up button
        pen_up_pos = [start_x + 2 * (button_width + button_spacing), start_y, button_width, button_height]
        self.btn_pen_up = widgets.Button(
            self.controls_ax.inset_axes(pen_up_pos),
            'Pen Up',
            color='lightgreen',
            hovercolor='lime'
        )
        self.btn_pen_up.on_clicked(self.on_pen_up)
        
        # Pen down button
        pen_down_pos = [start_x + 3 * (button_width + button_spacing), start_y, button_width, button_height]
        self.btn_pen_down = widgets.Button(
            self.controls_ax.inset_axes(pen_down_pos),
            'Pen Down',
            color='salmon',
            hovercolor='tomato'
        )
        self.btn_pen_down.on_clicked(self.on_pen_down)
        
        # Toggle safe zone button
        toggle_pos = [start_x + 4 * (button_width + button_spacing), start_y, button_width, button_height]
        self.btn_toggle_safe = widgets.Button(
            self.controls_ax.inset_axes(toggle_pos),
            'Toggle Safe Zone',
            color='lightgray',
            hovercolor='darkgray'
        )
        self.btn_toggle_safe.on_clicked(self.on_toggle_safe)
    def find_serial_ports(self):
        """Find available serial ports on the system"""
        if sys.platform.startswith('win'):
            ports = ['COM%s' % (i + 1) for i in range(256)]
        elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            ports = glob.glob('/dev/tty[A-Za-z]*')
        elif sys.platform.startswith('darwin'):
            # On macOS, expand to include all common USB-Serial port patterns
            ports = glob.glob('/dev/tty.usbmodem*') + glob.glob('/dev/tty.usbserial*') + glob.glob('/dev/tty.SLAB_USBtoUART*')
        else:
            raise EnvironmentError('Unsupported platform')

        available_ports = []
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                available_ports.append(port)
            except (OSError, serial.SerialException):
                pass
                
        return available_ports
        
    def on_connect(self, event):
        """Connect to the microcontroller via serial port"""
        if self.connected or self.serial_port or hasattr(self, '_port_reference'):
            # Disconnect
            try:
                print("Disconnecting from device...")
                # First close the instance variable
                if hasattr(self, 'serial_port') and self.serial_port:
                    if self.serial_port.is_open:
                        self.serial_port.close()
                    self.serial_port = None
                    
                # Then close the persistent reference if it exists
                if hasattr(self, '_port_reference'):
                    if self._port_reference and self._port_reference.is_open:
                        self._port_reference.close()
                    delattr(self, '_port_reference')
                    
                self.connected = False
                self.btn_connect.label.set_text('Connect')
                self.update_status("Disconnected from device")
                self.fig.canvas.draw_idle()
                print("Successfully disconnected")
            except Exception as e:
                import traceback
                traceback.print_exc()
                self.update_status(f"Error disconnecting: {e}")
            return
            
        # Find ports
        available_ports = self.find_serial_ports()
        
        if not available_ports:
            self.update_status("No serial ports found! Try specifying port directly.")
            
            # Allow manual entry if no ports found
            port = input("Enter serial port manually (e.g. /dev/tty.usbmodem101): ").strip()
            if not port:
                self.update_status("No port specified")
                return
        else:
            # Show available ports and let user choose
            print("Available ports:")
            for i, port in enumerate(available_ports):
                print(f"{i+1}. {port}")
                
            try:
                selection = input(f"Select port (1-{len(available_ports)}) or enter full port name: ").strip()
                # Clean input from escape sequences or control characters
                selection = ''.join(c for c in selection if ord(c) >= 32 and ord(c) < 127)
                
                if selection.isdigit() and 1 <= int(selection) <= len(available_ports):
                    port = available_ports[int(selection)-1]
                else:
                    port = selection  # Use directly entered port name
                    
                # Double check that we have a valid port string
                if not port or any(c for c in port if ord(c) < 32):
                    self.update_status("Invalid port selection")
                    return
                    
            except (ValueError, IndexError):
                self.update_status("Invalid selection")
                return
            
        # Prevent immediate finalization of the program
        print(f"\nAttempting to connect to {port}...")
            
        # Try to connect to the selected port
        try:
            self.update_status(f"Connecting to {port}...")
            
            # Open the port with more verbose error reporting
            print(f"Opening serial port {port} at {self.serial_baudrate} baud")
            
            # Additional validation of the port string
            if not isinstance(port, str) or not port or any(c for c in port if ord(c) < 32):
                raise ValueError(f"Invalid port name: {repr(port)}")
                
            # Create a class attribute to store the port reference
            # This is CRITICAL to prevent the port from being garbage collected
            self._port_reference = serial.Serial(
                port=port,
                baudrate=self.serial_baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=2,
                write_timeout=2,
                dsrdtr=False,      # No hardware flow control
                rtscts=False,      # No hardware flow control
                xonxoff=False,     # No software flow control
            )
            
            # Assign to the instance variable AFTER successful creation
            self.serial_port = self._port_reference
            print("Port opened successfully")
            
            # Wait longer for connection to stabilize
            time.sleep(2)
            
            # Flush any pending data
            print("Clearing input/output buffers")
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            
            # Test connection by sending a simple command
            self.update_status("Testing connection...")
            print("Sending test command (90,90)")
            
            # Use both carriage return AND newline for maximum compatibility
            print("Writing to port...")
            self.serial_port.write(b"90,90\r\n")
            self.serial_port.flush()
            print("Command sent")
            
            # Wait longer before checking for a response
            time.sleep(1);
            
            # Check if there's any response
            if self.serial_port.in_waiting:
                data = self.serial_port.read(self.serial_port.in_waiting)
                print(f"Response received: {data}")
                self.connected = True
                self.btn_connect.label.set_text('Disconnect')
                self.update_status(f"Connected to {port}")
            else:
                print("No immediate response, but will use port anyway")
                self.update_status(f"No immediate response from {port}, but connection established")
                self.connected = True
                self.btn_connect.label.set_text('Disconnect')
            
        except Exception as e:
            import traceback
            traceback.print_exc()  # Print the full exception traceback
            print(f"Connection error: {e}")
            self.update_status(f"Connection error: {e}")
            # Make sure to clean up both references properly
            if hasattr(self, 'serial_port') and self.serial_port:
                try:
                    if self.serial_port.is_open:
                        self.serial_port.close()
                    print("Serial port closed due to error")
                except Exception as close_err:
                    print(f"Error while closing port: {close_err}")
                finally:
                    self.serial_port = None
                    
            # Only delete the port reference after handling the serial_port attribute
            if hasattr(self, '_port_reference'):
                try:
                    if self._port_reference and self._port_reference.is_open:
                        self._port_reference.close()
                except Exception as ref_close_err:
                    print(f"Error while closing port reference: {ref_close_err}")
                finally:
                    delattr(self, '_port_reference')
                    
            self.connected = False
        
        self.fig.canvas.draw_idle()
    
    def send_command(self, command):
        """Send a command string to the device"""
        if not self.connected or not self.serial_port:
            self.update_status("Not connected to device")
            return False
            
        try:
            # First check if port is still open
            if not self.serial_port.is_open:
                print("Port closed unexpectedly! Attempting to reopen...")
                try:
                    self.serial_port.open()
                    time.sleep(1)  # Wait for port to stabilize
                except Exception as reopen_err:
                    print(f"Failed to reopen port: {reopen_err}")
                    self.connected = False
                    return False
                    
            # Add both CR and LF for maximum compatibility with different devices
            full_command = f"{command}\r\n"
            print(f"Sending command: '{command}'")
            
            # Write command bytes directly with careful error handling
            try:
                bytes_written = self.serial_port.write(full_command.encode())
                print(f"Wrote {bytes_written} bytes")
                
                # Flush to ensure all data is sent
                self.serial_port.flush()
                print("Command flushed to port")
                
                # Track the time of last activity
                self.last_serial_activity = time.time()
                
                # Add a small delay after sending to let the device process
                time.sleep(0.1)
                
                return True
            except serial.SerialTimeoutException:
                print("Write timeout occurred")
                return False
                
        except Exception as e:
            import traceback
            traceback.print_exc()
            self.update_status(f"Send error: {e}")
            
            # Handle disconnection
            if "Port is closed" in str(e) or "Device disconnected" in str(e):
                self.connected = False
                self.update_status("Device disconnected")
                
            return False
    
    def read_response(self, timeout=3):
        """Read response from the device"""
        if not self.connected or not self.serial_port:
            return None
            
        try:
            # Check if port is still open first
            if not self.serial_port.is_open:
                print("WARNING: Port is closed when trying to read! Attempting to reopen...")
                try:
                    self.serial_port.open()
                except Exception as reopenError:
                    print(f"Could not reopen port: {reopenError}")
                    self.connected = False
                    self.update_status("Connection lost - port closed")
                    return None
            
            print(f"Waiting for response (timeout: {timeout}s)")
            
            # Set timeout for reading
            original_timeout = self.serial_port.timeout
            self.serial_port.timeout = timeout
            
            # First check if there's any data already waiting
            try:
                if self.serial_port.in_waiting > 0:
                    print(f"{self.serial_port.in_waiting} bytes already in buffer")
            except Exception as e:
                print(f"Error checking buffer: {e}")
                self.update_status("Connection error - reconnect required")
                self.connected = False
                return None
            
            # Read with a timeout
            start_time = time.time()
            all_data = bytearray()
            response = ""
            
            # Keep reading until we get data or timeout
            while time.time() - start_time < timeout:
                try:
                    if self.serial_port.in_waiting > 0:
                        # Read everything that's available
                        chunk = self.serial_port.read(self.serial_port.in_waiting)
                        if chunk:
                            all_data.extend(chunk)
                            print(f"Read chunk: {chunk}")
                            
                            # Track the time of last activity
                            self.last_serial_activity = time.time()
                            
                            # Try to decode what we have so far
                            try:
                                response = all_data.decode('utf-8', errors='replace').strip()
                                print(f"Current response: '{response}'")
                                
                                # If we have a complete line, we're done
                                if '\n' in response or '\r' in response:
                                    break
                            except Exception as e:
                                print(f"Decode error: {e}")
                except Exception as read_error:
                    print(f"Error during read loop: {read_error}")
                    break
                
                # Brief delay to prevent CPU hogging
                time.sleep(0.05)
            
            # Restore original timeout
            try:
                self.serial_port.timeout = original_timeout
            except Exception:
                # If setting timeout fails, port may be closed/invalid
                pass
            
            if all_data:
                # Try one final decode of everything we received
                try:
                    final_response = all_data.decode('utf-8', errors='replace').strip()
                    print(f"Final response data: '{final_response}'")
                    return final_response
                except Exception as e:
                    print(f"Final decode error: {e}")
                    return str(all_data)
            else:
                print("No data received within timeout period")
                return None
                
        except Exception as e:
            import traceback
            traceback.print_exc()
            self.update_status(f"Read error: {e}")
            
            # Check if we need to reconnect
            if "Port is closed" in str(e):
                self.update_status("Serial port was closed. Reconnection required.")
                self.connected = False
                
            return None

    def send_heartbeat(self):
        """Send a heartbeat command to keep the connection alive"""
        if not self.connected or not self.serial_port:
            return False
            
        # Only send heartbeat if we haven't communicated recently
        current_time = time.time()
        if current_time - self.last_serial_activity < self.serial_heartbeat_interval:
            return True
            
        # Send a simple command that won't affect operation
        try:
            if not self.serial_port.is_open:
                print("Port closed when attempting heartbeat - reconnecting...")
                try:
                    self.serial_port.open()
                    time.sleep(0.5)
                except Exception as e:
                    print(f"Failed to reopen port for heartbeat: {e}")
                    self.connected = False
                    return False
                    
            # Use a simple status request command
            self.serial_port.write(b"status\r\n")
            self.serial_port.flush()
            self.last_serial_activity = current_time
            return True
            
        except Exception as e:
            print(f"Heartbeat failed: {e}")
            if "Port is closed" in str(e) or "Device disconnected" in str(e):
                self.connected = False
            return False
    
    def on_send_position(self, event):
        """Send the current position to the device"""
        if not self.connected or not self.serial_port:
            self.update_status("Not connected to device")
            return
        
        # Check if port is still valid
        if not hasattr(self, '_port_reference') or not self._port_reference.is_open:
            self.update_status("Connection lost. Please reconnect.")
            self.connected = False
            return
            
        # Format command for the plotter using xy: format
        command = f"xy:{self.current_x:.1f},{self.current_y:.1f}"
        
        if self.send_command(command):
            self.update_status(f"Sent: {command}")
            response = self.read_response()
            if response:
                self.update_status(f"Response: {response}")
            else:
                self.update_status(f"Command sent, no response received")
    
    def on_pen_up(self, event):
        """Send pen up command to the device"""
        if not self.connected or not self.serial_port:
            self.update_status("Not connected to device")
            return
            
        # Check if port is still valid
        if not hasattr(self, '_port_reference') or not self._port_reference.is_open:
            self.update_status("Connection lost. Please reconnect.")
            self.connected = False
            return
            
        self.pen_down = False
        command = "pen:up"
        
        if self.send_command(command):
            self.update_status(f"Sent: {command}")
            response = self.read_response()
            if response:
                self.update_status(f"Response: {response}")
            else:
                self.update_status(f"Command sent, no response received")
                
        self.draw_arm_config(self.current_x, self.current_y)
    def on_pen_down(self, event):
        """Send pen down command to the device"""
        if not self.connected or not self.serial_port:
            self.update_status("Not connected to device")
            return
            
        # Check if port is still valid
        if not hasattr(self, '_port_reference') or not self._port_reference.is_open:
            self.update_status("Connection lost. Please reconnect.")
            self.connected = False
            return
            
        self.pen_down = True
        command = "pen:down"
        
        if self.send_command(command):
            self.update_status(f"Sent: {command}")
            response = self.read_response()
            if response:
                self.update_status(f"Response: {response}")
            else:
                self.update_status(f"Command sent, no response received")
                
        self.draw_arm_config(self.current_x, self.current_y)
    
    def on_toggle_safe(self, event):
        """Toggle the safe zone visibility"""
        self.toggle_safe_zone()
        self.draw_arm_config(self.current_x, self.current_y)
        status = "visible" if self.safe_zone_enabled else "hidden"
        self.update_status(f"Safe zone: {status}")
        
        # Send safe zone toggle command if connected
        if self.connected and self.serial_port:
            command = f"safe:{'on' if self.safe_zone_enabled else 'off'}"
            if self.send_command(command):
                response = self.read_response()
                if response:
                    self.update_status(f"Response: {response}")
                
    def update_status(self, message):
        """Update status display"""
        self.status_ax.clear()
        self.status_ax.text(0.5, 0.5, message, 
                           ha='center', va='center', fontsize=12)
        self.status_ax.set_axis_off()
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
    
    def draw_arm_config(self, target_x, target_y, clear_plot=True):
        # Store the current position
        self.current_x = target_x
        self.current_y = target_y
        
        if clear_plot:
            self.ax.clear()
            self.ax.set_xlim(-150, 150)
            self.ax.set_ylim(-250, 50)
            self.ax.set_aspect('equal')
            self.ax.grid(True, alpha=0.3)
            self.ax.set_title('SCARA Parallel Dual Arm IK Visualization')
            
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
        
        # Calculate IK
        alpha, beta = self.calcIK(target_x, target_y)
        
        if alpha is None or beta is None:
            self.ax.text(0, 0, f"Target ({target_x:.1f}, {target_y:.1f}) unreachable!", 
                        fontsize=12, color='red', ha='center')
            self.update_status("Position unreachable")
            self.fig.canvas.draw_idle()
            return
        
        # Convert to radians for drawing
        alpha_rad = math.radians(alpha)
        beta_rad = math.radians(beta)

        # Calculate elbow positions
        left_elbow = self.left_origin + self.l1 * np.array([math.cos(alpha_rad), math.sin(alpha_rad)])
        right_elbow = self.right_origin + self.l1 * np.array([math.cos(beta_rad), math.sin(beta_rad)])
        
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
        # Use different marker based on pen state
        end_marker = 'D' if self.pen_down else 'o'
        end_color = 'darkgreen' if self.pen_down else 'green'
        self.ax.plot(target_x, target_y, marker=end_marker, color=end_color, 
                     markersize=12, label=f'End Effector (Pen {"Down" if self.pen_down else "Up"})')
        
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
        
        # Add neat status display in the corner instead of cluttering the plot
        status_text = [
            f'Target: ({target_x:.1f}, {target_y:.1f})',
            f'Pen: {"DOWN" if self.pen_down else "UP"}',
            f'Left angle: {180--alpha:.1f}°',
            f'Right angle: {180--beta:.1f}°'
        ]
        
        # Add a cleaner status box
        y_pos = -180
        for text in status_text:
            self.ax.text(-140, y_pos, text, fontsize=10, 
                        bbox=dict(facecolor='white', alpha=0.7, boxstyle='round'))
            y_pos += 15
        
        # Add a legend but with better positioning
        self.ax.legend(loc='upper left', bbox_to_anchor=(1.01, 1))
        
        # Check if point is in safe zone
        in_safe_zone = self.is_point_in_safe_zone(target_x, target_y)
        zone_status = "SAFE: Within paper area" if in_safe_zone else "CAUTION: Outside paper area!"
        zone_color = "green" if in_safe_zone else "red"
        
        # Update status display
        self.update_status(zone_status)
        
        # Draw without adjusting the plot view to avoid movement
        self.fig.canvas.draw_idle()
    
    def interactive_test(self):
        """Interactive test - click to set target positions"""
        def on_click(event):
            if event.inaxes != self.ax:
                return
            target_x = event.xdata
            target_y = event.ydata
            self.draw_arm_config(target_x, target_y)
        
        # Connect click handler
        self.fig.canvas.mpl_connect('button_press_event', on_click)
        
        # Add a heartbeat timer
        def heartbeat_callback(event):
            if self.connected:
                self.send_heartbeat()
        
        # Create a timer for heartbeat (every 5 seconds)
        heartbeat_timer = self.fig.canvas.new_timer(interval=5000)
        heartbeat_timer.add_callback(heartbeat_callback)
        heartbeat_timer.start()
        
        # Add a close event handler to ensure clean shutdown
        def handle_close(event):
            print("Window close event detected, cleaning up...")
            if hasattr(self, 'serial_port') and self.serial_port:
                try:
                    if self.serial_port.is_open:
                        self.serial_port.close()
                    print("Serial connection closed on window close")
                except Exception as e:
                    print(f"Error during cleanup: {e}")
            
        self.fig.canvas.mpl_connect('close_event', handle_close)
        
        # Draw initial configuration
        self.draw_arm_config(50, 100)
        
        print("Interactive IK Visualizer with Serial Control")
        print("-------------------------------------------")
        print("Click anywhere on the plot to test IK at that position")
        print("Use the buttons to control the plotter:")
        print("  - Connect: Connect/disconnect to the microcontroller")
        print("  - Send Position: Send current position to plotter")
        print("  - Pen Up/Down: Control the pen position")
        print("  - Toggle Safe Zone: Enable/disable safe zone check")
        print("\nAvailable Serial Commands:")
        print("  'angle_a,angle_b' - Move to servo angles (e.g. '120,60')")
        print("  'xy:x,y' - Move to XY coordinates (e.g. 'xy:50,80')")
        print("  'pen:up' or 'pen:down' - Move pen up or down")
        print("  'safe:x,y,w,h' - Set safe zone parameters (e.g. 'safe:-50,100,100,100')")
        print("  'safe:on' or 'safe:off' - Enable or disable safe zone checks")
        print("  'exit' - Quit program")
        
        plt.tight_layout()
        plt.subplots_adjust(bottom=0.15)  # Make room for buttons
        
        # Use this instead of plt.show() to better handle exceptions
        try:
            plt.show(block=True)
        except Exception as e:
            print(f"Exception in matplotlib event loop: {e}")
            import traceback
            traceback.print_exc()

    def set_safe_zone(self, x, y, width, height, enabled=True):
        """Configure the safe zone (paper area) dimensions and position"""
        self.safe_zone_x = x
        self.safe_zone_y = y
        self.safe_zone_width = width
        self.safe_zone_height = height
        self.safe_zone_enabled = enabled
        
    def toggle_safe_zone(self):
        """Toggle the visibility of the safe zone"""
        self.safe_zone_enabled = not self.safe_zone_enabled
        
    def is_point_in_safe_zone(self, x, y):
        """Check if a point is within the safe zone"""
        return (self.safe_zone_x <= x <= self.safe_zone_x + self.safe_zone_width and
                self.safe_zone_y - self.safe_zone_height <= y <= self.safe_zone_y)

if __name__ == "__main__":
    # Register cleanup function
    def cleanup():
        print("Program exiting, cleaning up resources...")
        # Any additional cleanup can be done here
    
    atexit.register(cleanup)
    
    visualizer = None
    try:
        print("Starting IK Visualizer...")
        visualizer = IKVisualizer()
        visualizer.interactive_test()
    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
    except Exception as e:
        import traceback
        print(f"Unexpected error: {e}")
        traceback.print_exc()
    finally:
        # Clean up serial connection if it exists
        if visualizer and hasattr(visualizer, 'serial_port') and visualizer.serial_port:
            try:
                visualizer.serial_port.close()
                print("Serial connection closed")
            except Exception as e:
                print(f"Error closing serial connection: {e}")
        
        # Close all matplotlib figures
        plt.close('all')
        print("Visualization terminated.")
