#!/usr/bin/env python3
import serial
import time
import sys
import glob
import os

def find_ports():
    """Find available serial ports"""
    if sys.platform.startswith('darwin'):  # macOS
        ports = []
        ports.extend(glob.glob('/dev/tty.usbmodem*'))
        ports.extend(glob.glob('/dev/tty.usbserial*'))
        ports.extend(glob.glob('/dev/cu.usbmodem*'))
        ports.extend(glob.glob('/dev/cu.usbserial*'))
    elif sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    else:
        ports = glob.glob('/dev/tty[A-Za-z]*')
    
    available_ports = []
    for port in ports:
        try:
            s = serial.Serial(port, baudrate=115200, timeout=0.5)
            s.close()
            available_ports.append(port)
        except (OSError, serial.SerialException):
            pass
            
    return available_ports

def send_command_interactive(port, baudrate=115200):
    """Interactive serial terminal that sends commands and shows responses"""
    try:
        # Open serial port
        print(f"\nOpening port {port} at {baudrate} baud...")
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
        
        # Wait for the port to be ready
        time.sleep(0.5)
        
        # Clear initial data
        if ser.in_waiting:
            initial_data = ser.read(ser.in_waiting)
            if initial_data:
                print(f"Cleared initial data: {initial_data}")
        
        print("\n===== INTERACTIVE SERIAL TERMINAL =====")
        print(f"Connected to {port} at {baudrate} baud")
        print("Type commands to send to the device")
        print("Available commands:")
        print("  'angle_a,angle_b' - Move to servo angles (e.g. '120,60')")
        print("  'xy:x,y' - Move to XY coordinates (e.g. 'xy:50,80')")
        print("  'pen:up' or 'pen:down' - Move pen up or down")
        print("  'safe:x,y,w,h' - Set safe zone parameters")
        print("  'safe:on' or 'safe:off' - Enable/disable safe zone")
        print("  'exit' - Quit this program")
        print("=====================================\n")
        
        # Main loop
        while True:
            # Get command from user
            cmd = input("\n> ").strip()
            
            if cmd.lower() == 'exit':
                break
                
            # Send command
            print(f"Sending: '{cmd}'")
            ser.write(f"{cmd}\n".encode())
            ser.flush()
            
            # Wait briefly for response
            time.sleep(0.5)
            
            # Read response
            response_data = bytearray()
            start_time = time.time()
            
            # Read for up to 3 seconds or until no more data
            while (time.time() - start_time) < 3:
                if ser.in_waiting > 0:
                    chunk = ser.read(ser.in_waiting)
                    response_data.extend(chunk)
                    # If we got data, reset the timeout to allow for more data
                    if chunk:
                        start_time = time.time()
                else:
                    # No data available, wait briefly
                    time.sleep(0.1)
                    # If we've waited at least 0.5s with no data, assume we're done
                    if time.time() - start_time > 0.5:
                        break
            
            # Process response
            if response_data:
                try:
                    response_text = response_data.decode('utf-8', errors='replace').strip()
                    print(f"Response:\n{response_text}")
                except Exception as e:
                    print(f"Raw response data: {response_data}")
            else:
                print("No response received")
        
        # Close the connection
        print("Closing connection...")
        ser.close()
        print("Connection closed")
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

def basic_test(port, baudrate=115200):
    """Run a basic test of serial communication"""
    try:
        # Open serial port
        print(f"Opening port {port} at {baudrate} baud...")
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=2
        )
        
        print("Port opened successfully!")
        
        # Try different newline types
        test_commands = [
            ("CR+LF", f"xy:0,0\r\n"),
            ("LF only", f"xy:0,0\n"),
            ("CR only", f"xy:0,0\r"),
        ]
        
        for name, cmd_bytes in test_commands:
            print(f"\nTrying {name}...")
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            
            print(f"Sending: {repr(cmd_bytes)}")
            ser.write(cmd_bytes.encode())
            ser.flush()
            
            # Wait for response
            time.sleep(1)
            
            # Check for response
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                print(f"Received {len(data)} bytes: {data}")
                try:
                    print(f"Decoded: {data.decode('utf-8')}")
                    print(f"SUCCESS! {name} works!")
                except:
                    print("Could not decode response")
            else:
                print(f"{name} - No response")
        
        # Close the connection
        print("\nClosing connection...")
        ser.close()
        print("Test complete!")
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    # Find available ports
    print("Searching for serial ports...")
    ports = find_ports()
    
    if not ports:
        print("No serial ports found!")
        port = input("Enter serial port manually: ").strip()
        if not port:
            sys.exit(1)
    else:
        print("Available ports:")
        for i, port in enumerate(ports):
            print(f"{i+1}. {port}")
            
        selection = input(f"Select port (1-{len(ports)}) or enter full port name: ").strip()
        if selection.isdigit() and 1 <= int(selection) <= len(ports):
            port = ports[int(selection)-1]
        else:
            port = selection  # Use directly entered port name
    
    # Choose test mode
    print("\nChoose test mode:")
    print("1. Basic test (tests different line endings)")
    print("2. Interactive terminal")
    mode = input("Select mode (1-2): ").strip()
    
    if mode == "2":
        send_command_interactive(port)
    else:
        basic_test(port)
