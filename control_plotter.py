"""
Control the SCARA plotter directly from the visualizer
This script provides a simple way to control the physical plotter
using the IK visualizer.
"""

from visualizeIK import IKVisualizer
import time

def main():
    # Create the visualizer
    visualizer = IKVisualizer()
    
    # Ask for serial port
    default_port = '/dev/tty.usbmodem1101'
    port = input(f"Enter serial port (default: {default_port}): ").strip()
    if not port:
        port = default_port
    
    # Connect to the microcontroller
    if not visualizer.connect_to_mcu(port):
        print("Failed to connect to microcontroller. Exiting.")
        return
    
    # Start the interactive visualization
    try:
        print("Starting interactive plotter control")
        print("Click on the plot to set target positions")
        print("Use the buttons to control the plotter")
        visualizer.interactive_test()
    finally:
        # Ensure we disconnect when done
        visualizer.disconnect_from_mcu()

if __name__ == "__main__":
    main()
