# SPLOTCH:.
(**S**CARA **PLOT**ter that's **CH**eap).

SPLOTCH is a parallel arm SCARA plotter, that can *usually* trick people into
thinking it draws what it's meaning to.

This project was built in 72 hours as part of the Undercity Hackathon in SF, and
it's state does represent that.

# Development
Going into this, I had literally no idea how to make a robot arm, and here I am
now, with still almost no idea how to make a robot arm.  Anyways, we made this.
We started by researching different mechanisms, we settled on this because it
didn't need a belt, and kept the arms relatively light. Servos were used for
their relative ease of use over steppers, and the fact they didn't need
dedicated driver boards which were in very short supply.  The first versions of
the arm assembly were designed to not require screws, but the relative
unreliability of PLA lead to a lot of broken parts, and thus the decisiion to
screw literally everything together. ( exept for the pen holder, which was most
efficient to do snap fit )

# Make your own!
Considering the relatively lightweight requirements, you can build your own SCARA plotter with a few servos, some 3d printed parts, and a pen. Specifically, you will need:
| Component               | Description | Quantity | Link |
|-------------------------|-------------|----------|------|
| Servos                  | G996R or similar for arm joints, and pen lifter | 3 | https://towerpro.com.tw/product/sg90-analog/ |
| 3D printed parts        | 4 3D printed arms, servo holder, and case | 1 | |
| Rotary encoders         | For manual control (etch-a-sketch mode!), we used EC11's | 2 | |
| Pen                     | Any standard pen or marker, should be no more than 9mm in diameter near the head. | 1 | | 
| Microcontroller         | Raspberry Pi pico | 1 | https://www.raspberrypi.com/products/raspberry-pi-pico/ |
| Wiring                  | Dupont Jumper wires| ~ |  |
| Sticky tape or glue | For securing the servos, double sided tape is pretty good | ~ | |
| Breadboard | wiring aid | 1| |
| M.2 screws longer than 5mm | for attaching arms together | 4 |
| Baseplate | A flat-ish base to mount everything on, we used cardboard| 1 | |

### Assembly
1. Print all 3D parts, or get some elsewhere.
2. put the servos in the holder, and fix them in place with double sided tape or
   glue.
3. Attach the holder to the baseplate, in the center of the bottom edge. The
   curved corners of the baseplate should be facing forwards, and the wires of
   the servos should be facing backwards.
4. Attach the front and back links of each arm. The m.2 screw should pass
   through the first arm and bore into the second arm, securing them together
   and allowing room for rotation. The screws on the two front arms will face
   opposite directions, and the screws on the two back arms will face opposite
   directions.
   The front arm should be on the top, and the back arm should be on the bottom.

5. Attach the third servo to the end of the arm, in the small slot.
6. Wire up the motors, and rotary encoders, referencing this wiring diagram:
<img width="602" height="459" alt="image" src="https://github.com/user-attachments/assets/63ee224e-738d-476a-a62a-4e6fc717d8ac" />

7. attach the breadboard and servo holder to the baseplate
8. Place the case over the breadboard, make sure to reconnect wires properly
9. Insert rotary encoders into the slots on the lid (maybe use glue), slide in
   the lid.
10. Done!


# Usage
1. Connect the Raspberry Pi Pico to your computer via USB.
2. Setup python enviroment by running:
```bash
pip install -r requirements.txt
```
3. Flash the firmware to the Pico by running:
```bash
mpremote connect auto fs cp firmware/main.py :
```
4. use minicom or any other serial terminal to connect to the Pico:
```bash
minicom -D /dev/tty.usbmodem1101 -b 115200
```
5. Once connected, you can send commands to the plotter, you can use the
   following commands:
   - `angle_a,angle_b` - Move to servo angles (e.g. `120,60`)
   - `xy:x,y` - Move to XY coordinates (e.g. `xy:50,80`)
   - `pen:up`, `pen:down` or `pen:dot` - Move pen up or down, or do both
     (down-up)
   - `safe:x,y,w,h` - Set safe zone (paper) parameters (e.g.
     `safe:-50,100,100,100`)
   - `safe:on` or `safe:off` - Enable or disable safe zone checks
   - `enc:on` or `enc:off` - Enable or disable encoder (manual) control")
   - `enc:step:N` - Set encoder sensitivity (N = degrees per step, e.g.
     `enc:step:0.5`
   - `exit` - Quit program
  Command can be chained together with a semicolon, e.g.
  `angle_a,angle_b;xy:x,y;pen:up`
6. To generate complex strings of commands, you can either use visualizeIK.py to
   draw your own, or use the SVG to commands script to convert an SVG file into
   a set of commands.
7. - Use ik_path_planner.py to draw your own commands:
   - Run `python3 firmware/ik_path_planner.py`
   - Click on the canvas to add a point, adding subsequent points will create a
     line between them. To raise the head, click the "Pen Up" button, and to
     lower it, click the "Pen Down" button. to create a dot at your current
     position, click the "Pen Dot" button.
   - Once you are done, click the "Generate Commands" button to generate a set
     of commands based on the points you have drawn.
   - The generated commands will be printed to the console.
   - Copy the commands to your clipboard and paste them into the serial terminal
     to see them executed.
<img width="1174" height="876" alt="image" src="https://github.com/user-attachments/assets/50720b72-b19b-4c02-b11a-b8449cd2dd66" />
<img width="628" height="316" alt="image" src="https://github.com/user-attachments/assets/979031ea-504b-43e0-8e03-55fa96d630dd" />
