# SPLOTCH:.
(**S**CARA **PLOT**ter that's **CH**eap).

SPLOTCH is a parallel arm SCARA plotter, that can *usually* trick people into thinking it draws what it's meaning to.

This project was built in 72 hours as part of the Undercity Hackathon in SF, and it's state does represent that.

# Development



# Make your own!
Considering the relatively lightweight requirements, you can build your own SCARA plotter with a few servos, some 3d printed parts, and a pen. Specifically, you will need:
| Component               | Description                                      | Quantity |
|-------------------------|--------------------------------------------------|----------|
| Servos                  | G996R or similar for arm joints, and pen lifter             | 3       |
| 3D printed parts        | 4 3D printed arms, servo holder, and case                              | 1        |
| Pen                     | Any standard pen or marker, should be no more than 9mm in diameter near the head.                        | 1        |
| Microcontroller         | Raspberry Pi pico                | 1        |

| Wiring                  | Jumper wires, connectors, etc.                   | ~ |
| Sticky tape or glue | For securing the servos | ~ |
| M.2 screws | for attaching arms together | 4 |
| Baseplate | A flat-ish base to mount everything on, we used cardboard| 1 |

### Assembly
1. Print all 3D parts, or get some elsewhere.
2. put the servos in the holder, and fix them in place with double sided tape or glue.
3. Attach the holder to the baseplate, in the center of the bottom edge. The curved corners of the baseplate should be facing forwards, and the wires of the servos should be facing backwards.
4. Attach the front and back links of each arm. The m.2 screw should pass through the first arm and bore into the second arm, securing them together and allowing room for rotation. The screws on the two front arms will face opposite directions, and the screws on the two back arms will face opposite directions.
The front arm should be on the top, and the back arm should be on the bottom.
