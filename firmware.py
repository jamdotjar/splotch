from servo import Servo
import math
import time

class Plotter:
    def __init__(self):
        # pen lift servo angles
        self.penZUp = 23
        self.penZDown = 40

        self.C2 = 12.7  # C/2, where C is the distance between the two servos
        self.totalDist = 117  # distance btwn origin and servos midpoint
        self.v = 57.295  # 180/pi

        # limits
        self.Xmax = 60
        self.Xmin = -60
        self.Ymax = 45
        self.Ymin = -45

        # Initialize servos on GPIO pins (change pins as needed)
        self.penservo = Servo(pin=2)
        self.xservo = Servo(pin=0)
        self.yservo = Servo(pin=16)

        self.actuatorPos = {'x': 0.0, 'y': 0.0}

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

    def servowrite(self, a, b):
        self.xservo.move(round(a))
        self.yservo.move(round(b))

    def drawLine(self, x1, y1):
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
