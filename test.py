from servo import Servo
import time

xservo = Servo(6)
yservo = Servo(7)
penservo = Servo(8)

for i in range(5):
    # Minimum position
    xservo.move(0)
    yservo.move(0)
    penservo.move(0)
    time.sleep(0.5)
    # Middle position
    xservo.move(90)
    yservo.move(90)
    penservo.move(90)
    time.sleep(0.5)
    # Maximum position
    xservo.move(180)
    yservo.move(180)
    penservo.move(180)
    time.sleep(0.5)
