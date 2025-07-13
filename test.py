from servo import Servo
import time

xservo = Servo(7)
yservo = Servo(6)
penservo = Servo(8)

for i in range(5):
    
    xservo.move(90)
    yservo.move(90)
    penservo.move(90)
    time.sleep(0.5)