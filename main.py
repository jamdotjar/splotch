from machine import Pin, PWM
import time

# TODO: push button lifts

# Setup encoder pins
pin_a = Pin(26, Pin.IN, Pin.PULL_UP)
pin_b = Pin(27, Pin.IN, Pin.PULL_UP)
pin1 = Pin(20, Pin.IN, Pin.PULL_UP)
pin2 = Pin(21, Pin.IN, Pin.PULL_UP)

press_pin_a = Pin(22, Pin.IN, Pin.PULL_UP)
press_pin_b = Pin(23, Pin.IN, Pin.PULL_UP)

# Setup servo PWM
servo_a = PWM(Pin(6))
servo_a.freq(50)
servo_b = PWM(Pin(7))
servo_b.freq(50)
pen_servo = PWM(Pin(8))
pen_servo.freq(50)

# Start at middle position
position_a = 90
position_b = 90
pen_up = True


def angle_to_duty(angle):
    # Convert angle to pulse width in microseconds (typical 500-2500 us)
    min_us = 500
    max_us = 2500
    us = min_us + (max_us - min_us) * angle // 180
    return int(us * 65535 // 20000)  # Convert to duty cycle (16-bit)

def set_servo_a(angle):
    print("angle_a: ", angle)
    servo_a.duty_u16(angle_to_duty(angle))

def set_servo_b(angle):
    print("angle_b: ", angle)
    servo_b.duty_u16(angle_to_duty(angle))

def pen():
    if pen_up:
        print("Raising pen...")
        angle = 70
    else:
        print("Lowering pen...")
        angle = 0
    pen_servo.duty_u16(angle_to_duty(angle))

pen()
set_servo_a(position_a)
set_servo_b(position_b)

last_a = pin_a.value()
last1 = pin1.value()
last_press_a = press_pin_a.value()
last_press_b = press_pin_b.value()

while True:
    current_time = time.ticks_ms()

    a = pin_a.value()
    b = pin_b.value()
    if a != last_a:
        if b != a:
            position_a = min(180, position_a + 1)  # Clockwise rotation
            print("CLOCKWISE position_a: ", position_a)
        else:
            position_a = max(0, position_a - 1)    # Counter-clockwise
            print("COUNTER-CLOCKWISE position_a: ", position_a)
        set_servo_a(position_a)
    last_a = a

    c = pin1.value()
    d = pin2.value()
    if c != last1:
        if d != c:
            position_b = min(180, position_b + 1)  # Clockwise rotation
            print("CLOCKWISE position_b: ", position_b)
        else:
            position_b = max(0, position_b - 1)    # Counter-clockwise
            print("COUNTER-CLOCKWISE position_b: ", position_b)
        set_servo_b(position_b)
    last1 = c

    press_a = press_pin_a.value()
    press_b = press_pin_b.value()

    if press_a != last_press_a or press_b != last_press_b:
        if (last_press_a == 1 and press_a == 0) or (last_press_b == 1 and press_b == 0):
            print("Encoder button pressed!")
            pen_up = not pen_up
            pen()
    last_press_a = press_pin_a.value()
    last_press_b = press_pin_b.value()
    time.sleep_ms(2)
