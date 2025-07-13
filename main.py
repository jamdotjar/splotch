from machine import Pin, PWM
import time

# TODO: push button lifts
# TODO: second servo

# Setup encoder pins
pin_a = Pin(26, Pin.IN, Pin.PULL_UP)
pin_b = Pin(27, Pin.IN, Pin.PULL_UP)

# Setup servo PWM
servo = PWM(Pin(7))
servo.freq(50)

position = 90  # Start at middle position

def set_servo(angle):
    # Convert angle to pulse width in microseconds (typical 500-2500 us)
    min_us = 500
    max_us = 2500
    us = min_us + (max_us - min_us) * angle // 180
    duty = int(us * 65535 // 20000)  # Convert to duty cycle (16-bit)
    servo.duty_u16(duty)

set_servo(position)

last_a = pin_a.value()

while True:
    a = pin_a.value()
    b = pin_b.value()
    if a != last_a:
        if b != a:
            position = min(180, position + 1)  # Clockwise rotation
        else:
            position = max(0, position - 1)    # Counter-clockwise
        set_servo(position)
    last_a = a
    time.sleep_ms(2)
