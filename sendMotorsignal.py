import serial
import time

LEFT_MOTOR = 0
RIGHT_MOTOR = 1

def send_motor_command(motor, direction, speed):
    speed = min(speed, 255)  # Begrenzen Sie die Geschwindigkeit auf 255
    speed = speed // 4  # Teilen Sie die Geschwindigkeit durch 4, um sie in den Bereich von 0 bis 63 zu bringen
    command = (motor << 7) | (direction << 6) | speed
    ser.write(bytes([command]))

def m(left, right, duration):
    left_direction = 1
    if left > 0:
        left_direction = 0
    right_direction = 1
    if right > 0:
        right_direction = 0
    send_motor_command(LEFT_MOTOR, left_direction, abs(left))
    send_motor_command(RIGHT_MOTOR, right_direction, abs(right))
    if duration > 0:
        time.sleep(duration / 1000)
        send_motor_command(LEFT_MOTOR, 1, 0)
        send_motor_command(RIGHT_MOTOR, 1, 0)


ser = serial.Serial('/dev/ttyUSB0', 115200)
time.sleep(2)
m(0, 100, 200)
time.sleep(1)
ser.close()
