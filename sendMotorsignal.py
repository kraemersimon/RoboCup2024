import serial
import time

LEFT_MOTOR = 0
RIGHT_MOTOR = 1

def send_motor_command(motor, direction, speed):
    speed = min(speed, 255)  # Begrenzen Sie die Geschwindigkeit auf 255
    speed = speed // 4  # Teilen Sie die Geschwindigkeit durch 4, um sie in den Bereich von 0 bis 63 zu bringen
    command = (motor << 7) | (direction << 6) | speed
    ser.write(bytes([command]))

ser = serial.Serial('/dev/ttyUSB0', 115200)
time.sleep(5)

send_motor_command(LEFT_MOTOR, 0, 255)  # Beispiel: Setze die Geschwindigkeit des linken Motors auf 255 (maximal)
print("left motor forward")
time.sleep(0.5)

send_motor_command(LEFT_MOTOR, 1, 255)  # Beispiel: Setze die Geschwindigkeit des linken Motors auf 255 (maximal)
print("left motor backwards")
time.sleep(0.5)

send_motor_command(LEFT_MOTOR, 0, 0)  # Beispiel: Stoppe den linken Motor
time.sleep(0.5)

send_motor_command(RIGHT_MOTOR, 0, 255)  # Beispiel: Setze die Geschwindigkeit des rechten Motors auf 255 (maximal)
print("right motor forward")
time.sleep(0.5)

send_motor_command(RIGHT_MOTOR, 1, 255)  # Beispiel: Setze die Geschwindigkeit des rechten Motors auf 255 (maximal)
print("right motor backwards")
time.sleep(0.5)

send_motor_command(RIGHT_MOTOR, 0, 80)
print("right motor 80 speed")
time.sleep(0.5)

send_motor_command(LEFT_MOTOR, 1, 80)
print("left motor backwards 80 speed")
time.sleep(0.5)


send_motor_command(RIGHT_MOTOR, 0, 0)  # Beispiel: Stoppe den rechten Motor
send_motor_command(LEFT_MOTOR, 0, 0)  # Beispiel: Stoppe den rechten Motor

ser.close() #a short test, if my new git setup works
