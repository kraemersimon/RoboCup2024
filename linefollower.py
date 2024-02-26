import serial
import time
import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera

ser = serial.Serial('/dev/ttyUSB0', 115200)

LEFT_MOTOR = 0
RIGHT_MOTOR = 1

def send_motor_command(motor, direction, speed):
    print("Executing: send_motor_command(", motor, "), (", direction, "), (", speed, ")")

    speed = min(speed, 255)  # Begrenzen Sie die Geschwindigkeit auf 255
    #speed = speed // 4  # Teilen Sie die Geschwindigkeit durch 4, um sie in den Bereich von 0 bis 63 zu bringen
    command = (motor << 7) | (direction << 6) | speed
    ser.write(bytes([command]))

def m(left, right, duration):
    left = left * 4 - 1
    right = right * 4 - 1

    left_is_forward = 1
    right_is_forward = 1

    if left < 0:
        left_is_forward = 0
    if right < 0:
        right_is_forward = 0

    send_motor_command(LEFT_MOTOR, left_is_forward, abs(left))
    send_motor_command(RIGHT_MOTOR, right_is_forward, abs(left))
    if duration > 0:
        time.sleep(int(duration / 1000))

        # TODO: Arduino soll ganz stoppen bei 0 und nicht nur ausrollen
        send_motor_command(LEFT_MOTOR, 1, 0)
        send_motor_command(RIGHT_MOTOR, 1, 0)

#m(64, 64, 1000)
# send_motor_command(0, 1, 255)
# time.sleep(1)
# send_motor_command(1, 1, 255)
# time.sleep(1)
# send_motor_command(0, 1, 0)
# time.sleep(1)
# send_motor_command(1, 1, 0)
# time.sleep(1)
send_motor_command(LEFT_MOTOR, 1, 50)
time.sleep(1)
ser.close()

"""


camera = PiCamera()
camera.resolution = (320, 200)
camera.rotation = 0
rawCapture = PiRGBArray(camera, size=(320, 200))
time.sleep(0.1)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):    
    image = frame.array    
    Blackline = cv2.inRange(image, (0,0,0), (75,75,75))    
    kernel = np.ones((3,3), np.uint8)
    Blackline = cv2.erode(Blackline, kernel, iterations=2)

    contours_blk, hierarchy_blk = cv2.findContours(Blackline.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    
    contours_blk_len = len(contours_blk)
    if contours_blk_len > 0 :
        if contours_blk_len == 1 :
            blackbox = cv2.minAreaRect(contours_blk[0])
        else:
            canditates=[]
            off_bottom = 0    
            for con_num in range(contours_blk_len):        
                blackbox = cv2.minAreaRect(contours_blk[con_num])
                (x_min, y_min), (w_min, h_min), ang = blackbox        
                box = cv2.boxPoints(blackbox)
                (x_box,y_box) = box[0]
                if y_box > 198 :        
                    off_bottom += 1
                canditates.append((y_box,con_num,x_min,y_min))        
            canditates = sorted(canditates)
            if off_bottom > 1:    
                canditates_off_bottom=[]
                for con_num in range ((contours_blk_len - off_bottom), contours_blk_len):
                    (y_highest,con_highest,x_min, y_min) = canditates[con_num]        
                    total_distance = (abs(x_min - x_last)**2 + abs(y_min - y_last)**2)**0.5
                    canditates_off_bottom.append((total_distance,con_highest))
                canditates_off_bottom = sorted(canditates_off_bottom)
                (total_distance,con_highest) = canditates_off_bottom[0]
                blackbox = cv2.minAreaRect(contours_blk[con_highest])    
            else:        
                (y_highest,con_highest,x_min, y_min) = canditates[contours_blk_len-1]        
                blackbox = cv2.minAreaRect(contours_blk[con_highest])    
        (x_min, y_min), (w_min, h_min), ang = blackbox
        x_last = x_min
        y_last = y_min
        if ang < -45 :
            ang = 90 + ang
        if w_min < h_min and ang > 0:    
            ang = (90-ang)*-1
        if w_min > h_min and ang < 0:
            ang = 90 + ang    
        setpoint = 160
        error = int(x_min - setpoint)
        ang = int(ang)
        speed = 255  # Setze die Geschwindigkeit auf maximal
        send_motor_command(LEFT_MOTOR, 1 if error > 0 else 0, abs(error))  # Steuerung des linken Motors basierend auf dem Fehler
        send_motor_command(RIGHT_MOTOR, 1 if error < 0 else 0, abs(error))  # Steuerung des rechten Motors basierend auf dem Fehler

        box = cv2.boxPoints(blackbox)
        box = np.int0(box)
        cv2.drawContours(image,[box],0,(0,0,255),3)    
        cv2.putText(image,str(ang),(10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(image,str(error),(10, 320), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        cv2.line(image, (int(x_min),200 ), (int(x_min),250 ), (255,0,0),3)
    cv2.imshow("orginal with line", image)    
    rawCapture.truncate(0)    

    key = cv2.waitKey(1) & 0xFF    
    if key == ord("q"):
        print('q')
        send_motor_command(LEFT_MOTOR, 1, 0)
        send_motor_command(RIGHT_MOTOR, 1, 0)
        cv2.destroyAllWindows()
        ser.close()
        exit(0)
"""