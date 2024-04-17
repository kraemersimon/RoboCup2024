import serial
import time
import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import RPi.GPIO as GPIO 

debug = False
PIN_RST_BUTTON = 10

ser = serial.Serial('/dev/ttyUSB0', 115200)
time.sleep(2) # wait for Nano to reboot

GPIO.setmode(GPIO.BCM)     # set up BCM GPIO numbering  
GPIO.setup(PIN_RST_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

LEFT_MOTOR = 0
RIGHT_MOTOR = 1

# linefollowing paramters
BASE_SPEED = 150
SENSITIVITY = 3

def send_motor_command(motor, direction, speed):
    speed = min(speed, 255)
    speed = speed // 4
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

def button_pressed():
    return GPIO.input(PIN_RST_BUTTON)
    
camera = PiCamera()
camera.resolution = (320, 192)
camera.rotation = 0
#camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(320, 192))

start_time = time.time()
number_of_frames = 0 # frame counter for FPS
red_start_time = None
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):    
    if button_pressed():
        m(0, 0, 0)
        time.sleep(3)
        while not button_pressed():
            pass
        time.sleep(3)
        
    
    
    Greendetected = False
    original_image = frame.array
    roi = original_image[0:150, 0:320]    
    kernel = np.ones((3, 3), np.uint8)
    roi = cv2.GaussianBlur(roi, ((9, 9)), 3, 3)
   
    Blackline = cv2.inRange(roi, (0, 0, 0), (70, 70, 70))
    Greensign = cv2.inRange(roi, (0, 80, 0), (100, 200, 100))
    RedLine = cv2.inRange(roi, (0, 0, 65), (100, 100, 200))
    
    Blackline = cv2.erode(Blackline, kernel, iterations = 3)
    Blackline = cv2.dilate(Blackline, kernel, iterations = 3)

    Greensign = cv2.erode(Greensign, kernel, iterations = 3)
    Greensign = cv2.dilate(Greensign, kernel, iterations = 3)
    
    RedLine = cv2.erode(RedLine, kernel, iterations = 3)
    RedLine = cv2.dilate(RedLine, kernel, iterations = 3)
    
    contours_blk, hierarchy_blk = cv2.findContours(Blackline.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_grn, hierarchy_grn = cv2.findContours(Greensign.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_red, hierarchy_red = cv2.findContours(RedLine.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    contours_blk_len = len(contours_blk)
    contours_grn_len = len(contours_grn)
    contours_red_len = len(contours_red)
    
    if contours_blk_len > 0:
        x_blk, y_blk, w_blk, h_blk = cv2.boundingRect(contours_blk[0])
        centerx_blk = x_blk + (w_blk / 2)
        centery_blk = y_blk + (h_blk / 2)
        if contours_blk_len == 1:
            blackbox = cv2.minAreaRect(contours_blk[0])
        else:
            canditates=[]
            off_bottom = 0    
            for con_num in range(contours_blk_len):        
                blackbox = cv2.minAreaRect(contours_blk[con_num])
                (x_min, y_min), (w_min, h_min), ang = blackbox        
                box = cv2.boxPoints(blackbox)
                (x_box,y_box) = box[0]
                if y_box > 198:        
                    off_bottom += 1
                canditates.append((y_box,con_num,x_min,y_min))        
            canditates = sorted(canditates)
            if off_bottom > 1:    
                canditates_off_bottom=[]
                for con_num in range ((contours_blk_len - off_bottom), contours_blk_len):
                    (y_highest,con_highest,x_min, y_min) = canditates[con_num]        
                    total_distance = (abs(x_min - x_last)*2 + abs(y_min - y_last)*2)*0.5
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
            ang = (90 - ang) * -1
        if w_min > h_min and ang < 0:
            ang = 90 + ang    
        setpoint = 160
        error = int(x_min - setpoint)
        ang = int(ang)
        #send_motor_command(LEFT_MOTOR, 1 if error > 0 else 0, abs(error))  # Steuerung des linken Motors basierend auf dem Fehler
        #send_motor_command(RIGHT_MOTOR, 1 if error < 0 else 0, abs(error))  # Steuerung des rechten Motors basierend auf dem Fehler

        box = cv2.boxPoints(blackbox)
        box = np.int0(box)
        cv2.drawContours(roi, [box], 0, (0, 0, 255), 3)    
        cv2.putText(roi, str(error), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        number_of_frames = number_of_frames + 1
        cv2.putText(roi, str(int(number_of_frames / (time.time() - start_time))),(270, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.line(roi, (int(x_min), 200), (int(x_min), 250), (255, 0, 0), 3)

        # set motors
        m(BASE_SPEED + (SENSITIVITY * error), BASE_SPEED - (SENSITIVITY * error), 0)
    print(w_blk)    
    #if len(contours_red) > 0:
        #time.sleep(5)
            
    green_points = []
    
    if w_blk > 300:
        print("Go straight")
    elif contours_grn_len > 0:
        for i in range(contours_grn_len):
            x_grn, y_grn, w_grn, h_grn = cv2.boundingRect(contours_grn[i])
            centerx_grn = x_grn + (w_grn / 2)
            centery_grn = y_grn + (h_grn / 2)
            green_points.append(centerx_grn)
            cv2.rectangle(roi, (x_grn, y_grn), (x_grn + w_grn, y_grn + h_grn), (255, 0, 0), 2)
        
    if green_points:
        if all(point > centerx_blk for point in green_points):
            print("Right")
        elif all(point < centerx_blk for point in green_points):
            print("Left")
        elif len(green_points) > 1:
            print("Turn 180 degrees")

            
    
    if debug == True:        
        cv2.imshow("orginal_image", original_image) 
        cv2.imshow("Roi_with_blur", roi) 
        cv2.imshow("black", Blackline)       
        cv2.imshow("green", Greensign) # new window to show green dot detection
   
    rawCapture.truncate(0)    
    key = cv2.waitKey(1) & 0xFF    
    if key == ord("q"):
        m(0, 0, 0)
        cv2.destroyAllWindows()
        ser.close()
        break
