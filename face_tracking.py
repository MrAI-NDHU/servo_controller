import logging
import math
import sys
import time

import cv2
import imutils
import RPi.GPIO as GPIO
from imutils.video import FPS, VideoStream

from servo import Servo
from servo.controller import ControllerForRPi

logging.basicConfig(stream=sys.stdout, level=logging.INFO)
GPIO.setwarnings(False)

# Constants
FACE_CC_PATH = \
    "/usr/local/share/OpenCV/haarcascades/haarcascade_frontalface_alt.xml"
RES_W, RES_H = 320, 240
RES_WH, RES_HH = RES_W / 2, RES_H / 2
LED_PIN, PAN_PIN, TILT_PIN = 36, 12, 33
PAN, TILT = "pan", "tilt"
RX_MIN, RY_MIN, RX_MAX, RY_MAX, RX_STEP, RY_STEP = 0, 70, 180, 110, 10, 10
ALLOW_ERR_LIMIT = 8
EXT_RATES = [0.15, 0.32, 0.49, 0.66, 0.83, 1.00]  # len=6 & [0]>0.0 & [5]=1.0
ADJUST_EXT_X, ADJUST_EXT_Y = 1.60, 1.20
YELLOW, GREEN, CYAN = (0, 255, 255), (0, 255, 0), (255, 255, 0)

# Global variables
rx, ry = (RX_MIN + RX_MAX) / 2, (RY_MIN + RY_MAX) / 2
is_tracking = False
allowed_err_time = 0
is_face_appeared = False

# Setup face cascade classifier
face_cc = cv2.CascadeClassifier(FACE_CC_PATH)

# Initialize video stream and camera
vs = VideoStream(usePiCamera=True, resolution=(RES_W, RES_H)).start()
time.sleep(2)

# Initialize LED
GPIO.setup(LED_PIN, GPIO.OUT)
GPIO.output(LED_PIN, GPIO.HIGH)

# Setup Pan-Tilt
sg90 = Servo(0.0, 180.0, 2.8, 12.8, 50.0, 0.003)
servos = {PAN: sg90, TILT: sg90}
pins = {PAN: PAN_PIN, TILT: TILT_PIN}
c = ControllerForRPi(servos, pins)
c.start([PAN, TILT])


def main():
    global rx, ry, is_tracking, c
    
    def fix_val(val, val_min, val_max, val_step):
        if val < val_min:
            return val_min
        elif val > val_max:
            return val_max
        else:
            return round(val / val_step) * val_step
    
    tx, ty = 0, 0
    try:
        while True:
            ry_end = RY_MAX if ty % 2 == 0 else RY_MIN
            while RY_MIN <= ry <= RY_MAX:
                c.move({TILT: ry}, False)
                rx_end = RX_MAX if tx % 2 == 0 else RX_MIN
                while RX_MIN <= rx <= RX_MAX:
                    c.move({PAN: rx}, False)
                    is_tracking = True
                    while is_tracking:
                        detect_face()
                    rx += RX_STEP if tx % 2 == 0 else -RX_STEP
                    rx = fix_val(rx, RX_MIN, RX_MAX, RX_STEP)
                    if rx == rx_end:
                        break
                tx += 1
                ry += RY_STEP if ty % 2 == 0 else -RY_STEP
                ry = fix_val(ry, RY_MIN, RY_MAX, RY_STEP)
                if ry == ry_end:
                    break
            ty += 1
    except KeyboardInterrupt:
        c.stop([PAN, TILT])
        del c


def detect_face():
    global is_tracking, allowed_err_time, is_face_appeared
    
    logging.info("detecting faces…")
    fps = FPS().start()
    frame = imutils.rotate(vs.read(), 180)
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_cc.detectMultiScale(
        image=frame_gray, scaleFactor=1.1, minNeighbors=5
    )
    fps.update()
    
    logging.info("detected %d face(s)", len(faces))
    i, fc_dist2_min = 0, math.pow(RES_WH, 2) + math.pow(RES_HH, 2)
    fi, fcx_min, fcy_min = 0, 0, 0
    for (fx, fy, fw, fh) in faces:
        logging.info("face #%d: x=%d y=%d w=%d h=%d", i, fx, fy, fw, fh)
        fcx, fcy = fx + fw / 2, fy + fh / 2
        if len(faces) == 1:
            fi, fcx_min, fcy_min = 0, fcx, fcy
            break
        fc_dist2 = math.pow(fcx - RES_WH, 2) + math.pow(fcy - RES_HH, 2)
        if fc_dist2 <= fc_dist2_min:
            fi, fcx_min, fcy_min, fc_dist2_min = i, fcx, fcy, fc_dist2
        i += 1
    
    i, font = 0, cv2.FONT_HERSHEY_DUPLEX
    for (fx, fy, fw, fh) in faces:
        cv2.putText(frame, "face #%d" % i, (fx, fy - 4), font, 0.4, GREEN)
        color = CYAN if i == fi else YELLOW
        cv2.rectangle(frame, (fx, fy), (fx + fw, fy + fh), color, 2)
        i += 1
    
    cv2.imshow("frame", frame)
    cv2.waitKey(1)
    logging.info("selected face #%d", fi)
    
    is_tracking = len(faces) > 0
    if is_tracking:
        GPIO.output(LED_PIN, GPIO.LOW)
        allowed_err_time = 0
        is_face_appeared = True
        adjust_pan_tilt(fcx_min - RES_WH, fcy_min - RES_HH)
    else:
        GPIO.output(LED_PIN, GPIO.HIGH)
        if is_face_appeared and allowed_err_time <= ALLOW_ERR_LIMIT:
            allowed_err_time += 1
            is_tracking = True
        else:
            allowed_err_time = 0
            is_face_appeared = False
    
    fps.stop()
    logging.info("elasped %f sec, FPS=%f", fps.elapsed(), fps.fps())


def adjust_pan_tilt(ox, oy):
    global rx, ry, c
    
    logging.info("adjusting camera to face's center, x-=%f y+=%f", ox, oy)
    
    dx = abs(ox / RES_WH)
    dy = abs(oy / RES_HH)
    if dx <= EXT_RATES[0] and dy <= EXT_RATES[0]:
        logging.info("skip adjusting camera, dx=%f dy=%f", dx, dy)
        return
    
    def get_adjust(diff, adjust_ext):
        if EXT_RATES[0] < diff <= EXT_RATES[1]:
            return adjust_ext * 0.60
        elif EXT_RATES[1] < diff <= EXT_RATES[2]:
            return adjust_ext * 0.80
        elif EXT_RATES[3] < diff <= EXT_RATES[4]:
            return adjust_ext * 1.60
        elif EXT_RATES[4] < diff <= EXT_RATES[5]:
            return adjust_ext * 2.40
        return adjust_ext
    
    adjust_x = get_adjust(dx, ADJUST_EXT_X)
    adjust_y = get_adjust(dy, ADJUST_EXT_Y)
    
    if ox > 0:
        adjust_x *= -1
    if oy < 0:
        adjust_y *= -1
    rx += adjust_x
    ry += adjust_y
    c.move({PAN: adjust_x, TILT: adjust_y}, True)


if __name__ == '__main__':
    main()
