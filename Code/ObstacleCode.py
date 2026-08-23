import cv2
import numpy as np
from picamera2 import Picamera2
from time import sleep
import time
import serial

#def log(func, *args):
    #def logging_func(*args):
        #print(f"Calling function {func}")
        #func(*args)
    #return logging_func
        

# --- Arduino init ---
arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
arduino.write_timeout = 1.0
# Most Arduinos reset when the port opens; give it time to boot
time.sleep(2.0)
# Clear anything the Arduino might have printed during boot
servo_value = 66
motor_value = 1500
#set default values
arduino.reset_input_buffer()
arduino.reset_output_buffer()
starting = True
turn_side = ""
prev_turn_side = ""
#IMU Heading Tracking
last_heading_time = time.time()
HEADING_INTERVAL = 0.1 #request heading from arduino every 1 second
imu_heading = 0.0
starting_heading = 0.0
relative_heading = 0.0
first_read = True
end_run = False
end_run_counter = 0
END_RUN_LIMIT = 100
TURN_LIMIT = 12
TOGGLE_GYRO_TURN = False
GYRO_TURN_VAL = 15
relative_turn_heading = 0.0
abs_turn_heading = 0.0
MIN_NOISE_AREA = 300
MIN_REACT_AREA = 700
MIN_PILLAR_Y = 100
MAX_PILLAR_Y = 300
RED_TARGET_CX = 140
GREEN_TARGET_CX = 500
PILLAR_EXIT_FRAMES = 8
recovery_mode = False
distance_error = 1
offbalance = 0
MAX_OFFREAD = 1000
#@log
def send_servo(value):
    global servo_value
    value = int(value)
    if value < 0:
        value = 0
    if value > 180:
        value = 180
    valueb = bytes(str(value), encoding="utf-8")
    arduino.write(b"@S%b\n" %valueb)
    servo_value = value

#@log
def send_motor(value):
    global motor_value
    value = int(value)
    if value < 1000:
        value = 1000
    if value > 2000:
        value = 2000
    valueb = bytes(str(value), encoding="utf-8")
    arduino.write(b"@M%b\n" %valueb)
    motor_value = value
#@log
def send_servo_assigned(value): 
    global servo_value
    value = int(value)
    value = 66 + value
    if value < 0:
        value = 0
    if value > 180:
        value = 180
    valueb = bytes(str(value), encoding="utf-8")
    arduino.write(b"@S%b\n" %valueb)
    servo_value = value
#@log
def send_motor_assigned(value):
    global motor_value
    value = int(value)
    value = 1500 + value
    if value < 1000:
        value = 1000
    if value > 2000:
        value = 2000
    valueb = bytes(str(value), encoding="utf-8")
    arduino.write(b"@M%b\n" %valueb)
    motor_value = value
#@log
def send_servo_relative(value): 
    global servo_value
    value = int(value)
    value = servo_value + value
    if value < 0:
        value = 0
    if value > 180:
        value = 180
    valueb = bytes(str(value), encoding="utf-8")
    arduino.write(b"@S%b\n" %valueb)
    servo_value = value
#@log
def send_motor_relative(value):
    global motor_value
    value = int(value)
    value = motor_value + value
    if value < 1000:
        value = 1000
    if value > 2000:
        value = 2000
    valueb = bytes(str(value), encoding="utf-8")
    arduino.write(b"@M%b\n" %valueb)
    motor_value = value
def get_heading():
    arduino.write(b'@H\n')
send_servo(servo_value)
send_motor(motor_value)
# --- Camera init ---
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate = 30
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()
sleep(0.5)
turn_count = 0
lap_count = 0
# --- ROIs (x1, y1, x2, y2) ---
ROI1 = [0, 200, 240, 300]     # Left ROI [0, 230, 240, 300] 
ROI2 = [400, 200, 640, 300]    # Right ROI [400, 230, 640, 300]  

def draw_roi(img, roi, color=(0, 255, 255), thickness=2, label=None):
    x1, y1, x2, y2 = roi
    cv2.rectangle(img, (x1, y1), (x2, y2), color, thickness)
    if label:
        cv2.putText(img, label, (x1, max(15, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

def find_wall_area_lab(frame_rgb, roi, lab_lower, lab_upper, min_contour_area=50):
    """
    Returns:
      area_max: area of largest contour inside ROI (0 if none)
      contour_max: contour points (shifted to full-frame coordinates) or None
      mask_roi: binary mask for ROI (for optional debug display)
    """
    x1, y1, x2, y2 = roi
    roi_rgb = frame_rgb[y1:y2, x1:x2]  # Picamera2 configured to RGB888.

    lab = cv2.cvtColor(roi_rgb, cv2.COLOR_RGB2Lab)
    lab = cv2.GaussianBlur(lab, (7, 7), 0)

    mask = cv2.inRange(lab, lab_lower, lab_upper)

    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    area_max = 0
    contour_max = None
    for c in contours:
        a = cv2.contourArea(c)
        if a > area_max and a >= min_contour_area:
            area_max = a
            contour_max = c

    if contour_max is not None:
        contour_max = contour_max + np.array([[[x1, y1]]], dtype=np.int32)

    return area_max, contour_max, mask
def best_pillar(mask):
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    contours, _= cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours == ():
        return None, None, 0, None
    best = None
    best_area = 0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        x,y,w,h = cv2.boundingRect(cnt)
        cy = y+h//2
        if area > best_area and cy > MIN_PILLAR_Y:
            best_area = area
            best = cnt
        if best is None or best_area < MIN_NOISE_AREA:
            return None, None, 0, None
        x, y, w, h = cv2.boundingRect(best)
        cx = x + w // 2
        cy = y + h // 2
        best = best + np.array([[[roiPillar[0], roiPillar[1]]]], dtype=np.int32)
        return cx, cy, best_area, best
# --- LAB threshold for "black wall" (tune this!) ---
LAB_BLACK_LOWER = np.array([0,   0,   0], dtype=np.uint8)
LAB_BLACK_UPPER = np.array([70, 180, 255], dtype=np.uint8)

    
# --- Turning-state requirements (your rules) ---
LEFT_ENTER_TURN_THRESH = 1000      # condition: leftArea OR rightArea < 550
RIGHT_ENTER_TURN_THRESH = 1000
LEFT_EXIT_GROW_THRESH  = 2500     # the side that dropped must grow > 1200
RIGHT_EXIT_GROW_THRESH  = 2500
EXIT_TIME_SEC     = 0.5      # AND at least 10 seconds must pass since entry
EXIT_TIME_THRESH = 1.5 #                                                                     NOW AT 0 DUE TO PILLAR AVOID 
MAX_TIME_SEC 	  = 10.0
TURN_LEFT_ANGLE   = 41 # change lower if nessassary  avg 41
TURN_RIGHT_ANGLE  = 81 # change higher if nessassary avg 81
RECOVERY_CORRECTION = 5 # reduces the turn rate by this much when recovering
PILLAR_MAX_TURN_RATE = 10
MAX_TURN_RATE = 40
# --- Anti-false-trigger improvement ---
# Require the enter condition to be true for N consecutive frames
ENTER_CONFIRM_FRAMES = 5

# --- Mode state ---
MODE_WALL_FOLLOW = "WALL_FOLLOW"
MODE_CORNER_TURN = "CORNER_TURN"
MODE_PILLAR_AVOID = "PILLAR_AVOID"
prev_mode = ""
mode = MODE_WALL_FOLLOW
WALL_FOLLOW_MOTOR_VALUE = 1622
CORNER_TURN_MOTOR_VALUE = 1622
PILLAR_AVOID_SPEED = 1622
turn_enter_time = None       # monotonic time when we entered corner-turning mode
turn_thresh_time = None
turn_trigger_side = None     # "left", "right", or "both"
enter_counter = 0            # consecutive-frame counter for entering turning mode
Kp = 15
Kd = -1
kp_avoid = 0.15
mode_change = True
prev_error = 0
frames_without_pillar = 0
MAX_RECOVERY_TIME = 1.5
exit_counter = 0
EXIT_COUNT_THRESH = 5
time_thresh = 0
MAX_OFFBALANCE = 3.0
def recovery(side, left_area, right_area, time, old_time):
    global active_color
    global mode
    global prev_error
    global frames_without_pillar
    global recovery_mode
    global turn_trigger_side
    global turn_enter_time
    global exit_counter
    global time_thresh
    if active_color is not None:
        mode = MODE_PILLAR_AVOID
        send_motor(PILLAR_AVOID_SPEED)
        recovery_mode = False
        return
    if left_area == 0:
        offbalance = MAX_OFFREAD
    elif right_area == 0:
        offbalance = -MAX_OFFREAD
    else:
        offbalance = round(left_area/right_area-right_area/left_area,2)
    if offbalance < 0:
        send_servo(TURN_LEFT_ANGLE + RECOVERY_CORRECTION)
        exit_thresh = left_area > LEFT_EXIT_GROW_THRESH and offbalance < MAX_OFFBALANCE
        side = "left"
    else:
        send_servo(TURN_RIGHT_ANGLE - RECOVERY_CORRECTION)
        exit_thresh = right_area > RIGHT_EXIT_GROW_THRESH and offbalance > -MAX_OFFBALANCE
        side = "right"
    if exit_thresh:
        exit_counter = exit_counter + 1
    if exit_counter > EXIT_COUNT_THRESH:
        mode = MODE_WALL_FOLLOW
        frames_without_pillar = 0
        prev_error = 0
        exit_counter = 0
        send_motor(WALL_FOLLOW_MOTOR_VALUE)
        recovery_mode = False
        return
    time_thresh = time-old_time
    time_out = time_thresh > MAX_RECOVERY_TIME
    if time_out:
        turn_enter_time = time
        mode = MODE_CORNER_TURN
        frames_without_pillar = 0
        prev_error = 0
        exit_counter = 0
        turn_trigger_side = side
        send_motor(CORNER_TURN_MOTOR_VALUE)
        recovery_mode = False
        return
    
last_time = time.monotonic()
#start moving (motor start)
send_motor(WALL_FOLLOW_MOTOR_VALUE)
try:
    while True:
        """
        if turn_side == "left":
            relative_turn_heading = round(relative_heading-(turn_count*-90),2)
            abs_turn_heading = abs(relative_turn_heading)
            #print(servo_value,motor_value, lap_count, turn_count, turn_side, imu_heading, starting_heading, relative_heading, relative_turn_heading, abs_turn_heading, end_run, end_run_counter)
        else:
            relative_turn_heading = round(relative_heading-(turn_count*90),2)
            abs_turn_heading = abs(relative_turn_heading)
            #print(servo_value,motor_value, lap_count, turn_count, turn_side, imu_heading, starting_heading, relative_heading, relative_turn_heading, abs_turn_heading, end_run, end_run_counter)
        """
        if arduino.in_waiting > 0:
            arduino.reset_input_buffer()  # Throw away unread data from Arduino
        if abs(relative_heading) > 1080 and mode == MODE_WALL_FOLLOW:
            end_run = True
        if end_run_counter >= END_RUN_LIMIT:
            send_servo(66)
            send_motor(1500)
            break 
        else:
            frame = picam2.capture_array()  # RGB image
            roiPillar = (0, 100, 640, 400)
            pillar_crop = frame[roiPillar[1]:roiPillar[1]+roiPillar[3], roiPillar[0]:roiPillar[0]+roiPillar[2]]
            hsv_frame = cv2.cvtColor(pillar_crop, cv2.COLOR_RGB2HSV)
            lower_red = np.array([115, 150, 70]) # [115, 150, 70]
            upper_red = np.array([150, 255, 255]) #[150, 255, 255]
            mask_red = cv2.inRange(hsv_frame, lower_red, upper_red)
            lower_green = np.array([30, 120, 0]) # [30, 120, 0]
            upper_green = np.array([70, 255, 255]) # [70, 255, 255]
            mask_green = cv2.inRange(hsv_frame, lower_green, upper_green)
            red_cx, red_cy, red_area, red_contour = best_pillar(mask_red)
            green_cx, green_cy, green_area, green_contour = best_pillar(mask_green)
            if red_area > green_area and red_area > MIN_REACT_AREA:
                active_color = "red"
                active_cx = red_cx
                active_cy = red_cy
            elif green_area > MIN_REACT_AREA:
                active_color = "green"
                active_cx = green_cx
                active_cy = green_cy
            else:
                active_color = None
            #print("Active Color: ", active_color,"\nRed Size: ", red_area,"\nRed CX, CY: ", [red_cx, red_cy], "\nGreen Size: ", green_area, "n\Green CX, CY: ", [green_cx, green_cy])
            #Periodically request IMU heading from arduino
            if time.time()-last_heading_time >= HEADING_INTERVAL:
                get_heading()
                last_heading_time = time.time()
                time.sleep(0.02)
                if arduino.in_waiting > 0:
                    try:
                        raw = arduino.read(arduino.in_waiting).decode('utf-8', errors='ignore')
                        for line in raw.splitlines():
                            line = line.strip()
                            if not line:
                                continue
                            try:
                                imu_heading = float(line)
                                if first_read == True:
                                    starting_heading = imu_heading
                                    time.sleep(0.02)
                                    first_read = False
                                relative_heading = round(imu_heading - starting_heading, 2)
                            except ValueError:
                                pass
                    except Exception:
                        pass
            leftArea, leftContour, leftMask = find_wall_area_lab(frame, ROI1, LAB_BLACK_LOWER, LAB_BLACK_UPPER)
            rightArea, rightContour, rightMask = find_wall_area_lab(frame, ROI2, LAB_BLACK_LOWER, LAB_BLACK_UPPER)
            now = time.monotonic()
            # -------------------------
            # MODE / STATE MACHINE
            # -------------------------
            if mode == MODE_WALL_FOLLOW:
                if active_color is not None:
                    mode = MODE_PILLAR_AVOID
                    avoid_color = active_color
                    send_motor(PILLAR_AVOID_SPEED)
                if end_run == True:
                    end_run_counter = end_run_counter + 1
                if mode != prev_mode: 
                    send_motor(WALL_FOLLOW_MOTOR_VALUE)
                low_left = leftArea < LEFT_ENTER_TURN_THRESH
                low_right = rightArea < RIGHT_ENTER_TURN_THRESH
                thresh = now - (turn_thresh_time if turn_thresh_time is not None else now)
                time_elapsed = thresh >= EXIT_TIME_THRESH
                # Debounce: require N consecutive frames below threshold
                if low_left or low_right:
                    enter_counter += 1
                else:
                    enter_counter = 0
                prev_mode = mode
                if starting:
                    time_elapsed = True
                if enter_counter >= ENTER_CONFIRM_FRAMES and time_elapsed:
                    starting = False
                    mode = MODE_CORNER_TURN
                    turn_thresh_time = None
                    turn_enter_time = now
                    enter_counter = 0  # reset for next time

                    # Remember which side triggered (or both) at the moment we commit to turning mode
                    if low_left and low_right:
                        turn_trigger_side = "both"
                    elif low_left:
                        turn_trigger_side = "left"
                    else:
                        turn_trigger_side = "right"
                else:
                    if (leftArea + rightArea) > 0:
                        error = (leftArea - rightArea) / (leftArea + rightArea)
                    else:
                        error = 0
                    current_time = now
                    dt = max(current_time - last_time, 1e-3)
                    derivative = (error - prev_error) / dt if dt > 0 else 0
                    output = (Kp * error) + (Kd * derivative)
                    correction = int(output)
                    if correction > MAX_TURN_RATE:
                        correction = MAX_TURN_RATE
                    elif correction < -MAX_TURN_RATE:
                        correction = -MAX_TURN_RATE
                    send_servo_assigned(correction)
                    prev_error = error
                    last_time = current_time
            elif mode == MODE_PILLAR_AVOID:
                if active_color == "red":
                    target_cx = RED_TARGET_CX
                    prev_red = True
                else:
                    target_cx = GREEN_TARGET_CX
                    prev_red = False
                error_avoid = active_cx - target_cx
                distance_error = round((active_cy)/200, 2)
                correction = int(kp_avoid*error_avoid)*(distance_error)
                if correction > PILLAR_MAX_TURN_RATE:
                    correction = PILLAR_MAX_TURN_RATE
                elif correction < -PILLAR_MAX_TURN_RATE:
                    correction = -PILLAR_MAX_TURN_RATE
                print(correction)
                send_servo_assigned(correction)
                if active_cy > MAX_PILLAR_Y:
                    frames_without_pillar = frames_without_pillar + 1
                elif active_color is None:
                    frames_without_pillar = frames_without_pillar + 1
                else:
                    frames_without_pillar = 0
                if frames_without_pillar >= PILLAR_EXIT_FRAMES:
                    if not recovery_mode:
                        start_time = now
                    recovery_mode = True
                    if prev_red:
                        recovery("left", leftArea, rightArea, now, start_time)
                    else:
                        recovery("right", leftArea, rightArea, now, start_time)

            else:  # MODE_CORNER_TURN
                if prev_mode != mode: 
                    send_motor(CORNER_TURN_MOTOR_VALUE)
                    mode_change = True
                    prev_error = 0
                    turn_count += 1
                    if turn_count % 4 == 0:
                        lap_count += 1

                
                elapsed = now - (turn_enter_time if turn_enter_time is not None else now)

                # Requirement (1): the side that became small must grow > 1000
                if turn_trigger_side == "left":
                    grew_ok = leftArea > LEFT_EXIT_GROW_THRESH
                elif turn_trigger_side == "right":
                    grew_ok = rightArea > RIGHT_EXIT_GROW_THRESH
                else:  # "both" or unknown
                    grew_ok = (leftArea > LEFT_EXIT_GROW_THRESH) or (rightArea > RIGHT_EXIT_GROW_THRESH)
                if leftArea == 0:
                    offbalance = MAX_OFFREAD
                elif rightArea == 0:
                    offbalance = -MAX_OFFREAD
                else:
                    offbalance = round(leftArea/rightArea-rightArea/leftArea,2)
                balance_ok = (offbalance < MAX_OFFBALANCE and offbalance > 0) or (offbalance > -MAX_OFFBALANCE and offbalance < 0)
                # Requirement (2): 5 seconds passed since entering turning state
                time_ok = elapsed >= EXIT_TIME_SEC
                time_max = elapsed >= MAX_TIME_SEC
                print(time_ok, grew_ok, balance_ok)
                prev_mode = mode
                # Exit turning mode only if BOTH requirements are true
                if TOGGLE_GYRO_TURN:
                    if abs_turn_heading <= GYRO_TURN_VAL:
                        mode = MODE_WALL_FOLLOW
                        turn_enter_time = None
                        turn_thresh_time = now
                        turn_trigger_side = None
                        enter_counter = 0
                    else:
                            if mode_change == True:
                                if turn_trigger_side == "left":
                                    turn_side = "left"
                                    prev_turn_side = "left"
                                    send_servo(TURN_LEFT_ANGLE)  # turn left
                                elif turn_trigger_side == "right":
                                    send_servo(TURN_RIGHT_ANGLE)   # turn right
                                    turn_side = "right"
                                    prev_turn_side = "right"
                                else:
                                    if prev_turn_side == "left":
                                        send_servo(TURN_LEFT_ANGLE)  # default
                                        turn_side = "left"
                                    else:
                                        send_servo(TURN_RIGHT_ANGLE)
                                        turn_side = "right"
                                    
                                mode_change = False
                else:
                    if (grew_ok and time_ok and balance_ok) or time_max:
                        mode = MODE_WALL_FOLLOW
                        turn_enter_time = None
                        turn_thresh_time = now
                        turn_trigger_side = None
                        enter_counter = 0
                    else:
                        if mode_change == True:
                            if turn_trigger_side == "left":
                                turn_side = "left"
                                prev_turn_side = "left"
                                send_servo(TURN_LEFT_ANGLE)  # turn left
                            elif turn_trigger_side == "right":
                                send_servo(TURN_RIGHT_ANGLE)   # turn right
                                turn_side = "right"
                                prev_turn_side = "right"
                            else:
                                if prev_turn_side == "left":
                                    send_servo(TURN_LEFT_ANGLE)  # default
                                    turn_side = "left"
                                else:
                                    send_servo(TURN_RIGHT_ANGLE)
                                    turn_side = "right"
                                
                            mode_change = False

            # -------------------------
            # Visualization
            # -------------------------
            draw_roi(frame, ROI1, label="ROI1 (Left)")
            draw_roi(frame, ROI2, label="ROI2 (Right)")
            xyPillar = (roiPillar[0], roiPillar[1], roiPillar[0]+roiPillar[2], roiPillar[1]+roiPillar[3])
            draw_roi(frame, xyPillar, label="Pillar Roi")
            
            if leftContour is not None:
                cv2.drawContours(frame, [leftContour], -1, (255, 255, 0), 2)
            if rightContour is not None:
                cv2.drawContours(frame, [rightContour], -1, (255, 255, 0), 2)
            if red_contour is not None:
                cv2.drawContours(frame, [red_contour], -1, (0, 0, 255), 5)
            if green_contour is not None:
                cv2.drawContours(frame, [green_contour], -1, (0, 255, 0), 5)
            cv2.putText(frame, f"leftArea: {int(leftArea)}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(frame, f"rightArea: {int(rightArea)}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

            # Mode line + extra info
            if mode == MODE_WALL_FOLLOW:
                mode_color = (0, 255, 255)  # yellow
                cv2.putText(frame, "MODE: WALL_FOLLOW", (10, 95),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, mode_color, 2, cv2.LINE_AA)

                # Show debounce status (useful for tuning)
                cv2.putText(frame, f"enter_counter: {enter_counter}/{ENTER_CONFIRM_FRAMES}", (10, 125),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, mode_color, 2, cv2.LINE_AA)
                cv2.putText(frame, f"Recovery Mode: {recovery_mode}", (10, 155),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, mode_color, 2, cv2.LINE_AA)  
            elif mode == MODE_PILLAR_AVOID:
                if active_color == "red":
                    mode_color = (0, 0, 255)
                elif active_color == "green":
                    mode_color = (0, 255, 0)
                else:
                    if avoid_color == "red":
                        mode_color = (0, 0, 255)
                    elif avoid_color == "green":
                        mode_color = (0, 255, 0)
                    else:
                        mode_color = (255, 255, 255)
                cv2.putText(frame, "MODE: PILLAR_AVOID", (10, 95),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, mode_color, 2, cv2.LINE_AA)
                cv2.putText(frame, f"enter_counter: {frames_without_pillar}/{PILLAR_EXIT_FRAMES}", (10, 125),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, mode_color, 2, cv2.LINE_AA)   
                cv2.putText(frame, f"Recovery Mode: {recovery_mode}", (10, 155),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, mode_color, 2, cv2.LINE_AA)   
                cv2.putText(frame, f"Recovery Frames: {exit_counter} / {EXIT_COUNT_THRESH} ",
                        (10, 175), cv2.FONT_HERSHEY_SIMPLEX, 0.55, mode_color, 2, cv2.LINE_AA)
                cv2.putText(frame, f"Recovery Time: {time_thresh:0.1f}s / {MAX_RECOVERY_TIME}s ",
                        (10, 195), cv2.FONT_HERSHEY_SIMPLEX, 0.55, mode_color, 2, cv2.LINE_AA)
                cv2.putText(frame, f"Distance Error: {active_cy} / 200 = {distance_error} ",
                        (10, 215), cv2.FONT_HERSHEY_SIMPLEX, 0.55, mode_color, 2, cv2.LINE_AA)
                if offbalance > 0:
                    larger_side = "left"
                elif offbalance < 0:
                    larger_side = "right"
                else:
                    larger_side = "even"
                cv2.putText(frame, f"Off Balance: {offbalance} Larger Side = {larger_side}",
                        (10, 235), cv2.FONT_HERSHEY_SIMPLEX, 0.55, mode_color, 2, cv2.LINE_AA)
                print(f"Off Balance: {offbalance} Larger Side = {larger_side}")
                        
            else:
                mode_color = (255, 0, 0)    # red
                elapsed = now - (turn_enter_time if turn_enter_time is not None else now)
                side = turn_trigger_side or "unknown"
                cv2.putText(frame, f"MODE: CORNER_TURN (side={side})", (10, 95),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, mode_color, 2, cv2.LINE_AA)
                cv2.putText(frame, f"turn time: {elapsed:0.1f}s  exit if area>{LEFT_EXIT_GROW_THRESH} AND time>={EXIT_TIME_SEC}s",
                            (10, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.55, mode_color, 2, cv2.LINE_AA)
                cv2.putText(frame, f"Recovery Mode: {recovery_mode}", (10, 145),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, mode_color, 2, cv2.LINE_AA)  
                if offbalance > 0:
                    larger_side = "left"
                elif offbalance < 0:
                    larger_side = "right"
                else:
                    larger_side = "even"
                cv2.putText(frame, f"Off Balance: {offbalance} Larger Side = {larger_side}",
                        (10, 165), cv2.FONT_HERSHEY_SIMPLEX, 0.55, mode_color, 2, cv2.LINE_AA)
                print(f"Off Balance: {offbalance} Larger Side = {larger_side}")
            cv2.imshow("Wall Detect + Mode (Left/Right ROI)", frame)

            # Optional: show masks for tuning thresholds
            # cv2.imshow("Left ROI Mask", leftMask)
            # cv2.imshow("Right ROI Mask", rightMask)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
finally:
    send_motor(1500)  # stop motor
    send_servo(66)    # center steering
    cv2.destroyAllWindows()
    picam2.stop()
    arduino.close()
