# MAIN ISSUE: PILLAR AVOID LITERALLY JUST WONT END IN GREEN LIGHT RECOVERY MODE BEFORE RECOVERY MODE STARTS
import sys
kickOverride = False
DebugMode = False
AutoStart = True
if len(sys.argv) > 1 and sys.argv[1] == "Debug":
	AutoStart = False
ColorBias = True
PositionRequired = True
ForceDefault = False
DOYELLOW = False
DOEND = False
start_run = True
end_run = False
kick = False
OverrideRed = True
import cv2
import numpy as np
from picamera2 import Picamera2
from time import sleep
import time
import serial
import struct
from gpiozero import Button
#def log(func, *args):
	#def logging_func(*args):
		#print(f"Calling function {func}")
		#func(*args)
	#return logging_func
		
# === Lidar Init ===
PORT = "/dev/ttyAMA0"
BAUD = 230400
PACKET_HEADER = 0x54
PACKET_LEN = 47
read_lidar = True
ser = serial.Serial(PORT, BAUD, timeout=0.1)
buffer = bytearray()
deg_0 = ""
d_0 = [0, None, 0]
deg_45 = ""
d_45 = [45, None, 0]
deg_90 = ""
d_90 = [90, None, 0]
deg_135 = ""
d_135 = [135, None, 0]
deg_180 = ""
d_180 = [180, None, 0]
unread_packets = True
packets_read = 0
max_packets = 5
# LD19 CRC-8 table (standard LDROBOT checksum, poly 0x4D reflected)
CRC_TABLE = [
	0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c,
	0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5,
	0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
	0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea,
	0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62,
	0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
	0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d,
	0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4,
	0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
	0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89,
	0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f,
	0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
	0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e,
	0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7,
	0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
	0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8,
]
def calc_crc8(data):
	crc = 0
	for b in data:
		crc = CRC_TABLE[(crc ^ b) & 0xFF]
	return crc
def find_packet_start(buffer):
	for i in range(len(buffer) - 1):
		if buffer[i] == 0x54 and (buffer[i+1] & 0xFF) == 0x2C:
			return i
	return -1
def parse_packet(packet):
	if len(packet) != PACKET_LEN:
		return None
	if calc_crc8(packet[:46]) != packet[46]:
		return None
	speed = struct.unpack_from('<H', packet, 2)[0] / 64.0
	start_angle = struct.unpack_from('<H', packet, 4)[0] / 100.0
	measurements = []
	for i in range(12):
		offset = 6 + i * 3
		dist = struct.unpack_from('<H', packet, offset)[0]
		confidence = packet[offset + 2]
		measurements.append((dist, confidence))
	end_angle = struct.unpack_from('<H', packet, 42)[0] / 100.0
	timestamp = struct.unpack_from('<H', packet, 44)[0]
	crc = packet[46]
	return {
		"speed": speed,
		"start_angle": start_angle,
		"end_angle": end_angle,
		"timestamp": timestamp,
		"crc": crc,
		"points": measurements
	}
def interpolate_angles(start, end, count):
	angle_range = (end - start + 360) % 360
	step = angle_range / (count - 1)
	return [(start + i * step) % 360 for i in range(count)]
# --- Arduino init ---
button = Button(5)
arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
arduino.write_timeout = 1.0
# Most Arduinos reset when the port opens; give it time to boot
time.sleep(2.0)
# Clear anything the Arduino might have printed during boot
servo_value = 82
default_servo_value = 82
default_motor_value = 1500
motor_value = 1500
#set default values
arduino.reset_input_buffer()
arduino.reset_output_buffer()
starting = True
turn_side = ""
prev_turn_side = ""
active_cx = 0
active_cy = 0
error_avoid = 0
correction = 0
active_color = None
#IMU Heading Tracking
last_heading_time = time.time()
heading_interval = 0.1 #request heading from arduino every 1 second
imu_heading = 0.0
starting_heading = 0.0
relative_heading = 0.0
first_read = True
end_run_count = False
end_run_counter = 0
END_RUN_LIMIT = 75
TOGGLE_GYRO_TURN = False
GYRO_TURN_VAL = 15
relative_turn_heading = 0.0
abs_turn_heading = 0.0
MIN_NOISE_AREA = 300
MIN_REACT_AREA = 700
MIN_PILLAR_Y = 100
MAX_PILLAR_Y = 250
RED_TARGET_CX = 90
GREEN_TARGET_CX = 550
PILLAR_EXIT_FRAMES = 5
recovery_mode = False
distance_error = 1
offbalance = 0
MAX_OFFREAD = 1000
highest_heading = 0
lap_direction = None # CHANGE BACK TO 'None' AFTER
lap_margin = 30
target_cx = 0
prev_red = False
outer_wall = None
inner_wall = None
pillar_location = None
LIGHT_RECOVERY = 0
NORMAL_RECOVERY = 1
HEAVY_RECOVERY = 2
recovery_type = NORMAL_RECOVERY
start_step = 1
end_step = 1
LEFT = 1
RIGHT = 2
orientation = 0
LED_OFF = 0
LED_RED = 1
LED_GREEN = 2
LED_BLUE = 3
LED_YELLOW = 4
LED_CYAN = 5
LED_MAGENTA = 6
LED_WHITE = 7
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
	#print(value)
	valueb = bytes(str(value), encoding="utf-8")
	arduino.write(b"@M%b\n" %valueb)
	motor_value = value
#@log
def send_servo_assigned(value): 
	global servo_value
	value = int(value)
	value = default_servo_value + value
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
	value = default_motor_value + value
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
def send_led(value):
	value = int(value)
	if value < 0:
		value = 0
	if value > 7:
		value = 7
	valueb = bytes(str(value), encoding="utf-8")
	arduino.write(b"@L%b\n" %valueb)
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
ROI1 = [0, 220, 240, 300]     # Left ROI [0, 230, 240, 300] 
ROI2 = [400, 220, 640, 300]    # Right ROI [400, 230, 640, 300]  
ROI3 = [260, 320, 380, 400]
middle_divisor_line = [320, 100, 320, 640]
middle_divisor = 320
left_bias = 120
left_bias_line = [left_bias, 100, left_bias, 640]
right_bias = 520
right_bias_line = [right_bias, 100, right_bias, 640]
extra_divisor = 0
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
	lab = cv2.GaussianBlur(lab, (3,3), 0)

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
def best_pillar(mask, isPink=False):
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
		if area < MIN_NOISE_AREA:
			continue
		if cy <= MIN_PILLAR_Y:
			continue
		if not isPink:
			if h/w < 1.2:
				continue
		if isPink:
			if h/w > 0.8:
				continue
		if area > best_area:
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
LAB_BLACK_LOWER2 = np.array([0, 0, 0], dtype=np.uint8)
LAB_BLACK_UPPER2 = np.array([70, 180, 255], dtype=np.uint8)
	
# --- Turning-state requirements (your rules) ---
LEFT_ENTER_TURN_THRESH = 1000      # condition: leftArea OR rightArea < 550
RIGHT_ENTER_TURN_THRESH = 1000
LEFT_EXIT_GROW_THRESH  = 2500     # the side that dropped must grow > 1200
RIGHT_EXIT_GROW_THRESH  = 2500
EXIT_TIME_SEC     = 0     # AND at least 10 seconds must pass since entry
EXIT_TIME_THRESH = 0 #                                                                     NOW AT 0 DUE TO PILLAR AVOID 
MAX_TIME_SEC 	  = 10.0
TURN_LEFT_ANGLE   = 67 # change lower if nessassary  avg 41 82-15 51
TURN_RIGHT_ANGLE  = 107 # change higher if nessassary avg 81 82+15 71
RECOVERY_CORRECTION = 5 # reduces the turn rate by this much when recovering
PILLAR_MAX_TURN_RATE = 50
MAX_TURN_RATE = 40
MAX_TURN_LEFT = default_servo_value - 25
MAX_TURN_RIGHT = default_servo_value + 25
# --- Anti-false-trigger improvement ---
# Require the enter condition to be true for N consecutive frames
ENTER_CONFIRM_FRAMES = 5

# --- Mode state ---
MODE_WALL_FOLLOW = "WALL_FOLLOW"
MODE_CORNER_TURN = "CORNER_TURN"
MODE_PILLAR_AVOID = "PILLAR_AVOID"
prev_mode = ""
mode = MODE_WALL_FOLLOW
if not DebugMode:
	WALL_FOLLOW_MOTOR_VALUE = 1622
	CORNER_TURN_MOTOR_VALUE = 1622
	PILLAR_AVOID_SPEED = 1622
	PARKING_SPEED_FORWARD = 1605
	PARKING_SPEED_BACKWARD = 1395
else:
	WALL_FOLLOW_MOTOR_VALUE = 1500
	CORNER_TURN_MOTOR_VALUE = 1500
	PILLAR_AVOID_SPEED = 1500
	PARKING_SPEED_FORWARD = 1500
	PARKING_SPEED_BACKWARD = 1500
turn_enter_time = None       # monotonic time when we entered corner-turning mode
turn_thresh_time = None
turn_trigger_side = None     # "left", "right", or "both"
enter_counter = 0            # consecutive-frame counter for entering turning mode
Kp = 20 #30
Kd = -1.5 #-2.5
kp_avoid = 0.15
mode_change = True
prev_error = 0
frames_without_pillar = 0
MAX_RECOVERY_TIME = 3
MIN_RECOVERY_TIME = 1.5
exit_counter = 0
EXIT_COUNT_THRESH = 5
time_thresh = 0
MAX_OFFBALANCE = 2
EXTRA_RECOVERY_TIME = 2
DISTANCE_ERROR_DIVISOR = 200
time_out = False
time_good = False
side_ok = False
new_pillar = True
from_turn = False
exit_thresh = False
middle_seen = False
recovery_override = False
regain_control = True
go_around = False
left_turn = TURN_LEFT_ANGLE + RECOVERY_CORRECTION
right_turn = TURN_RIGHT_ANGLE - RECOVERY_CORRECTION
def recovery(side, left_area, right_area, middle_area, time, old_time, last_x):
	global active_color
	global middle_seen
	global mode
	global prev_error
	global frames_without_pillar
	global recovery_mode
	global turn_trigger_side
	global turn_enter_time
	global exit_counter
	global time_thresh
	global left_turn
	global right_turn
	global from_turn
	global active_color
	global turn_thresh_time
	global offbalance
	global recovery_type
	global prev_red
	global time_out
	global time_good
	global side_ok
	global exit_thresh
	global recovery_override
	global go_around
	global regain_control
	time_thresh = time-old_time
	left_low = left_area < LEFT_ENTER_TURN_THRESH
	right_low = right_area < RIGHT_ENTER_TURN_THRESH
	left_gone = left_area <= 0
	right_gone = left_area <= 0
	middle_high = middle_area > 5000
	if (active_color is not None) and (not ((recovery_type == LIGHT_RECOVERY) and (active_cy > 250))) and (not ((active_color == "green") and (active_cy > 320))):
		print("exit recovery mode to pillar avoid")
		recovery_mode = False
		recovery_override = False
		frames_without_pillar = 0
		prev_error = 0
		exit_counter = 0
		time_thresh = 0
		new_pillar = True
		mode = MODE_PILLAR_AVOID
		send_motor(PILLAR_AVOID_SPEED)
		return
	if left_area == 0:
		offbalance = MAX_OFFREAD
	elif right_area == 0:
		offbalance = -MAX_OFFREAD
	else:
		offbalance = round(left_area/right_area-right_area/left_area,2)
	if (lap_direction is None) or (pillar_location is None):
		time_good = time_thresh > MIN_RECOVERY_TIME
		time_out = time_thresh > MAX_RECOVERY_TIME
		left_turn = TURN_LEFT_ANGLE + RECOVERY_CORRECTION + last_x*0.001
		right_turn = TURN_RIGHT_ANGLE - RECOVERY_CORRECTION + last_x*0.001
	else:
		if recovery_type == NORMAL_RECOVERY:
			# normal recovery steps
			past_pillar = True
			if lap_direction == "CW" and prev_red:
				leeway = 1
			else:
				leeway = 0
			time_good = time_thresh > (MIN_RECOVERY_TIME-leeway)
			time_out = time_thresh > MAX_RECOVERY_TIME
			left_turn = TURN_LEFT_ANGLE + (RECOVERY_CORRECTION + last_x*0.001)
			right_turn = TURN_RIGHT_ANGLE - (RECOVERY_CORRECTION + last_x*0.001)
		if recovery_type == LIGHT_RECOVERY:
			# light recovery steps
			past_pillar = active_color == None
			if lap_direction == "CW" and prev_red:
				leeway = 1
				turn_addition = 10
			else:
				leeway = 0
				turn_addition = 0
			time_good = time_thresh > (MIN_RECOVERY_TIME-leeway)
			time_out = time_thresh > (MAX_RECOVERY_TIME-MIN_RECOVERY_TIME)
			left_turn = TURN_LEFT_ANGLE
			right_turn = TURN_RIGHT_ANGLE
		if recovery_type == HEAVY_RECOVERY: 
			# heavy recovery steps
			past_pillar = True
			if lap_direction == "CW" and not prev_red:
				leeway = 1.5
			else:
				leeway = 0
			time_passed = time_thresh > 1.5
			time_good = time_thresh > (MIN_RECOVERY_TIME+EXTRA_RECOVERY_TIME-leeway)
			time_out = time_thresh > (MAX_RECOVERY_TIME+EXTRA_RECOVERY_TIME)
			left_turn = TURN_LEFT_ANGLE - RECOVERY_CORRECTION 
			right_turn = TURN_RIGHT_ANGLE + RECOVERY_CORRECTION*2
	if side == "left":
		if recovery_type == LIGHT_RECOVERY:
			if not past_pillar:
				send_servo(right_turn-10+turn_addition)
		elif recovery_type == LIGHT_RECOVERY:
			send_servo(left_turn+10-turn_addition)
		else:
			send_servo(left_turn)
		exit_thresh = left_area > LEFT_EXIT_GROW_THRESH and offbalance < MAX_OFFBALANCE
		side_ok = right_low or left_low
		if recovery_type == LIGHT_RECOVERY:
			side_ok = right_low
	else:
		if recovery_type == LIGHT_RECOVERY:
			if not past_pillar:
				send_servo(left_turn+10-turn_addition)
		elif recovery_type == LIGHT_RECOVERY:
			send_servo(right_turn-10-turn_addition)
		else:
			send_servo(right_turn)
		exit_thresh = right_area > RIGHT_EXIT_GROW_THRESH and offbalance > -MAX_OFFBALANCE
		side_ok = left_low or right_low
		if recovery_type == LIGHT_RECOVERY:
			side_ok = left_low
			"""
	if recovery_type == HEAVY_RECOVERY:
		if middle_high:
			middle_seen = True
		if time_passed and not middle_seen:
			recovery_override = True
		if middle_seen:
			regain_control = False
		if recovery_override and middle_seen and regain_control == False:
			if prev_red:
				send_servo(left_turn)
			else:
				send_servo(right_turn)
			go_around = True
		if not middle_high:
			regain_control = True
			go_around = False
			"""
	if left_gone:
		side_ok = True
		left_low = True
		time_good = True
	if right_gone:
		side_ok = True
		right_low = True
		time_good = True
	if exit_thresh and time_good:
		exit_counter = exit_counter + 1
	else:
		exit_counter = 0
	if exit_counter > EXIT_COUNT_THRESH:
		mode = MODE_WALL_FOLLOW
		frames_without_pillar = 0
		prev_error = 0
		exit_counter = 0
		turn_thresh_time = time
		time_thresh = 0
		from_turn = False
		send_motor(WALL_FOLLOW_MOTOR_VALUE)
		recovery_override = False
		recovery_mode = False
		return
	if (time_out or side_ok) and time_good:
		turn_enter_time = time
		mode = MODE_CORNER_TURN
		frames_without_pillar = 0
		prev_error = 0
		exit_counter = 0
		time_thresh = 0
		from_turn = True
		if left_low:
			turn_trigger_side = "left"
		elif right_low:
			turn_trigger_side = "right"
		else:
			turn_trigger_side = side
		send_motor(CORNER_TURN_MOTOR_VALUE)
		recovery_override = False
		recovery_mode = False
		return
	
last_time = time.monotonic()
#start moving (motor start)
send_led(LED_RED)
sleep(1)
send_led(LED_GREEN)
print("waiting for button press...")
button.wait_for_press()
send_led(LED_WHITE)
sleep(1)
send_led(LED_GREEN)
try:
	while True:
		if active_color == "green":
			print("cx=", active_cx,
			"error=", error_avoid,
			"correction=", correction,
			"recovery=", recovery_mode)
		kick = button.is_pressed
		if not kickOverride:
			if kick:
				send_led(LED_RED)
				send_motor(default_motor_value)  # stop motor
				send_servo(default_servo_value)    # center steering
				sleep(0.5)
				send_led(LED_OFF)
				cv2.destroyAllWindows()
				picam2.stop()
				arduino.close()
				break
		#print(servo_value,motor_value, lap_count, turn_count, turn_side, imu_heading, starting_heading, relative_heading, end_run, end_run_counter)
		if arduino.in_waiting > 0:
			arduino.reset_input_buffer()  # Throw away unread data from Arduino
		if ser.in_waiting > 0:
			ser.reset_input_buffer()
		if DOEND:
			if lap_direction is None:
				if abs(relative_heading) > 1040: #1040
					end_run_count = True
				if end_run_count == True:
					end_run_counter += 1
				if end_run_counter >= END_RUN_LIMIT and abs(relative_heading) > 1060: #1060
					end_run = True
			if lap_direction == "CW":
				if abs(relative_heading) > 1060: #1040
					end_run_count = True
				if end_run_count == True:
					end_run_counter += 1
				if end_run_counter >= END_RUN_LIMIT and abs(relative_heading) > 1080: #1060
					end_run = True
			if lap_direction == "CCW":
				if abs(relative_heading) > 1040: #1040
					end_run_count = True
				if end_run_count == True:
					end_run_counter += 1
				if end_run_counter >= END_RUN_LIMIT and abs(relative_heading) > 1060: #1060
					end_run = True
		else:
			if lap_direction is None:
				if abs(relative_heading) > 1040: #1040
					end_run_count = True
				if end_run_count == True:
					end_run_counter += 1
				if end_run_counter >= END_RUN_LIMIT and abs(relative_heading) > 1070: #1060
					send_servo(82)
					send_motor(1500)
					break
			if lap_direction == "CW":
				if abs(relative_heading) > 1070: #1040
					end_run_count = True
				if end_run_count == True:
					end_run_counter += 1
				if end_run_counter >= END_RUN_LIMIT and abs(relative_heading) > 1090: #1080
					send_servo(82)
					send_motor(1500)
					break
			if lap_direction == "CCW":
				if abs(relative_heading) > 1070: #1040
					end_run_count = True
				if end_run_count == True:
					end_run_counter += 1
				if end_run_counter >= END_RUN_LIMIT and abs(relative_heading) > 1090: #1060
					send_servo(82)
					send_motor(1500)
					break
		frame = picam2.capture_array()  # RGB image
		roiPillar = (0, 100, 640, 400)
		pillar_crop = frame[roiPillar[1]:roiPillar[1]+roiPillar[3], roiPillar[0]:roiPillar[0]+roiPillar[2]]
		hsv_frame = cv2.cvtColor(pillar_crop, cv2.COLOR_RGB2HSV) # CHANGE BACK TO RGB2HSV IF NESSCESARY 
		lower_red = np.array([115, 120, 60]) # [115, 150, 70] # lab [0, 150, 60]
		upper_red = np.array([130, 255, 255]) #[150, 255, 255] # lab [60, 180, 80] (might want to change to [130, 255, 255])
		mask_red = cv2.inRange(hsv_frame, lower_red, upper_red)
		lower_green = np.array([30, 100, 0]) # [30, 120, 0] # lab [70, 85, 150]
		upper_green = np.array([70, 255, 255]) # [70, 255, 255] # lab [100, 110 185]
		mask_green = cv2.inRange(hsv_frame, lower_green, upper_green)
		lower_pink = np.array([135, 150, 70]) #[130, 150, 70] lab [60, 160, 60]
		upper_pink = np.array([150, 255, 255])#[150, 255, 255] lab [90, 180, 90]
		mask_pink = cv2.inRange(hsv_frame, lower_pink, upper_pink)
		lower_yellow = np.array([75, 100, 100]) #[75, 100, 100] lab [125, 80, 110]
		upper_yellow = np.array([95, 255, 255])#[95, 255, 255] lab [170, 100, 130]
		mask_yellow = cv2.inRange(hsv_frame, lower_yellow, upper_yellow)
		red_cx, red_cy, red_area, red_contour = best_pillar(mask_red)
		green_cx, green_cy, green_area, green_contour = best_pillar(mask_green)
		yellow_cx, yellow_cy, yellow_area, yellow_contour = best_pillar(mask_yellow)
		pink_cx, pink_cy, pink_area, pink_contour = best_pillar(mask_pink, isPink=True)
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
		if DOYELLOW:
			if yellow_area and yellow_area > MIN_REACT_AREA and active_color is not None:
				active_cx = yellow_cx
				active_cy = yellow_cy
		#print(active_cy)
		unread_packets = ser.in_waiting > 0
		if read_lidar:
			while unread_packets:
				data = ser.read(256)
				unread_packets = ser.in_waiting > 0
				packets_read += 1
				if data:
					buffer += data
					while True:
						idx = find_packet_start(buffer)
						idx = find_packet_start(buffer)
						if idx == -1 or len(buffer) - idx < PACKET_LEN:
							break
						packet = buffer[idx:idx+PACKET_LEN]
						buffer = buffer[idx+PACKET_LEN:]
						parsed = parse_packet(packet)
						#print(f"data: {data}")
						#print(f"buffer: {buffer}")
						#print(f"packet: {packet}")
						#print(f"parsed: {parsed}")
						if parsed:
							angles = interpolate_angles(parsed["start_angle"], parsed["end_angle"], 12)
							#print(f"\nSpeed: {parsed['speed']:.2f} RPM | Timestamp: {parsed['timestamp']} ms")
							for i, ((dist, conf), angle) in enumerate(zip(parsed["points"], angles)):
								if conf > 0:
									if abs(angle - 0.0) < 0.3: 
										d_0 = [0, dist, conf]
										deg_0 = f"  Pt {i+1:02d}: {angle:.2f}  {dist} mm  (conf: {conf})"
									if abs(angle - (360-45.0)) < 0.3:
										d_45 = [45, dist, conf]
										deg_45 = f"  Pt {i+1:02d}: {angle:.2f}  {dist} mm  (conf: {conf})"
									if abs(angle - (360-90.0)) < 0.3: 
										d_90 = [90, dist, conf]
										deg_90 = f"  Pt {i+1:02d}: {angle:.2f}  {dist} mm  (conf: {conf})"
									if abs(angle - (360-135.0)) < 0.3:
										d_135 = [135, dist, conf]
										deg_135 = f"  Pt {i+1:02d}: {angle:.2f}  {dist} mm  (conf: {conf})"
									if abs(angle - (360-180.0)) < 0.3:
										d_180 = [180, dist, conf]
										deg_180 = f"  Pt {i+1:02d}: {angle:.2f}  {dist} mm  (conf: {conf})"
							
						else:
							print("Invalid packet")
		#print(d_0, "\n", d_45, "\n", d_90, "\n", d_135, "\n", d_180, "\n")
		#print("Active Color: ", active_color,"\nRed Size: ", red_area,"\nRed CX, CY: ", [red_cx, red_cy], "\nGreen Size: ", green_area, "n\Green CX, CY: ", [green_cx, green_cy])ing from arduino
		if time.time()-last_heading_time >= heading_interval:
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
							#print(f"direct: {relative_heading}")
						except ValueError:
							pass
				except Exception:
					pass
		leftArea, leftContour, leftMask = find_wall_area_lab(frame, ROI1, LAB_BLACK_LOWER, LAB_BLACK_UPPER)
		rightArea, rightContour, rightMask = find_wall_area_lab(frame, ROI2, LAB_BLACK_LOWER, LAB_BLACK_UPPER)
		MArea, MContour, MMask = find_wall_area_lab(frame, ROI3, LAB_BLACK_LOWER2, LAB_BLACK_UPPER2)
		if abs(relative_heading) > abs(highest_heading):
			highest_heading = relative_heading
		if lap_direction is None:
			if highest_heading > lap_margin:
				lap_direction = "CW"
				outer_wall = "left"
				inner_wall = "right"
			elif highest_heading < -lap_margin:
				lap_direction = "CCW"
				outer_wall = "right"
				inner_wall = "left"
		now = time.monotonic()
		if start_run or end_run:
			heading_interval = 0.1
			if start_run:
				print(relative_heading)
				#print(d_0, d_45, d_90, d_135, d_180)
				if orientation == 0:
					read_lidar = True
					if (d_0[1] is None) or (d_180[1] is None):
						continue
					#print(d_0[1], d_180[1], d_0[2], d_180[2])
					if d_0[1] > d_180[1]:
						orientation = LEFT
						#180 degrees is closer robot is facing left (CW)
					else:
						orientation = RIGHT
						#0 degrees is closer robot is facing right (CCW)
					read_lidar = False
				if orientation == RIGHT:
					if start_step == 1:
						send_motor(default_motor_value)
						send_servo(MAX_TURN_LEFT-10)
						send_motor(PARKING_SPEED_FORWARD)
						print("step 1")
						start_step = 2
						continue
					elif start_step == 2:
						if relative_heading > -70:
							print("step 2 loop")
							continue
						else:
							send_motor(default_motor_value)
							print("step 2 end")
							start_step = 3
							continue
					elif start_step == 3:
						send_servo(MAX_TURN_RIGHT+5)
						send_motor(PARKING_SPEED_FORWARD)
						print("step 3")
						start_step = 4
						continue
					elif start_step == 4:
						if relative_heading < -65:
							print("step 4 loop")
							continue
						else:
							send_motor(default_motor_value)
							print("step 4 end")
							start_step = 5
							continue
					elif start_step == 5:
						send_servo(MAX_TURN_LEFT)
						send_motor(PARKING_SPEED_BACKWARD)
						print("step 5")
						start_step = 6
						continue
					elif start_step == 6:
						if relative_heading < -50:
							print("step 6 loop")
							continue
						else:
							send_motor(default_motor_value)
							print("step 6 end")
							start_step = 7
							continue
					elif start_step == 7:
						send_servo(MAX_TURN_RIGHT+5)
						send_motor(PARKING_SPEED_FORWARD)
						start_step = 8
						continue
					elif start_step == 8:
						if relative_heading < -40:
							continue
						else:
							send_motor(default_motor_value)
							start_step = 9
							continue
					elif start_step == 9:
						send_servo(MAX_TURN_LEFT)
						send_motor(PARKING_SPEED_BACKWARD)
						start_step = 10
					elif start_step == 10:
						if relative_heading < -25:
							continue
						else:
							send_motor(default_motor_value)
							start_step = 11
							continue
					elif start_step == 11:
						send_servo(MAX_TURN_RIGHT+5)
						send_motor(PARKING_SPEED_FORWARD)
						start_step = 12
						continue
					elif start_step == 12:
						if relative_heading < -15:
							continue
						else:
							send_motor(default_motor_value)
							start_step = 13
							continue
					elif start_step == 13:
						send_servo(MAX_TURN_LEFT)
						send_motor(PARKING_SPEED_BACKWARD)
						start_step = 14
					elif start_step == 14:
						if relative_heading < 0:
							continue
						else:
							send_motor(default_motor_value)
							start_step = 15
							continue
					elif start_step == 15:
						send_servo(90)
						send_motor(1380)
						sleep(1)
						send_motor(default_motor_value)
						start_step = 16
						continue
					elif start_step == 16:
						start_run = False
						print("step 9")
						continue
				else:
					if start_step == 1:
						send_motor(default_motor_value)
						send_servo(MAX_TURN_RIGHT+20)
						send_motor(PARKING_SPEED_FORWARD)
						print("step 1")
						start_step = 2
						continue
					elif start_step == 2:
						if relative_heading < 70:
							print("step 2 loop")
							continue
						else:
							send_motor(default_motor_value)
							print("step 2 end")
							start_step = 3
							continue
					elif start_step == 3:
						send_servo(MAX_TURN_LEFT)
						send_motor(PARKING_SPEED_FORWARD)
						print("step 3")
						start_step = 4
						continue
					elif start_step == 4:
						if relative_heading > 55:
							print("step 4 loop")
							continue
						else:
							send_motor(default_motor_value)
							print("step 4 end")
							start_step = 5
							continue
					elif start_step == 5:
						send_servo(MAX_TURN_RIGHT+5)
						send_motor(PARKING_SPEED_BACKWARD)
						print("step 5")
						start_step = 6
						continue
					elif start_step == 6:
						if relative_heading > 50:
							print("step 6 loop")
							continue
						else:
							send_motor(default_motor_value)
							print("step 6 end")
							start_step = 7
							continue
					elif start_step == 7:
						send_servo(MAX_TURN_LEFT)
						send_motor(PARKING_SPEED_FORWARD)
						start_step = 8
						continue
					elif start_step == 8:
						if relative_heading > 35:
							continue
						else:
							send_motor(default_motor_value)
							start_step = 9
							continue
					elif start_step == 9:
						send_servo(MAX_TURN_RIGHT+5)
						send_motor(PARKING_SPEED_BACKWARD)
						start_step = 10
					elif start_step == 10:
						if relative_heading > 25:
							continue
						else:
							send_motor(default_motor_value)
							start_step = 11
							continue
					elif start_step == 11:
						send_servo(MAX_TURN_LEFT)
						send_motor(PARKING_SPEED_FORWARD)
						start_step = 12
						continue
					elif start_step == 12:
						if relative_heading > 10:
							continue
						else:
							send_motor(default_motor_value)
							start_step = 13
							continue
					elif start_step == 13:
						send_servo(MAX_TURN_RIGHT+5)
						send_motor(PARKING_SPEED_BACKWARD)
						start_step = 14
					elif start_step == 14:
						if relative_heading > 0:
							continue
						else:
							send_motor(default_motor_value)
							start_step = 15
							continue
					elif start_step == 15:
						send_servo(82)
						send_motor(1380)
						sleep(0.5)
						send_motor(default_motor_value)
						start_step = 16
						continue
					elif start_step == 16:
						start_run = False
						continue
			else:
				print(d_0, d_45)
				kp_end = 0.25
				read_lidar = True
				if lap_direction == "CW":
					if d_180[1] == None or d_135[1] == None:
						print("i cant see!")
						continue
					if end_step == 1:
						target_end = 500
						wall_follow_exit_counter = 0
						Pink_seen = False
						send_motor(1392)
						end_step = 2
						continue
					if end_step == 2:
						error_end = -(d_180[1] - target_end)
						error_trig = d_135[1] - d_180[1]*1.414
						end_correction = ((kp_end*error_end)+(kp_end*error_trig)/1.414)/2
						#print(f"{d_0[1]}, {d_45[1]}, {error_end}, {error_trig}, {kp_end*error_end}+{kp_end*error_trig/2}/2={end_correction}")
						print(pink_area, pink_cy, Pink_seen)
						if end_correction > 0:
							end_correction = min(end_correction, 40)
						else:
							end_correction = max(end_correction, -40)
						send_servo_assigned(end_correction)
						if wall_follow_exit_counter > 10:
							send_motor(1500)
							end_step = 3
							continue
						if pink_area > 0:
							wall_follow_exit_counter = wall_follow_exit_counter + 1
						continue
					if end_step == 3:
						send_servo(82)
						send_motor(1622)
						end_step = 4
						continue
					if end_step == 4:
						if pink_cy == None:
							continue
						else:
							sleep(2)
							send_motor(1500)
							end_step = 5
							continue
					if end_step == 5:
						send_servo(50)
						send_motor(1620)
						sleep(3.5)
						send_motor(1500)
						send_servo(82)
						break
						
				else:
					if d_0[1] == None or d_45[1] == None:
						print("i cant see!")
						continue
					if end_step == 1:
						target_end = 350
						wall_follow_exit_counter = 0
						Pink_seen = False
						send_motor(1620)
						end_step = 2
						continue
					if end_step == 2:
						error_end = d_0[1] - target_end
						error_trig = d_45[1] - d_0[1]*1.414
						end_correction = ((kp_end*error_end)+(kp_end*error_trig)/1.414)/2
						#print(f"{d_0[1]}, {d_45[1]}, {error_end}, {error_trig}, {kp_end*error_end}+{kp_end*error_trig/2}/2={end_correction}")
						print(pink_area, pink_cy, Pink_seen)
						if end_correction > 0:
							end_correction = min(end_correction, 40)
						else:
							end_correction = max(end_correction, -40)
						send_servo_assigned(end_correction)
						if wall_follow_exit_counter > 15:
							send_motor(1500)
							end_step = 3
							continue
						if pink_area > 0:
							Pink_seen = True
						if Pink_seen:
							if pink_cy == None:
								wall_follow_exit_counter = wall_follow_exit_counter + 1
								continue
						continue
					if end_step == 3:
						send_servo(82)
						send_motor(1390)
						end_step = 4
						continue
					if end_step == 4:
						if pink_cy == None:
							continue
						else:
							sleep(1)
							send_motor(1500)
							end_step = 5
							continue
					if end_step == 5:
						send_servo(115)
						send_motor(1620)
						sleep(3.5)
						send_motor(1500)
						send_servo(82)
						break
				#parallel parking code
		else:
			read_lidar = False
			heading_interval = 0.1
			# -------------------------
			# MODE / STATE MACHINE
			# -------------------------
			if mode == MODE_WALL_FOLLOW:
				if active_color is not None:
					mode = MODE_PILLAR_AVOID
					new_pillar = True
					avoid_color = active_color
					from_turn = False
					send_motor(PILLAR_AVOID_SPEED)
				if mode != prev_mode: 
					send_motor(WALL_FOLLOW_MOTOR_VALUE)
				low_left = leftArea < LEFT_ENTER_TURN_THRESH
				low_right = rightArea < RIGHT_ENTER_TURN_THRESH
				thresh = now - (turn_thresh_time if turn_thresh_time is not None else now)
				time_elapsed = thresh >= EXIT_TIME_THRESH
				# Debounce: require N consecutive frames below threshold
				if lap_direction == "CW":
					if low_right:
						enter_counter += 1
					else:
						enter_counter = 0
				elif lap_direction == "CCW":
					if low_left:
						enter_counter += 1
					else:
						enter_counter = 0
				else:
					if low_left or low_right:
						enter_counter += 1
					else:
						enter_counter = 0
				prev_mode = mode
				if starting:
					time_elapsed = True
					new_pillar = True
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
				left_low = leftArea < LEFT_ENTER_TURN_THRESH
				right_low = rightArea < RIGHT_ENTER_TURN_THRESH
				side_ok = left_low or right_low
				if ((recovery_mode) and (active_color is not None)) and (not ((recovery_type == LIGHT_RECOVERY) and (active_cy > 250))) and (not ((active_color == "green") and (active_cy > 320))):
					print("exit recovery mode into pillar avoid via the pillar avoid mode bypass")
					recovery_mode = False
					new_pillar = True
					frames_without_pillar = 0
					exit_counter = 0
					prev_error = 0
					send_motor(PILLAR_AVOID_SPEED)
				if ((active_color == "green") and prev_red) or ((active_color == "red") and not prev_red):
					mode = MODE_PILLAR_AVOID
					new_pillar = True
					avoid_color = active_color
					from_turn = False
					send_motor(PILLAR_AVOID_SPEED)
				if leftArea == 0:
					offbalance = MAX_OFFREAD
				elif rightArea == 0:
					offbalance = -MAX_OFFREAD
				else:
					offbalance = round(leftArea/rightArea-rightArea/leftArea,2)
				if active_color == "red":
					target_cx = RED_TARGET_CX
					prev_red = True
				elif active_color == "green":
					target_cx = GREEN_TARGET_CX
					prev_red = False
				if new_pillar:
					if active_color is not None:
						if ColorBias:
							if lap_direction is None:
								if active_cx < middle_divisor:
									pillar_location = "left"
								else:
									pillar_location = "right"	
							else:
								if (lap_direction == "CW") and prev_red:
									if PositionRequired:
										if active_cx < left_bias:
											pillar_location = "left"
										else:
											pillar_location = "right"	
									else:
										pillar_location = "left"
								if (lap_direction == "CW") and not prev_red:
									if PositionRequired:
										if active_cx < right_bias:
											pillar_location = "left"
										else:
											pillar_location = "right"	
									else:
										pillar_location = "right"
								if (lap_direction == "CCW") and prev_red:
									if PositionRequired:
										if active_cx < left_bias:
											pillar_location = "left"
										else:
											pillar_location = "right"	
									else:
										pillar_location = "right"
								if (lap_direction == "CCW") and not prev_red:
									if PositionRequired:
										if active_cx < right_bias:
											pillar_location = "left"
										else:
											pillar_location = "right"	
									else:
										pillar_location = "left"
						else:
							if active_cx < middle_divisor:
								pillar_location = "left"
							else:
								pillar_location = "right"
					new_pillar = False
					if not (((recovery_type == LIGHT_RECOVERY) and (active_cy > 250))) and (not ((active_color == "green") and (active_cy > 320))):
						print("changing recovery type")
						if ((lap_direction is None) or (pillar_location is None)) or ForceDefault:
							# DEFAULT = NORMAL RECOVERY
							recovery_type = NORMAL_RECOVERY
						else:
							if (lap_direction == "CW") and prev_red and (pillar_location == "left"):
								# CW RED LEFT = NORMAL RECOVERY
								recovery_type = NORMAL_RECOVERY
							if (lap_direction == "CW") and prev_red and (pillar_location == "right"):
								# CW RED RIGHT = LIGHT RECOVERY
								recovery_type = LIGHT_RECOVERY
							if (lap_direction == "CW") and not prev_red and (pillar_location == "left"):
								# CW GREEN LEFT = HEAVY RECOVERY
								recovery_type = HEAVY_RECOVERY
							if (lap_direction == "CW") and not prev_red and (pillar_location == "right"):
								# CW GREEN RIGHT = NORMAL RECOVERY
								recovery_type = NORMAL_RECOVERY
							if (lap_direction == "CCW") and prev_red and (pillar_location == "left"):
								# CCW RED LEFT = NORMAL RECOVERY
								recovery_type = NORMAL_RECOVERY
							if (lap_direction == "CCW") and prev_red and (pillar_location == "right"):
								# CCW RED RIGHT = HEAVY RECOVERY
								recovery_type = HEAVY_RECOVERY
							if (lap_direction == "CCW") and not prev_red and (pillar_location == "left"):
								# CCW GREEN LEFT = LIGHT RECOVERY
								recovery_type = LIGHT_RECOVERY
							if (lap_direction == "CCW") and not prev_red and (pillar_location == "right"):
								# CCW GREEN RIGHT = NORMAL RECOVERY
								recovery_type = NORMAL_RECOVERY
				if recovery_type == NORMAL_RECOVERY:
					# normal recovery steps
					distance_error = round(active_cy/200, 2)
					target_cx = target_cx
				if recovery_type == LIGHT_RECOVERY:
					# light recovery steps
					distance_error = round(active_cy/150, 2)
					if prev_red:
						target_cx = target_cx + round(50*(300/active_cy), 2)
					else:
						target_cx = target_cx - round(100*(300/active_cy), 2)
				if recovery_type == HEAVY_RECOVERY:
					# heavy recovery steps
					# MAKE TARGET_CX DEPENDANT ON ACTIVE_CY
					distance_error = round(active_cy/250, 2)
					target_cx = target_cx
				error_avoid = active_cx - target_cx
				if active_color == "green":
					if error_avoid < 0:
						# Green pillar is left of target.
						# Initial response: weaker
						correction = int(kp_avoid * 0.5 * error_avoid) * distance_error
					else:
						# Green pillar is right of target.
						# Strong response to bring it back.
						correction = int(kp_avoid * 4.0 * error_avoid) * distance_error
				elif active_color == "red":
					if error_avoid > 0 and not OverrideRed:
						# Red pillar is right of target.
						# Initial response: weaker
						correction = int(kp_avoid * 0.6 * error_avoid) * distance_error
					else:
						# Red pillar is left of target.
						# Strong response to bring it back.
						correction = int(kp_avoid * error_avoid) * distance_error
				if correction > PILLAR_MAX_TURN_RATE:
					correction = PILLAR_MAX_TURN_RATE
				elif correction < -PILLAR_MAX_TURN_RATE:
					correction = -PILLAR_MAX_TURN_RATE
				#print(correction)
				if not recovery_mode:
					send_servo_assigned(correction)
				if recovery_type == LIGHT_RECOVERY and active_cy > MAX_PILLAR_Y:
					if not recovery_mode:
						start_time = now
						recovery_mode = True
				elif active_cy > MAX_PILLAR_Y or active_color is None:
					frames_without_pillar += 1
				else:
					frames_without_pillar = max(0, frames_without_pillar - 1)
				if frames_without_pillar >= PILLAR_EXIT_FRAMES and not recovery_mode:
					start_time = now
					recovery_mode = True
				if recovery_mode:
					if prev_red:
						recovery("left", leftArea, rightArea, MArea, now, start_time, error_avoid)
					else:
						recovery("right", leftArea, rightArea, MArea, now, start_time, error_avoid)

			else:  # MODE_CORNER_TURN
				if (active_color is not None):
						mode = MODE_PILLAR_AVOID
						avoid_color = active_color
						send_motor(PILLAR_AVOID_SPEED)
						from_turn = True
				if prev_mode != mode: 
					send_motor(CORNER_TURN_MOTOR_VALUE)
					mode_change = True
					new_pillar = True
					prev_error = 0
					turn_count += 1
					if turn_count % 4 == 0:
						lap_count += 1

				
				elapsed = now - (turn_enter_time if turn_enter_time is not None else now)
				leftgone = leftArea <= 0
				rightgone = rightArea <= 0
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
				#print(time_ok, grew_ok, balance_ok)
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
						if leftgone:
							turn_side = "left"
							prev_turn_side = "left"
							send_servo(TURN_LEFT_ANGLE)  # turn left
						if rightgone:
							send_servo(TURN_RIGHT_ANGLE)   # turn right
							turn_side = "right"
							prev_turn_side = "right"
								
							mode_change = False

			# -------------------------
			# Visualization
			# -------------------------
			if not AutoStart:
				draw_roi(frame, ROI1, label="ROI1 (Left)")
				draw_roi(frame, ROI2, label="ROI2 (Right)")
				draw_roi(frame, ROI3)
				if MContour is not None:
					cv2.drawContours(frame, [MContour], -1, (255, 255, 0), 2)
				xyPillar = (roiPillar[0], roiPillar[1], roiPillar[0]+roiPillar[2], roiPillar[1]+roiPillar[3])
				draw_roi(frame, xyPillar, label="Pillar Roi")
				if not ColorBias:
					draw_roi(frame, middle_divisor_line, label="Middle")
				else:
					if lap_direction is None:
						draw_roi(frame, middle_divisor_line, label="Middle")
					else:
						draw_roi(frame, left_bias_line,color=(0,0,255), label="Left Bias")
						draw_roi(frame, right_bias_line,color=(0,255,0), label="Right Bias")
				if leftContour is not None:
					cv2.drawContours(frame, [leftContour], -1, (255, 255, 0), 2)
				if rightContour is not None:
					cv2.drawContours(frame, [rightContour], -1, (255, 255, 0), 2)
				if red_contour is not None:
					cv2.drawContours(frame, [red_contour], -1, (0, 0, 255), 5)
				if green_contour is not None:
					cv2.drawContours(frame, [green_contour], -1, (0, 255, 0), 5)
				if yellow_contour is not None:
					cv2.drawContours(frame, [yellow_contour], -1, (0, 255, 255), 5)
				if pink_contour is not None:
					cv2.drawContours(frame,[pink_contour], -1, (255, 100, 255), 5)
				cv2.putText(frame, f"leftArea: {int(leftArea)}", (10, 30),
							cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
				cv2.putText(frame, f"rightArea: {int(rightArea)}", (10, 55),
							cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
				cv2.putText(frame, f"MiddleArea: {int(MArea)}", (10, 80),
							cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
				cv2.putText(frame, f"Relative IMU Heading: {relative_heading}", (10, 420),
							cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
				cv2.putText(frame, f"Highest IMU Heading: {highest_heading}", (10, 440),
							cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
				cv2.putText(frame, f"Lap Direction: {lap_direction}", (10, 460),
							cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
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
					cx_line = [active_cx-5, 100, active_cx+5, 500]
					cy_line = [0, active_cy, 640, active_cy]
					draw_roi(frame, cx_line,color=(255,255,255)) 
					draw_roi(frame, cy_line,color=(255,0,255))
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
					if recovery_mode:
						recov_color = (0, 255, 255)
					else:
						recov_color = mode_color
					cv2.putText(frame, f"Recovery Mode: {recovery_mode}", (10, 155),
						cv2.FONT_HERSHEY_SIMPLEX, 0.55, recov_color, 2, cv2.LINE_AA)   
					cv2.putText(frame, f"Recovery Frames: {exit_counter} / {EXIT_COUNT_THRESH} ",
							(10, 175), cv2.FONT_HERSHEY_SIMPLEX, 0.55, mode_color, 2, cv2.LINE_AA)
					cv2.putText(frame, f"Recovery Time: {time_thresh:0.1f}s {time_out} {time_good} {side_ok}",
							(10, 195), cv2.FONT_HERSHEY_SIMPLEX, 0.55, mode_color, 2, cv2.LINE_AA)
					cv2.putText(frame, f"Distance Error: {distance_error} Target CX: {target_cx} Active CY: {active_cy} Active CX: {active_cx}",
							(10, 215), cv2.FONT_HERSHEY_SIMPLEX, 0.55, mode_color, 2, cv2.LINE_AA)
					if offbalance > 0:
						larger_side = "left"
					elif offbalance < 0:
						larger_side = "right"
					else:
						larger_side = "even"
					cv2.putText(frame, f"Off Balance: {offbalance} Larger Side = {larger_side}",
							(10, 235), cv2.FONT_HERSHEY_SIMPLEX, 0.55, mode_color, 2, cv2.LINE_AA)
					cv2.putText(frame, f"left_turn: {left_turn} right_turn = {right_turn} from_turn: {from_turn}",
							(10, 255), cv2.FONT_HERSHEY_SIMPLEX, 0.55, mode_color, 2, cv2.LINE_AA)
					if recovery_type == 0:
						recov_type = "LIGHT"
					if recovery_type == 1:
						recov_type = "NORMAL"
					if recovery_type == 2:
						recov_type = "HEAVY"
					cv2.putText(frame, f"Pillar Location: {pillar_location}, Recovery Type: {recov_type}",
							(10, 275), cv2.FONT_HERSHEY_SIMPLEX, 0.55, mode_color, 2, cv2.LINE_AA)
					#print(f"Off Balance: {offbalance} Larger Side = {larger_side}")
							
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
					cv2.putText(frame, f"left_turn: {left_turn} right_turn = {right_turn}",
							(10, 185), cv2.FONT_HERSHEY_SIMPLEX, 0.55, mode_color, 2, cv2.LINE_AA)
					#print(f"Off Balance: {offbalance} Larger Side = {larger_side}")
					
				cv2.imshow("Wall Detect + Mode (Left/Right ROI)", frame)
			if cv2.waitKey(1) & 0xFF == ord('q'):
				break
finally:
	send_motor(default_motor_value)  # stop motor
	send_servo(default_servo_value)    # center steering
	send_led(LED_OFF)
	cv2.destroyAllWindows()
	picam2.stop()
	arduino.close()
