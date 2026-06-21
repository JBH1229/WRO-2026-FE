## Simple foundation for our code

### Raspberry-Arduino Communication (Arduino)
```cpp

#include <Arduino.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <Arduino_BMI270_BMM150.h> // Include Rev2 IMU Library

static const size_t LINE_MAX = 16;    // max length of command (excluding the terminate character '\0')
static char   lineBuf[LINE_MAX + 1];
static size_t lineLen = 0;
static const char SOC = '@';          // start-of-command marker
static bool   inCommand = false;

// --- IMU Variables ---
float heading = 0.0;
float gyroZBias = 0.0;
unsigned long lastTime;

void SetRGBLED(int state)//0 - Off; 1 - RED; 2 - Green; 3 - Blue
{
  if(state == 0)
  {
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, HIGH);
  }
  else if(state == 1)
  {
    digitalWrite(LEDR, LOW);// Red ON
    digitalWrite(LEDG, HIGH);// Green OFF
    digitalWrite(LEDB, HIGH);// Blue OFF
  }
  else if(state == 2)
  {
    digitalWrite(LEDR, HIGH);// Red OFF
    digitalWrite(LEDG, LOW);// Green ON
    digitalWrite(LEDB, HIGH);// Blue OFF
  }
  else if(state == 3)
  {
    digitalWrite(LEDR, HIGH);// Red OFF
    digitalWrite(LEDG, HIGH);// Green OFF
    digitalWrite(LEDB, LOW);// Blue ON
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);
  Steering_Servo.attach(3);
  lizardESC.attach(4);
  lizardESC.writeMicroseconds(1500); // 1500us is typically neutral
  delay(2000);

  // Initialize the RGB pins as outputs
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);

  // Turn all colors OFF immediately at startup
  SetRGBLED(0);

  // Initialize the Rev2 IMU
  if (!IMU.begin()) { //Failed to initialize Rev2 IMU!
    SetRGBLED(1);
    while (1); 
  }

  // Gyro Calibration: Keep the car perfectly still when booting up!
  SetRGBLED(3);
  int samples = 500;
  float sum = 0;
  int validSamples = 0;
  for (int i = 0; i < samples; i++) {
    if (IMU.gyroscopeAvailable()) {
      float x, y, z;
      IMU.readGyroscope(x, y, z);
      sum += z;
      validSamples++;
    }
    delay(10);
  }
  gyroZBias = (validSamples > 0) ? sum / validSamples : 0.0;
  
  //Calibration Complete.
  SetRGBLED(2);
  delay(1000);
  SetRGBLED(0);
  
  lastTime = micros();
}

static void handleCommand(char* command) {
  if (!command || command[0] == '\0') return;

  // If the line includes '\r' (CRLF), terminate at it
  char* cr = strchr(command, '\r');
  if (cr) *cr = '\0';

  // Trim leading spaces
  while (*command && isspace((unsigned char)*command)) command++;
  if (*command == '\0') return;

  // Command type
  char type = *command++;

  // --- Handle Heading request immediately before integer parsing ---
  if (type == 'H') {
    Serial.println(heading);
    return;
  }

  // Parse integer (For S and M commands)
  char* endp = nullptr;
  long value = strtol(command, &endp, 10);

  // Must have at least one digit for S and M commands
  if (endp == command) return;

  if (type == 'S') {
    if (value < 0 || value > 180) return;
      Steering_Servo.write(value);

  } else if (type == 'M') {
    if (value < 1000 || value > 2000) return;
      lizardESC.writeMicroseconds(value);
  }
}

void loop() {
  // 1. Read exactly ONE available byte per loop execution pass
  if (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == SOC) {
      // Start a fresh command tracking window
      inCommand = true;
      lineLen = 0;
    } 
    else if (inCommand) {
      // Only process characters if we have successfully parsed a valid SOC ('@')
      if (c == '\n') {
        lineBuf[lineLen] = '\0';
        handleCommand(lineBuf);
        inCommand = false;
        lineLen = 0;
      } 
      else if (lineLen < LINE_MAX) {
        lineBuf[lineLen++] = c;
      } 
      else {
        // Buffer Overflow protection: dump frame, reset, wait for next '@'
        inCommand = false;
        lineLen = 0;
      }
    }
  }

  // =======================================================================
  // 2. Background Time-Critical Work: IMU Tracking
  // =======================================================================
  if (IMU.gyroscopeAvailable()) {
    float x, y, z;
    IMU.readGyroscope(x, y, z);

    // Remove baseline sensor noise drift
    z -= gyroZBias;

    // Ignore micro-vibrations below a small deadzone threshold
    if (abs(z) < 0.1) z = 0.0; 

    // Compute elapsed time (dt) in seconds
    unsigned long currentTime = micros();
    float dt = (currentTime - lastTime) / 1000000.0;
    lastTime = currentTime;

    // Integrate angular velocity over time
    heading -= z * dt; 
  }
}
```
### Camera Capture & ROI Drawing
```py
import cv2
from picamera2 import Picamera2
from time import sleep

# --- Camera init ---
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate = 30
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

# --- ROIs (x1, y1, x2, y2)  ---
ROI1 = [20, 170, 240, 220]     # Left ROI
ROI2 = [400, 170, 620, 220]    # Right ROI

def draw_roi(img, roi, color=(0, 255, 255), thickness=2, label=None):
    x1, y1, x2, y2 = roi
    cv2.rectangle(img, (x1, y1), (x2, y2), color, thickness)
    if label:
        cv2.putText(img, label, (x1, max(0, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

# --- Continuously capture and display frame, with ROIs ---
while True:
    frame = picam2.capture_array()  # continuously capture frame

    # show left and right ROI in the frame displayed
    draw_roi(frame, ROI1, label="ROI1 (Left)")
    draw_roi(frame, ROI2, label="ROI2 (Right)")

    cv2.imshow("Camera with Left/Right ROI", frame)  # continuously display

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
picam2.stop()
```


### Wall Detection inside the ROI's

```py

import cv2
import numpy as np
from picamera2 import Picamera2
from time import sleep

# --- Camera init ---
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate = 30
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()
sleep(0.5)

# --- ROIs (x1, y1, x2, y2) ---
ROI1 = [20, 170, 240, 220]     # Left ROI
ROI2 = [400, 170, 620, 220]    # Right ROI

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
    roi_bgr = frame_rgb[y1:y2, x1:x2]  # picamera2 gives RGB, but OpenCV ops below work either way for Lab conversion
    lab = cv2.cvtColor(roi_bgr, cv2.COLOR_RGB2Lab)
    lab = cv2.GaussianBlur(lab, (7, 7), 0)

    mask = cv2.inRange(lab, lab_lower, lab_upper)

    # Clean up noise
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

    # Shift contour to full-frame coordinates for drawing on the original frame
    if contour_max is not None:
        contour_max = contour_max + np.array([[[x1, y1]]], dtype=np.int32)

    return area_max, contour_max, mask

# --- LAB threshold for "black wall" (tune this!) ---
# In Lab: L low = dark. Start conservative and tune by observing mask output.
LAB_BLACK_LOWER = np.array([0,   0,   0], dtype=np.uint8)
LAB_BLACK_UPPER = np.array([70, 255, 255], dtype=np.uint8)

# --- Main loop ---
while True:
    frame = picam2.capture_array()  # RGB image

    # Detect left/right wall areas
    leftArea, leftContour, leftMask = find_wall_area_lab(frame, ROI1, LAB_BLACK_LOWER, LAB_BLACK_UPPER)
    rightArea, rightContour, rightMask = find_wall_area_lab(frame, ROI2, LAB_BLACK_LOWER, LAB_BLACK_UPPER)

    # Draw ROIs
    draw_roi(frame, ROI1, label="ROI1 (Left)")
    draw_roi(frame, ROI2, label="ROI2 (Right)")

    # Draw largest contour (wall) if found
    if leftContour is not None:
        cv2.drawContours(frame, [leftContour], -1, (0, 255, 0), 2)
    if rightContour is not None:
        cv2.drawContours(frame, [rightContour], -1, (0, 255, 0), 2)

    # Show numeric areas on the frame
    cv2.putText(frame, f"leftArea: {int(leftArea)}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(frame, f"rightArea: {int(rightArea)}", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

    # Display main view
    cv2.imshow("Wall Detect (Left/Right ROI)", frame)

    # Optional: show masks for tuning thresholds (uncomment if needed)
    # cv2.imshow("Left ROI Mask", leftMask)
    # cv2.imshow("Right ROI Mask", rightMask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
picam2.stop()
```
