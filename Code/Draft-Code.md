## Simple foundation for our code

### Raspberry-Arduino Communication (Arduino)
```cpp

#include <Arduino.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <Servo.h>

static const size_t LINE_MAX = 16;    
static char   lineBuf[LINE_MAX + 1];
static size_t lineLen = 0;
static const char SOC = '@';          
static bool   inCommand = false;
Servo Steering_Servo;
Servo lizardESC;


void setup() {
  Serial.begin(115200);
  Serial.println("test");
  Steering_Servo.attach(3);
  lizardESC.attach(4);
  lizardESC.writeMicroseconds(1500); // 1500us is typically neutral
  delay(2000);


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

  // Parse integer
  char* endp = nullptr;
  long value = strtol(command, &endp, 10);

  // Must have at least one digit
  if (endp == command) return;

  if (type == 'S') {
    if (value < 0 || value > 180) return;
    Steering_Servo.write(value);
    Serial.print("SERVO,");
    Serial.println(value);


  } else if (type == 'M') {
    if (value < 1000 || value > 2000) return;
    lizardESC.writeMicroseconds(value);
    Serial.print("Motor,");
    Serial.println(value);
  } else {
    // Unknown command type, ignore
    return;
  }
}

void loop() {
  // Read any available bytes without blocking
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    // Start of a new command
    if (c == SOC) {
      inCommand = true;
      lineLen = 0;
      continue;
    }

    // Ignore until we see SOC
    if (!inCommand) continue;

    // End of command -> process
    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      handleCommand(lineBuf);
      inCommand = false;
      lineLen = 0;
      continue;
    }

    // Collect payload, drop frame on overflow (wait for next '@')
    if (lineLen < LINE_MAX) {
      lineBuf[lineLen++] = c;
    } else {
      // Abandon (too slow)
      inCommand = false;
      lineLen = 0;
    }
  }
}
```
### Camera Capture & ROI Drawing
```py
import cv2
from picamera2 import Picamera2
from time import sleep

picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate = 30
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

ROI1 = [20, 170, 240, 220] # Left ROI
ROI2 = [400, 170, 620, 220] # Right ROI
def draw_roi(img, roi, color=(0, 255, 255), thickness=2, label=None):
x1, y1, x2, y2 = roi
cv2.rectangle(img, (x1, y1), (x2, y2), color, thickness)
if label:
cv2.putText(img, label, (x1, max(0, y1 - 8)),
cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

while True:
frame = picam2.capture_array() 
draw_roi(frame, ROI1, label="ROI1 (Left)")
draw_roi(frame, ROI2, label="ROI2 (Right)")
cv2.imshow("Camera with Left/Right ROI", frame)
if cv2.waitKey(1) &amp; 0xFF == ord('q'):
break
cv2.destroyAllWindows()
```


### Wall Detection inside the ROI's

```py

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

#  Requiress tuning 
LAB_BLACK_LOWER = np.array([0, 0, 0], dtype=np.uint8)
LAB_BLACK_UPPER = np.array([70, 255, 255], dtype=np.uint8)

while True:
frame = picam2.capture_array() # RGB image
leftArea, leftContour, leftMask = find_wall_area_lab(frame, ROI1, LAB_BLACK_LOWER,
LAB_BLACK_UPPER)
rightArea, rightContour, rightMask = find_wall_area_lab(frame, ROI2, LAB_BLACK_LOWER,
LAB_BLACK_UPPER)
draw_roi(frame, ROI1, label="ROI1 (Left)")
draw_roi(frame, ROI2, label="ROI2 (Right)")
if leftContour is not None:
cv2.drawContours(frame, [leftContour], -1, (0, 255, 0), 2)
if rightContour is not None:
cv2.drawContours(frame, [rightContour], -1, (0, 255, 0), 2)
cv2.putText(frame, f"leftArea: {int(leftArea)}", (10, 30),
cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
cv2.putText(frame, f"rightArea: {int(rightArea)}", (10, 60),
cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

cv2.imshow("Wall Detect (Left/Right ROI)", frame)

if cv2.waitKey(1) &amp; 0xFF == ord('q'):
break
cv2.destroyAllWindows()
```
