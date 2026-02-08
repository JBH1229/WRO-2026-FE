## Simple foundation for our code

### Camera Capture & ROI Drawing
```py
import cv2
from picamera2 import Picamera2
from time import sleep

picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = &quot;RGB888&quot;
picam2.preview_configuration.controls.FrameRate = 30
picam2.preview_configuration.align()
picam2.configure(&quot;preview&quot;)
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
frame = picam2.capture_array() # continuously capture frame
draw_roi(frame, ROI1, label=&quot;ROI1 (Left)&quot;)
draw_roi(frame, ROI2, label=&quot;ROI2 (Right)&quot;)
cv2.imshow(&quot;Camera with Left/Right ROI&quot;, frame) # continuously display
if cv2.waitKey(1) &amp; 0xFF == ord(&#39;q&#39;):
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
if a &gt; area_max and a &gt;= min_contour_area:
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
draw_roi(frame, ROI1, label=&quot;ROI1 (Left)&quot;)
draw_roi(frame, ROI2, label=&quot;ROI2 (Right)&quot;)
if leftContour is not None:
cv2.drawContours(frame, [leftContour], -1, (0, 255, 0), 2)
if rightContour is not None:
cv2.drawContours(frame, [rightContour], -1, (0, 255, 0), 2)
cv2.putText(frame, f&quot;leftArea: {int(leftArea)}&quot;, (10, 30),
cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
cv2.putText(frame, f&quot;rightArea: {int(rightArea)}&quot;, (10, 60),
cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

cv2.imshow(&quot;Wall Detect (Left/Right ROI)&quot;, frame)

if cv2.waitKey(1) &amp; 0xFF == ord(&#39;q&#39;):
break
cv2.destroyAllWindows()
```
