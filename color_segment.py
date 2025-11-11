import cv2
import numpy as np

# camera config
CAM_INDEX = 0
FRAME_W, FRAME_H = 640, 480
AREA_MIN = 500  # ignore tiny blobs

# HSV range use online picker to match object color as close as possible
LOWER = np.array([40, 90, 70])    # green-ish low
UPPER = np.array([85, 255, 255])  # green-ish high

cap = cv2.VideoCapture(CAM_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)

if not cap.isOpened():
    print("Camera not found")
    raise SystemExit

def find_largest_contour(mask, area_min=AREA_MIN):
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(c) < area_min:
        return None
    return c

def contour_centroid(contour):
    M = cv2.moments(contour)
    if M["m00"] == 0:
        return None
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    return (cx, cy)

while True:
    ok, frame = cap.read()
    if not ok:
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    
    mask = cv2.inRange(hsv, LOWER, UPPER)

    # Clean up noise a bit
    # This method uses opening and closing method talked about in class
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5,5), np.uint8))

    c = find_largest_contour(mask)
    target_found = False
    nx = ny = None  # normalized coords

    if c is not None:
        target_found = True
        # centroid
        cen = contour_centroid(c)
        if cen is not None:
            cx, cy = cen

            # draw visuals
            x,y,w,h = cv2.boundingRect(c)
            cv2.rectangle(frame, (x,y), (x+w, y+h), (0,255,0), 2)
            cv2.circle(frame, (cx, cy), 6, (0,0,255), -1)

            # normalized coords in [-1,+1], 0 = image center
            # should be easier to map to servo angles this way maybe?
            nx = (cx - FRAME_W/2) / (FRAME_W/2)
            ny = (cy - FRAME_H/2) / (FRAME_H/2)

            # crosshair at image center + text
            cv2.drawMarker(frame, (FRAME_W//2, FRAME_H//2), (255,255,255),
                           markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
            cv2.putText(frame, f"cx={cx} cy={cy}  nx={nx:.2f} ny={ny:.2f}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

    # show windows
    cv2.imshow("frame", frame)
    cv2.imshow("mask", mask)

    # print numeric values to terminal (useful for debugging/serial later)
    if target_found:
        print(f"{nx:.3f},{ny:.3f}")
    else:
        print("none")

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
