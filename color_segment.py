import cv2
import numpy as np
import serial
import time
import struct

# camera config
CAM_INDEX = 0
FRAME_W, FRAME_H = 640, 480
AREA_MIN = 500  # ignore tiny blobs

# USB communication config check readme for instr of how to choose COMx value
# Also 115200 is the baud rate for UART but probably shouldnt matter since camera is SPI
ser = serial.Serial('COM6', 115200, timeout=2) #change COM3 to whatever pico shows up as in device manager
time.sleep(2) # give the pico time to reset

# HSV range use online picker to match object color as close as possible
LOWER = np.array([40, 90, 70])    # green-ish low
UPPER = np.array([85, 255, 255])  # green-ish high

# uncomment for webcam interface
# cap = cv2.VideoCapture(CAM_INDEX)
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)

# if not cap.isOpened():
#     print("Camera not found")
#     raise SystemExit

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

def read_exact(n):
    buf = b''
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if not chunk:
            raise RuntimeError("Serial read timeout")
        buf += chunk
    return buf

# on pico this means wait for the command CAP
# capture an image from camera
# return frame
def get_frame_from_pico():
    # request capture
    ser.write(b"CAP\n")
    ser.flush()

    # read header 
    header = read_exact(3)
    if header != b"IMG":
        raise RuntimeError(f"Bad header from Pico: {header}")

    # read 4-byte length
    length_bytes = read_exact(4)
    (length,) = struct.unpack('>I', length_bytes)

    # read image payload
    img_bytes = read_exact(length)

    # decode image to BGR OpenCV frame
    np_arr = np.frombuffer(img_bytes, np.uint8)
    frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    if frame is None:
        raise RuntimeError("Failed to decode image from Pico")

    return frame


while True:
    # uncomment to use computer webcam
    # ok, frame = cap.read()
    # if not ok:
    #     break

    frame = get_frame_from_pico()


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

            # create packet and send to pico, packet is CSV style,
            # could make json or some other structure if that helps
            # inverted nx and ny because camera mirrored
            packet = f"{-nx},{-ny}\n"
            ser.write(packet.encode('utf-8'))

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

# cap.release()
ser.close()
cv2.destroyAllWindows()
