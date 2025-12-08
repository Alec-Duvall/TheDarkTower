import time as utime
import board
import digitalio
import usb_cdc
import struct

from Arducam import * 

import pwmio
from adafruit_motor import servo

# Servo setup (example: GP0)
pwm_x = pwmio.PWMOut(board.GP0, frequency=50)
my_servo_x = servo.Servo(pwm_x, min_pulse=500, max_pulse=2500)

pwm_y = pwmio.PWMOut(board.GP1, frequency=50)
my_servo_y = servo.Servo(pwm_y, min_pulse=500, max_pulse=2500)




ONCE = 512          # bytes per SPI burst
RESOLUTION = 0x04   #640x480 in OV2640 example table

buf = bytearray(ONCE)

# On-board LED for status
led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT
led.value = False

# GP10 Laser Dot Diode
laser = digitalio.DigitalInOut(board.GP10)
laser.direction = digitalio.Direction.OUTPUT
laser.value = False

# Use the USB "data" serial port
data_serial = usb_cdc.data


def log(msg):
    # print to the console REPL
    print("[{:.3f}] {}".format(utime.monotonic(), msg))


def set_resolution(cam, value):
    if value == 0x00: cam.OV2640_set_JPEG_size(OV2640_160x120)
    elif value == 0x01: cam.OV2640_set_JPEG_size(OV2640_176x144)
    elif value == 0x02: cam.OV2640_set_JPEG_size(OV2640_320x240)
    elif value == 0x03: cam.OV2640_set_JPEG_size(OV2640_352x288)
    elif value == 0x04: cam.OV2640_set_JPEG_size(OV2640_640x480)
    elif value == 0x05: cam.OV2640_set_JPEG_size(OV2640_800x600)
    elif value == 0x06: cam.OV2640_set_JPEG_size(OV2640_1024x768)
    elif value == 0x07: cam.OV2640_set_JPEG_size(OV2640_1280x1024)
    elif value == 0x08: cam.OV2640_set_JPEG_size(OV2640_1600x1200)
    else:              cam.OV2640_set_JPEG_size(OV2640_640x480)


def send_fifo_to_usb(cam, length):
    """
    Stream one captured JPEG frame from Arducam FIFO to USB in chunks.
    Protocol: b'IMG' + 4-byte BE length + [length bytes JPEG data]
    """
    log("Streaming {} bytes over USB".format(length))
    led.value = True

    # 1) send header + length
    data_serial.write(b"IMG")
    data_serial.write(struct.pack(">I", length))

    # 2) burst-read FIFO in ONCE-byte chunks
    cam.SPI_CS_LOW()
    cam.set_fifo_burst()

    sent = 0
    try:
        while sent < length:
            n = ONCE if (length - sent) >= ONCE else (length - sent)
            # Read n bytes from SPI into buf
            cam.spi.readinto(buf, start=0, end=n)
            # Write that chunk to USB
            data_serial.write(memoryview(buf)[:n])
            sent += n
    finally:
        cam.SPI_CS_HIGH()
        cam.clear_fifo_flag()
        led.value = False

    log("Done streaming frame")


def capture_one_to_usb(cam):
    """
    Trigger a capture on the OV2640 and stream it to USB.
    """
    # Prepare FIFO
    cam.flush_fifo()
    cam.clear_fifo_flag()
    cam.start_capture()

    t0 = utime.monotonic()
    # Wait for capture done flag (with timeout)
    while cam.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK) == 0:
        if utime.monotonic() - t0 > 5.0:
            raise RuntimeError("Timeout waiting for CAP_DONE")
        utime.sleep(0.005)

    length = cam.read_fifo_length()
    if length == 0 or length > 8_000_000:
        raise RuntimeError("Bad FIFO length: {}".format(length))

    send_fifo_to_usb(cam, length)


def init_camera():
    log("Init camera")
    cam = ArducamClass(OV2640)
    cam.Camera_Init()
    set_resolution(cam, RESOLUTION)
    utime.sleep(0.3)
    log("Camera ready")
    return cam


def read_line_from_usb():
    """
    Read a line (ending with b'\\n') from the data_serial port.
    Waits until a newline shows up.
    """
    line = bytearray()
    while True:
        if data_serial.in_waiting > 0:
            b = data_serial.read(1)
            if not b:
                continue
            if b == b'\n':
                break
            line.extend(b)
        else:
            utime.sleep(0.001)
    return bytes(line)

def nx_to_angle(nx, rotation_angle_x):
    nx = max(-1.0, min(1.0, nx))
    
    rotation_angle_x += nx*10
    rotation_angle_x = max(0, min(180, rotation_angle_x))
    return rotation_angle_x

def ny_to_angle(ny, rotation_angle_y):
    ny = max(-1.0, min(1.0, ny))
    
    rotation_angle_y -= ny*10
    rotation_angle_y = max(50, min(160, rotation_angle_y))
    return rotation_angle_y

def main():
    count = 0
    shoot = 0
    cam = init_camera()
    log("Waiting for commands on USB data port...")
    rotation_angle_x = 90.0
    rotation_angle_y = 90.0
    while True:
        cmd = read_line_from_usb()
        # Frame request
        if cmd == b"CAP":
            try:
                log("CAP command received")
                capture_one_to_usb(cam)
            except Exception as e:
                log("Capture error: {}".format(e))
        else:
            log("Unknown command: {}".format(cmd))
        
        # Servo command
        try:
            parts = cmd.decode().split(',')
            if len(parts) == 2:
                count = 0
                nx = float(parts[0])
                ny = float(parts[1]) #here just in case
                rotation_angle_x = nx_to_angle(nx, rotation_angle_x)
                rotation_angle_y = ny_to_angle(ny, rotation_angle_y)
                my_servo_x.angle = rotation_angle_x
                my_servo_y.angle = rotation_angle_y
                log(f"servo_x angle: {rotation_angle_x:.1f}")
                log(f"servo_y angle: {rotation_angle_y:.1f}")
                log(f"{nx}")
                log(f"{ny}")
                if(nx < 0.05 and ny < 0.05):
                    shoot += 1
                else:
                    laser.value = False
                    shoot = 0
                if(shoot >= 3):
                    laser.value = True
            else:
                log("No green detected.")
                laser.value = False
                count += 1
                if (count >= 20): # reinitialize servo position if no objects found every 20 frames
                    rotation_angle_x = 90
                    rotation_angle_y = 90
                    my_servo_x.angle = rotation_angle_x
                    my_servo_y.angle = rotation_angle_y
                    log(f"servo_x angle: {rotation_angle_x:.1f}")
                    log(f"servo_y angle: {rotation_angle_y:.1f}")
                    count = 0

        except Exception as e:
            log(f"servo parse error: {e}")

if __name__ == "__main__":
    main()
