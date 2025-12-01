
import time as utime
import board
import digitalio
import usb_cdc
import struct

from Arducam import * 

ONCE = 512          # bytes per SPI burst
RESOLUTION = 0x04   #640x480 in OV2640 example table

buf = bytearray(ONCE)

# On-board LED for status
led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT
led.value = False

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


def main():
    cam = init_camera()
    log("Waiting for commands on USB data port...")

    while True:
        cmd = read_line_from_usb()
        if cmd == b"CAP":
            try:
                log("CAP command received")
                capture_one_to_usb(cam)
            except Exception as e:
                log("Capture error: {}".format(e))
        else:
            log("Unknown command: {}".format(cmd))


if __name__ == "__main__":
    main()

