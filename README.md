# TheDarkTower
## What is The Dark Tower?

The Dark Tower is a project inspired by Barad-dûr from The Lord of the Rings. It is a simple 3D printed base (files included) that included a mounted camera, omnidirectional servo, and laser pointer powered by a Raspberry Pi Pico. It uses some fundamental concepts related to robot vision (color segmentation, morphological opening/closing) to process a video feed sent from the RPi Pico to any computer capable of installing and running Python 3.13 and the related libraries. Currently, the tower is configured to search for greenish colors. Highly saturated greens are picked up best by the camera that was used for testing.

[A video demo is available!](https://www.youtube.com/watch?v=s9SjvUJPs3w)

## Python environment setup (Python 3.13 preferred, Python 3.14 does not work)

1) `python -m venv venv` ---> sets up environment (optional)
2) `venv\Scripts\activate` ---> enter environment for windows (optional, required with step 1)
3) `pip install -r requirements.txt` ---> install dependencies
4) `deactivate` ---> leave environment (optional, required with step 1 and 2)

## PICO environment setup

### Tutorials Referenced
- [Arducam PICO SPI Camera](https://github.com/ArduCAM/PICO_SPI_CAM)
- [OV2640 2MP Camera on Raspberry Pi Pico](https://manuals.plus/arducam/ov2640-mini-2mp-spi-camera-on-raspberry-pi-pico-manual)
1) This project uses Thonny IDE to load the RPi Pico with the firmware required to interact with color_segmentation.py. Set up the RPi Pico to use a CIRCUITPY interface when programming. If this has been done correctly, reconnect the RPi Pico and a CIRCUITPY drive should appear.
2) All code located in the PICO_DRIVERS folder should get relocated to the CIRCUITPY drive
3) Restart the RPi Pico again, boot.py should automatically establish the communication (COM) ports associated with the RPi Pico.
4) Change line 14 in color_segment.py to connect to the data COM (try both, this isn't always consistently one COM port or another!)
5) Run code.py (open code.py using Thonny IDE in the CIRCUITPY drive, click the green run button)
6) Run color_segment.py if the previous steps have all been done correctly, two camera-related windows should appear.
7) Any changes to code.py will require rerunning code.py from within the CIRCUITPY drive before changes will appear
