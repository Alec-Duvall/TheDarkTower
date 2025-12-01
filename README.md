# TheDarkTower

## Python environment setup

first two optional setup virtual environment to avoid conflict with other projects

- python -m venv venv ---> sets up environment
- venv\Scripts\activate ---> enter environemnt for windows
- pip install -r requirements.txt ---> install dependencies
- deactivate ---> leave environment

## PICO environment setup

- tutorials I used to setup camera
  --> https://github.com/ArduCAM/PICO_SPI_CAM
  --> https://manuals.plus/arducam/ov2640-mini-2mp-spi-camera-on-raspberry-pi-pico-manual

- take all code in PICO_DRIVERS and place in CIRCUITPY drive when pico plugged in
- I used thonny IDE to boot.py and then restart and run code.py
- Edit line 14 in color_segment to connect to data COM (usually second COM in device manager)
- then run color_segment.py in terminal and hopefully camera appears
