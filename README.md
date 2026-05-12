# comma body

This the firmware for the comma body robotics dev kit.


Learn more at [commabody.com](https://commabody.com/).

## building

Compile: `scons`

Flash bootstub and app: `board/recover.sh`  # STM flasher should be connected to debug port, needs openocd

Flash app through CAN bus with (standalone) panda:

`board/flash_base.sh`  # base motherboard

`board/flash_knee.sh`  # knee motherboard

Flash app through CAN bus with (comma device) panda:

1. kill openpilot or just the panda processes
2. run `board/flash_base.sh`
3. restart openpilot via `op start` or restart device