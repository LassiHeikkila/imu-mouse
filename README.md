# IMU-mouse

Firmware is implemented as Zephyr based application.

The board devicetree files are found under [`imu_mouse/`](./imu_mouse/).

# Status

Working:
- buttons
- battery level reading
- BMA456 firmware upload
- BMA456 data polling
- BLE connection to host
- Mouse click

Not working yet:
- BMA456 interrupts
- BHI360 firmware upload
- Cursor movement
