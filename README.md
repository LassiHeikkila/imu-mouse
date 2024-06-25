# IMU-mouse

Firmware is implemented as Zephyr based application.

The board devicetree files are found under [`imu_mouse/`](./imu_mouse/).

## Status

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

## Pre-requisites for build

### nRF Connect SDK
You must have a working installation of nRF Connect SDK available on your system.

Refer to [the Nordic Semiconductor website](https://www.nordicsemi.com/Products/Development-software/nRF-Connect-SDK) and [nRF Connect SDK page](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/2.6.1/nrf/installation.html) for details on how to install it.

## Dependencies
This project uses the following dependencies:

| What | Version | For what? |
| ---- | ------- | --- |
| nRF Connect SDK | v2.6.1 | Toolchain |
