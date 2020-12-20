# RFUSB firmware

RFUSB is a USB dongle to send and receive RF packets over the air.

Firmware creates a virtual serial port over USB

## Requrements

- ARM GCC compiler for Cortex M0 controllers
- dfu-util to program the controller

## Compile and flash the firmware

```bash
make version
make
make program
```


