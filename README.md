# TControl
A library in progress for controlling the AK80-9 Tmotor Actuator from CubeMars over its CAN bus.

## Notes to include later:

1. You need to calibrate the motors before you can use them (or possibly just after tampering with them), this is done over serial
2. If the serial interface is giving you OTW fault, it's a temperature fault, wait for the chip to cool off
3. Serial baud rate: 921600
4. CAN bus data rate: 1MBps
5. You seem to need at least 2 devices to start a network with the newer motors, but with the older ones you seemed to need only 1
