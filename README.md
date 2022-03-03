# TControl
A library in progress for controlling the AK80-9 Tmotor Actuator from CubeMars over its CAN bus.

## Notes to include later:

1. You need to calibrate the motors before you can use them (or possibly just after tampering with them), this is done over serial
2. If the serial interface is giving you OTW fault, it's a temperature fault, wait for the chip to cool off
3. Serial baud rate: 921600
4. CAN bus data rate: 1MBps
5. You need to use the 120 Ohm Termination resistor for proper communication
6. The zeroing function of the TMotor control board causes about a second of delay time before the board will send state updates again :/
