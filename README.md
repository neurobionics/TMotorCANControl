# TMotorCANControl
A Python API for controlling the AK-series Tmotor Actuators from CubeMars over the CAN bus.
The project is geared towards the control of the AK80-9 actuator using a raspberry pi CAN hat, but
could eaisly be adapted for use with a different CAN interface. The API files are in the src/TMotorCANControl
folder in this repository. The main interface is in the file TMotorManager.py. The CAN_Manager file contains
a low-level CAN interface for interacting with the motor, which is used by the TMotorManager class to 
control the motor in a more user-friendly way. Sample scripts can be found in the src/TMotorCANControl/demos folder.
For help setting up the motor using a Raspberry Pi 4 and with the PiCAN hat, see [these instructions](https://opensourceleg.com/TMotorCANControl/)
on the Open Source Leg website. This page will walk you through all the setup.

## API usage
For some code examples, see the src/TMotorCANControl/demos folder in this repository.
These examples make use of the soft_real_timeloop class from the [NeuroLocoMiddleware library](https://pypi.org/project/NeuroLocoMiddleware/) 
for the control loops, in order to ensure safe exiting of the loop when the program is terminated.

The intended use case would be to declare a TMotorManager object in a with block, and then
write your controller within that block, in order to ensure the motor is powered on when in use
and powered off afterwards. The TMotorManager class is in the TMotorManager module in the TMotorCANControl package.
As such, it can be imported like this:

```python
from TMotorCANControl.TMotorManager import TMotorManager
```

To instantiate a motor object, you need to specify the motor's type as a string (eg, "AK80-9"), as well
as it's CAN ID and an optional log file and set of logging parameters. Note, with multiple motors 
each log file must have a different name.  The logger will always log a time stamp for each line 
in the log, starting from the instantiation of the TMotorManager object. By default, it will also log the 
output position, output velocity, output acceleration, current, and output torque. To specify
a different log format, you can pass in a list of parameters. The full list is shown below:

```python
logvars = [
    "output_angle", 
    "output_velocity", 
    "output_acceleration",
    "current",
    "output_torque",
    "motor_angle", 
    "motor_velocity", 
    "motor_acceleration", 
    "motor_torque"
]
```

And motor control could be entered as such for an AK80-9 motor with CAN ID 3:
```python
with TMotorManager(motor_type='AK80-9', motor_ID=3, CSV_file="log.csv", log_vars=logvars) as dev:
```

The motor can be controlled in current/torque, velocity, or impedance mode. Additionally, a
current/torque, a velocity, and a position could be specified, in what we call "Full State Feedback" mode, 
which makes full use of the functionality of the controller on the TMotor driver board.
Before using any of these control modes, the mode must be entered by calling the appropriate 
function, as follows:

- set_impedance_gains_real_unit(K=stiffness,B=damping): Used to enter impedance control only mode.
- set_current_gains(): Used to enter current control only mode, for current or torque control.
- set_impedance_gains_real_unit_full_state_feedback(K=stiffness,B=damping): Used to enter full state feedback mode.
- set_speed_gains(kd=velocity proportional gain): Used to enter velocity control only mode. This uses the "damping" gain for the impedance controller as a "P" gain for velocity commands.

Once entered, the motor can be controlled in any of these modes by setting the TMotorManager's
internal command, and then calling the update() method to send the command. The values of the
internal command can be set with the following methods:

- set_output_angle_radians(pos): Sets the position command to "pos" radians.
- set_motor_current_qaxis_amps(current): Sets the current command to "current" amps.
- set_output_torque_newton_meters(torque): Sets the current command based on the torque supplied.
- set_output_velocity_radians_per_second(vel): Sets velocity command to "vel" rad/s.
- set_motor_torque_newton_meters(torque): Sets torque command based on the torque specified, adjusted for the gear ratio to control motor-side torque.
- set_motor_angle_radians(pos): Sets position command based on the position specified, adjusted for the gear ratio to control motor-side position.
- set_motor_velocity_radians_per_second(vel): Sets velocity command based on the velocity specified, adjusted for the gear ratio to control motor-side velocity.

Furthermore, the motor state can be accessed with the following methods. The state is updated
every time the update() method is called, which are pretty self explanitory.
- get_current_qaxis_amps()
- get_output_angle_radians()
- get_output_velocity_radians_per_second()
- get_output_acceleration_radians_per_second_squared()
- get_output_torque_newton_meters()
- get_motor_angle_radians()
- get_motor_velocity_radians_per_second()
- get_motor_acceleration_radians_per_second_squared()
- get_motor_torque_newton_meters()

A second interface is also provided, which will use these methods to streamline your Python
code and make it look more like math. In this interface, properties are set to call the
above getters and setters as follows:

- i: current
- θ: output angle
- θd: output velocity
- θdd: output acceleration
- τ: output torque
- ϕ: motor-side angle
- ϕd: motor-side velocity
- ϕdd: motor-side acceleration
- τm: motor-side torque

Another notable function is the zero_position() function, which sends a command to the motor to 
zero it's current angle. This function will shut off control of the motor for about a second
while the motor zeros (sort of like zeroing a scale, it seems to record a few points to get 
a good measurement). As such, after calling the method you should delay for at least a second
if timely communication is important.

The following example would instantiate a TMotorManager for an AK80-9 motor with a CAN ID of 3,
logging into a CSV file named "log.csv", with the full set of log variables specified above. Then
it will zero the motor position and wait long enough for the motor to be done zeroing. Finally,
it will enter impedance control mode with gains of 10Nm/rad and 0.5Nm/(rad/s). It then will set 
the motor position to 3.14 radians until the program is exited.

```python
with TMotorManager(motor_type='AK80-9', motor_ID=3, CSV_file="log.csv", log_vars=logvars) as dev:
    dev.zero_position()
    time.sleep(1.5)
    dev.set_impedance_gains_real_unit(K=10,B=0.5)
    loop = SoftRealtimeLoop(dt = 0.01, report=True, fade=0)

    for t in loop:
        dev.update()
        dev.θ = 3.14
```

For more examples, see the src/TMotorCANControl/demo folder. Have fun controlling some TMotors!

## Other Resources
1. [Setup Instructions on the OSL Website](https://opensourceleg.com/TMotorCANControl/)

2. [AK-series motor manual](https://store.cubemars.com/images/file/20211201/1638329381542610.pdf)
The documentation for the AK-series TMotors, which includes the CAN protocol and how to use R-Link

3. [PiCAN 2 CAN Bus Hat](https://copperhilltech.com/pican-2-can-bus-interface-for-raspberry-pi/) 
The documentation for the CopperHill Raspberry Pi CAN hat.

4. [RLink Youtube videos](https://www.youtube.com/channel/UCs-rBZ4uKBpOT9vokLZPhog/featured)
Yoyo's youtube channel has some tutorials on how to use the RLink software.

5. [Mini-Cheetah-TMotor-Python-Can](https://github.com/dfki-ric-underactuated-lab/mini-cheetah-tmotor-python-can)
This is another, more low-level library for controlling these motors that functions simillarly to
our CAN_Manager class.

For questions, please contact Mitry Anderson or Vamsi Peddinti. 
