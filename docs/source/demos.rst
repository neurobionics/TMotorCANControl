Demos
=====
These demos make use of the SoftRealtimeLoop from the `NeuroLocoMiddleware <https://pypi.org/project/NeuroLocoMiddleware/>`_
library, written by Gray Thomas. The SoftRealtimeLoop tries to ensure more accurate timing on the updates
for a smoother motor control experience. However, you could eaisly write your own simple timing loop
with the python time module.

mit_can demos
-------------

A basic read-only example is as follows::
    from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
    import time
    from TMotorCANControl.mit_can import TMotorManager_mit_can

    # CHANGE THESE TO MATCH YOUR DEVICE!
    Type = 'AK80-9'
    ID = 1

    with TMotorManager_mit_can(motor_type=Type, motor_ID=ID) as dev:
        dev.set_zero_position()
        time.sleep(1.5) # wait for the motor to zero (~1 second)
        
        print("Starting read only demo. Press ctrl+C to quit.")
        loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0.0)
        
        for t in loop:
            dev.update()
            print("\r" + str(dev), end='')

See the demos on `GitHub <https://github.com/neurobionics/TMotorCANControl/tree/master/demos/mit_can>`_ 
for examples of more control modes.
    
        
servo_can demos
---------------

A basic read-only example is as follows::
    from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
    from TMotorCANControl.servo_can import TMotorManager_servo_can
    import time

    with TMotorManager_servo_can(motor_type='AK80-9', motor_ID=0) as dev:
        
        loop = SoftRealtimeLoop(dt=0.005, report=True, fade=0.0)
        dev.enter_idle_mode()
        dev.set_zero_position()
        for t in loop:
            dev.update()
            print("\r" + str(dev),end='')

See the demos on `GitHub <https://github.com/neurobionics/TMotorCANControl/tree/master/demos/servo_can>`_ 
for examples of more control modes.

servo_serial demos
------------------

A basic read-only example is as follows::
    from TMotorCANControl.servo_serial import *
    from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop

    baud=921600
    port='/dev/ttyUSB0'
    motor_params = Servo_Params_Serial['AK80-9']

    with TMotorManager_servo_serial(port=port, baud=baud, motor_params=motor_params) as dev:
            loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0.0)
            dev.set_zero_position()
            dev.update()
            dev.enter_idle_mode()
            for t in loop:
                dev.update()
                Pdes = np.sin(t)
                print(f"\r {dev}", end='')

See the demos on `GitHub <https://github.com/neurobionics/TMotorCANControl/tree/master/demos/servo_serial>`_ 
for examples of more control modes.
