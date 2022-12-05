.. TMotorCANControl documentation master file, created by
   sphinx-quickstart on Sun Dec  4 20:53:31 2022.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to TMotorCANControl's documentation!
============================================

The project is geared towards the control of the AK80-9 actuator using a raspberry pi CAN hat 
or serial bus, but could eaisly be adapted for use with a different CAN/serial interface. 

It can be download from `PyPi <https://pypi.org/project/TMotorCANControl/>`_ with
the command "pip install TMotorCANControl".

The API files are on `GitHub <https://github.com/neurobionics/TMotorCANControl>`_, 
in the src/TMotorCANControl folder. The main interface is in 
the file TMotorManager_mit_can.py for MIT mode, TMotorManager_servo_can.py for Servo mode (over CAN),
and TMotorManager_servo_serial for Servo mode (over serial). Sample scripts can be 
found in the demos folder. For help setting up the motor using a Raspberry Pi 4 and with the 
PiCAN hat, see `these instructions <https://opensourceleg.com/TMotorCANControl/>`_ on the 
Open Source Leg website. That page will walk you through all the setup.

.. toctree::
   :maxdepth: 3
   :caption: Contents:

   home <self>
   modules
   demos

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
