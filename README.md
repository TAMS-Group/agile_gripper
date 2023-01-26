# diana_gripper: Versatile 2-finger 4-servo gripper driven by Feetech SCS digital bus servos

This ROS package provides robot-models and software for a versatile
two-finger gripper using four digital-bus servos from Feetech Inc.

* Using four rotational joints, the gripper can execute parallel
  pinch grasps and tip-pinch grasps, but the fingers can also wrap
  around an object (e.g. a bottle) if needed.

* The fingertips can be easily exchanged. We provide URDF models
  for generic fixed fingers, finray-style flexible fingers,
  and adapters for mounting the camera-based high-res tactile
  DIGIT sensors.

* Both fingers are mounted on a pair of standard load-cells,
  providing measurements of grasping force and shearing force.

* Our Arduino firmware also supports multiple optional 
  finger-mounted IMUs, which can be used to measure vibration,
  impact, and fingertip orientation.

* The SCS-20 servo already provides a lot of sensor inputs,
  including joint (finger) position, velocity, motor torque,
  motor voltage and current, and motor temperature.

## Hardware Overview

## Electronics Overview 

The suggested electronics is based on an Arduino Nano 33 IOT
microcontroller, which includes another IMU (STM LSM6DS3).
The first serial interface (pins RX/TX) of the Nano 33 IOT
is connected to a Feetech "TTLinker Mini" board,
which in turn controls the SCS-20 servos.

Both GY-521 boards are connected to the first I2C-bus (pins A4/A5)
and are powered from the 5V/VUSB pin of the Arduino,
which requires to bridge the corresponding jumper on the
underside of the Nano 33 IOT. 
The GY-521 includes its own 5V->3V3 regulator and data 
communication uses 3V3 levels, compatible with the Nano 33 IOT.
Allocated I2C addresses are 0x68 and 0x69 for the GY-521 boards,
and 0x6A for the on-board LSM6DS3.

The load-cells of both fingers are connected to a common
HX-711 chip breakout board each. The HX-711 provides all
required amplification and filtering circuits together with
a 24-bit ADC running at roughly 80 Hz sample rate.
The digital interface only requires two lines (CLK, Data) each,
connected to the Arduino pins (D2,D3), (D4,D5), (D6,D7), and (D8,D9).

Communication with the host is via USB, and the whole electronics
is bus-powered by the USB. 
Of course, the SCS-20 servos require its own power supply (e.g. 7.2V, 8A).
For now, the 3V3 supply from the Arduino is used to power the load-cells,
as the HX-711 is specified for 2.7..5.5V. Using a stabilitzed
5V supply might increase signal-to-noise ratio. but the 5V USB
is too noisy for that, and another power supply would add to the
hardware complexity.

## udev device detection

The arduino transmits a (unique?) id that can be matched in
udev to identify the serial device. doc/10-diana-gripper-udev.rules
provides a rule file that can be copied to /etc/udev/rules.d/ to add a symlink
/dev/diana_gripper pointing to the correct device.

## Arduino Nano 33 IOT I2C Problems

Maybe try an active bus transceiver or at least a bus terminator, e.g.
https://learn.adafruit.com/adafruit-ltc4311-i2c-extender-active-terminator

See ~/.arduino15/packages/arduino/hardware/samd/1.8.11/cores/arduino/wiring_private.c
and below for the pin assignments and pin capabilities of your specific board.
Then try to edit the source for the Wire library a bit:
/homeL/hendrich/.arduino15/packages/arduino/hardware/samd/1.8.11/libraries/Wire

```
...
void TwoWire::begin(void) {
  //Master Mode
  sercom->initMasterWIRE(TWI_CLOCK);
  sercom->enableWIRE();

  pinPeripheral(_uc_pinSDA, g_APinDescription[_uc_pinSDA].ulPinType);
  pinPeripheral(_uc_pinSCL, g_APinDescription[_uc_pinSCL].ulPinType);
  // FNH! I2C always dying on Arduino Nano 33 IOT... see cores/arduino/wiring_private.c
   PORT->Group[g_APinDescription[_uc_pinSDA].ulPort].PINCFG[g_APinDescription[_uc_pinSDA].ulPin].reg |= (uint8_t) (PORT_PINCFG_DRVSTR);
   PORT->Group[g_APinDescription[_uc_pinSCL].ulPort].PINCFG[g_APinDescription[_uc_pinSCL].ulPin].reg |= (uint8_t) (PORT_PINCFG_DRVSTR);
   // FNH end 
}
...
```



## Servo-Commander firmware

The SCS commander utility scans the bus for devices and reports
all detected servos, together with their protocol type.
Once the initial scan is completed, the commander enters an
interactive loop where the user can select the active servo
and then execute read/write commands on the selected servo.

The SCS commander is not using ROS. 
The program is supposed to be used with a terminal application
or the serial console from the Arduino IDE.
All commands should be terminated by a \n character, either
\f (linefeed) or \r (carriage-return) or both.

* S <id>\n : select servo at address <id> (decimal 3..253) for 
  the next commands.

* i\n :    print motor current (SCS) (may read -1: not implemented by servo)
* j\n :    print motor current (SMS) as multiple of 0.01A 

* l\n :    print motor load (torque) (SCS), unknown units
* m\n :    print motor load (torque) (SMS), unknown units

* p\n :    print motor position (SCS), counts (0..1023)
* q\n :    print motor position (SMS), counts (0..4095)

* t\n :    print servo temperature  (degrees celsius)

* v\n :    print servo supply voltage (deci-volts)

* P <pos>\n:  (SCS-only) move to given position (counts) using current
           speed and/or duration

* Q <pos>\n:  (SMS-only) move to given position (counts) using
              current max-speed and max-acceleration

* S <speed>\n: set motion speed for next P/Q command






## Gripper / Hand firmware and command set

The hand microcontroller currently uses ASCII-based communication
with the host, so that debug messages appear as plain text in the
output stream, and motor values are also human-readable. 

The following lists summarize the most common output messages
and commands (input messages), but may be out-of-sync with the
current release. Please check the source code for the actually
implemented versions and their formatting.

### Firmware output ROS topics:

<node_name>/joint_states          (sensor_msgs/JointState)
<node_name>/joint_errors          (sensor_msgs/JointState: pos=pos vel=vel eff=pos-error)
<node_name>/voltages              (sensor_msgs/Joy: axes:raw values)
<node_name>/temperatures          (sensor_msgs/Joy: axes:raw values)
<node_name>/currents
<node_name>/forces_raw            (sensor_msgs/Joy: axes: raw values)
<node_name>/palm_imu_raw          (sensor_msgs/Imu)
<node_name>/left_finger_imu_raw   (sensor_msgs/Imu)
<node_name>/right_finger_imu_raw  (sensor_msgs/Imu)

### Firmware output messages from the hand

* E: servo position errors (current - goal)
 
  E seq_number stamp pos err1 err2 err3 err4

  published as joint_state message on topic ~joint_position_errors
  with <effort> = errors, <pos> = current servo positions, <vel> =
  current servo velocities

* F: "forces" current force measurementss in Ascii format

  F seq_number stamp forces lc1 lc2 lc3 lc4

  Latest raw values from the load-cells, as measured by
  the HX-711 A/D converters.
  No scaling or temperature compensation is
  applied to the values, which are published as joy messages
  on topic ~/forces_raw and zero-biased on ~/forces_cooked.

  Exponential filtering with alpha=0.1 and 0.01 is also 
  applied to the values to generate high-pass filtered versions
  on topics ~force_filtered_10 and  ~forces_filtered_100

* I: "current" servo motor currents (in Feetech units)

  I seq_number stamp amps i1 i2 i3 i4

  published as joy messages on topic ~currents

* M: "moments" servo motor torques (in Feetech units)

  M seq_number stamp trq m1 m2 m3 m4

  Stored in internal variable and sent as part of 
  next '''joint_state message'''. Also re-published as raw-values 
  (servo counts) to the '''~torques''' topic.

* P: "positions" servo positions (in Feetech counts)

  P seq_number stamp pos p1 p2 p3 p4

  Published as joint_state on topic ~joint_states, where
  <position> = incoming positions converted to radians,
  <velocity> = joint velocity in radians/sec calculated from current
               and previous position message (numerical difference),
               or zero unless at least two messages have been received,
  <effort>   = torques from pervious "M" message

* T: "temperatures" current servo temperaturs in degrees celsius

  T seq_number stamp temp t1 t2 t3 t4 t5 t6 t7

  published as joy messages on topic ~temperatures

* U: "voltages" current servo supply voltages (in deci-volts)

  U seq_number stamp volts u1 u2 u3 u4 u5 u6 u7

  published as joy messages on topic ~voltages

* V: "velocities" current servo velocities (in Feetech units)

  V seq_number stamp vel v1 v2 v3 v4 v5 v6 v7

  currently not used or re-published as ROS message



### Text-commands currently supported by the hand firmware:
