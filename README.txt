QUADCOPTER AUTOPILOT AND SIMULATION FRAMEWORK
==============================================

Overview
--------

This software implements a modular quadcopter flight stack capable of running both:

• Hardware flight on a Linux-based embedded system  
• Full simulation on Windows

The system includes state estimation, control loops, sensor interfaces, and a physics simulator. The architecture allows the same flight logic to run in simulation and on real hardware with minimal changes.

Core capabilities include:

- Extended Kalman Filter (EKF) state estimation
- Attitude estimation using an AHRS filter
- Position and velocity control
- Manual and autonomous flight modes
- Hardware interfaces for IMU, RC receiver, and motors
- Full physics simulation environment
- UDP telemetry streaming
- Platform abstraction for Windows (simulation) and Linux (hardware)

---------------------------------------------------------------------

SYSTEM ARCHITECTURE
-------------------

The program runs a real-time loop which manages sensor updates, state estimation, guidance logic, control loops, actuation, telemetry, and simulation.

High-level architecture:

Sensors → State Estimation → Guidance → Control → Motor Mixing → Actuation
                                     ↓
                                Telemetry
                                     ↓
                                 Simulation

The system uses multi-rate task scheduling, allowing different subsystems to run at different frequencies (IMU, navigation, control loops, etc.).

---------------------------------------------------------------------

MAJOR MODULES
-------------

Common Utilities
----------------

TimeKeeper

Manages:

- Simulation time
- Loop delta time
- Task clocks
- Scheduled subsystem update rates


MathUtils

Provides mathematical helper functions used throughout the system such as:

- vector utilities
- clamp functions
- coordinate conversions

---------------------------------------------------------------------

ESTIMATION
----------

EKF (Extended Kalman Filter)

The EKF estimates the quadcopter state using IMU and external position measurements.

Estimated state includes:

- Attitude (roll, pitch, yaw)
- Position (North, East, Down)
- Velocity
- Sensor biases

Key functions:

initializeFromOpti()  
Initializes the filter state using external position measurements.

predict()  
Propagates the state forward using IMU measurements.

correct()  
Updates the state using external measurements.

getx()  
Returns the current navigation state.

getHealth()  
Returns a Normalized Innovation Squared (NIS) metric used to evaluate filter health.

The system monitors EKF health continuously. If the NIS value becomes too large for an extended period, the filter is considered unreliable and the control system falls back to AHRS-based stabilization.


AHRS (Attitude and Heading Reference System)

A complementary filter used for attitude estimation using:

- gyroscope
- accelerometer

The AHRS serves primarily as a backup attitude estimator when the EKF becomes unhealthy.

Primary output:

Vec<3> euler()

Returns roll, pitch, and yaw.

---------------------------------------------------------------------

GUIDANCE
--------

ModeManager

The ModeManager handles high-level flight logic and determines how the vehicle should move.

Responsibilities include:

- determining commanded positions
- switching between flight modes
- managing autonomous vs manual control

Modes include:

Manual velocity control  
Autonomous position tracking

---------------------------------------------------------------------

CONTROL SYSTEM
--------------

The control system uses a cascaded control architecture consisting of an outer loop and an inner loop.

Outer Loop (Position Controller)

Inputs:

- position
- velocity
- desired position
- yaw angle

Outputs:

- desired attitude
- commanded thrust

This controller converts position errors into desired roll and pitch commands.


Inner Loop (Attitude Controller)

Inputs:

- desired attitude
- yaw rate command
- current attitude
- angular velocity

Outputs:

- body moments (Mx, My, Mz)

A PD controller is used to regulate roll, pitch, and yaw.

---------------------------------------------------------------------

MOTOR MIXING
------------

QuadMixer

The mixer converts desired vehicle forces and moments into individual motor thrust commands.

Input wrench:

[Fz, Mx, My, Mz]

Output thrusts:

[T1, T2, T3, T4]

Functions include:

mix2Thrust()  
thr2PWM()  
mix2Wrench()

---------------------------------------------------------------------

SIMULATION ENVIRONMENT
----------------------

The simulation environment allows full testing of the flight stack without real hardware.

Dynamics

Simulates quadcopter rigid-body motion including:

- translational dynamics
- rotational dynamics
- gravity


MotorModel

Simulates motor response characteristics including:

- thrust generation
- actuator lag
- thrust limits


Sensor Simulation

ImuSim

Simulates an IMU including:

- gyro measurements
- accelerometer measurements
- noise


OptiSim

Simulates motion capture measurements including:

- position
- yaw

---------------------------------------------------------------------

HARDWARE INTERFACE (LINUX)
--------------------------

When compiled on Linux, the system interfaces with real flight hardware.

IMUHandler

Communicates with the onboard IMU sensor.

Provides:

- accelerometer data
- gyroscope data
- sensor statistics and calibration


RCIn

Reads radio control commands through a PPM receiver.

Outputs normalized control signals for:

- roll
- pitch
- throttle
- yaw


MotorDriver

Controls motor PWM outputs.

Functions include:

initialize()  
command()  
wind_down()

---------------------------------------------------------------------

MANUAL CONTROL
--------------

Windows (Simulation)

Keyboard controls allow manual flight in simulation.

Controls:

W / S → forward / backward velocity  
A / D → left / right velocity  
Space → descend  
Shift → ascend  
Q / E → yaw rotation


Linux (Real Hardware)

Manual flight uses an RC transmitter.

Typical channels include:

Roll  
Pitch  
Throttle  
Yaw  
Arm switch  
Autopilot switch

---------------------------------------------------------------------

SAFETY AND FAILSAFES
--------------------

Arming Logic

Motors will only activate if:

- the vehicle is armed
- the arm timer has elapsed
- valid PWM commands are generated


EKF Health Monitoring

The EKF health is monitored using the Normalized Innovation Squared (NIS) statistic.

If the EKF becomes unhealthy for an extended period:

- autonomous flight is disabled
- attitude stabilization switches to the AHRS estimator

---------------------------------------------------------------------

TELEMETRY
---------

Telemetry data is sent via UDP for monitoring and logging.

UdpSender streams the following data:

- vehicle state
- EKF health
- control commands
- IMU measurements
- motor commands

Default configuration:

IP: 127.0.0.1  
Port: 8080

---------------------------------------------------------------------

SIMULATION VS HARDWARE MODES
----------------------------

The software automatically changes behavior based on compilation platform.

Windows

Used for:

- full simulation
- keyboard control
- simulated sensors


Linux

Used for:

- real IMU hardware
- RC receiver input
- real motor outputs

---------------------------------------------------------------------

TYPICAL EXECUTION FLOW
----------------------

Each loop iteration performs the following steps:

1. Update simulation time
2. Update sensors (IMU / OptiTrack)
3. Run EKF prediction
4. Run EKF correction
5. Update Mode Manager
6. Read manual inputs
7. Run outer position controller
8. Update AHRS
9. Run inner attitude controller
10. Mix motor commands
11. Send telemetry
12. Step simulation physics

---------------------------------------------------------------------

EXTERNAL DEPENDENCIES
---------------------

This project requires the following external libraries:

Navio2

Hardware interface library for the Navio2 autopilot board.  
Provides access to IMU sensors, PWM motor outputs, and RC input.

Repository:
https://github.com/emlid/Navio2


Nlohmann JSON

Header-only JSON library used for configuration and data serialization.

Repository:
https://github.com/nlohmann/json


Eigen

High-performance linear algebra library used for vectors, matrices, and numerical operations throughout the control and estimation algorithms.

Repository:
https://eigen.tuxfamily.org

---------------------------------------------------------------------

SUMMARY
-------

This project implements a complete quadcopter flight stack including:

- state estimation
- guidance
- control
- motor mixing
- hardware interfaces
- physics simulation
- telemetry

The architecture allows the same flight code to run in both simulation and real hardware environments, enabling rapid development, testing, and deployment of quadcopter flight software.