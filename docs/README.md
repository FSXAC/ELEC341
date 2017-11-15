ELEC 341 Design Project

*Muchen He (44838154); Leo Liu (18800152)*

[TOC]

# Selective Laser Sintering 3D Printer

## System Overview

The Selective Laser Sintering (SLS) 3D printer uses a laser to point at different locations on the build platform to solidify resin and produce solid components. The laser can be pointed using a two-axle system powered by two motors. The laser intensity is adjusted depending on which part we are printing such that all area of the part, during printing, receive uniform laser intensity.

Motor 0 is aligned on the Y axis and rotates the inner assembly (which the inner motor, motor 1, is mounted on) on the XZ plane. 

Motor 1 is aligned on the X axis and rotates the laser on the YZ axis.

## Inverse Kinematics

Initially, the printer is fed with Cartesian coordinates of where the laser should be pointed. Since the laser is controlled rotationally, we need to convert them into desired angles for the two motors to turn.

Hence **inverse kinematics**, which is a function block that has Cartesian coordinates as inputs, and outputs desired angle for the motors.

The input is a vector with three components:

1. Desired X position
2. Desired Y position
3. Desired Z position

To compute the angles for motor 0, it is simply $\tan^{-1}\left(\frac {x}{z'}\right)$ and the angle for motor 1 is $\tan^{-1}\left(\frac{y}{z'}\right)$ where $z'$ is the height from the laser to the part being printed.

## Direct Kinematics

For feedback, we need to know the actual X and Y position of where the laser is pointing, hence the **direct kinematic**. Where the input is the actual angle of the motors, and output is the actual position in Cartesian coordinates.

The input is three components which is:

1. Angle of motor 0
2. Angle of motor 1
3. Distance from the laser to the top of part being printed

Thus, the calculation is straightforward. X position is given by $\tan(q_0)\times z'$  and Y position is given by $\tan(q_1)\times z'$.

## Amplifier

Both motors is equipped with the power amplifier (as shown in figure ??). The operational amplifier is set up with the following resistor, capacitor, and inductor values.



## Electromechanical Dynamics

## Mechanical Dynamics

### Motor 0

### Motor 1

#### Static Friction

## Sensor Dynamics

