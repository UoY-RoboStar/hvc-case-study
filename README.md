# hvc-case-study
This repository contains the RoboChart/CSPM sources for the High-Voltage Controller (HVC) system.

Note that there are two RoboTool projects in this repository:

1. hvc_external_hardware

This is a model of the HVC system as used for co-verification
in the paper "Safety assurance of an industrial robotic control
system usinghardware/software co-verification" by Murray et al.

The CSPM scripts used in the verification are also included 
under the folder `csp-gen/comp/timed/properties_assertions.csp`
that can be loaded into FDR. To re-generate these, a development
version of RoboTool must be used.

2. smbf

This is an older model of the HVC system developed in the context of
"Safety Assurance of a High Voltage Controller foran Industrial 
Robotic System" by Murray et al.

This model should work fine with the existing public version of
RoboTool, as available from https://robostar.cs.york.ac.uk/robotool/.
