# TRAC-IK-Lib a standalone implementation

Trac-IK is an Inverse Kinematics library developed by [TracLabs](https://traclabs.com/). It is based on KDL library and implements a faster alternative Inverse Kinematics solver to the popular inverse Jacobian methods in KDL. Two algorithms are proposed:
- An extension to the KDL's Newton-based convergence algorithm, which detects and mitigates local minima due to joint limits by random jumps. 
- A SQP (Sequential Quadratic Programming) nonlinear optimization approach, which uses quasi-Newton methods that better handle joint limits.


# Why the fork?

The original implementation of Trac-IK is tightly linked with the ROS (Robotic Operating System) and do not allow an easy installation outside that environment. 

This repo adapts the current trac_ik_lib package to become a standlone library stripping down dependencies with ROS.


