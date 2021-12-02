# Autopilot-Design-for-Longitudinal-Airplane-Model-F-16-


## Introduction and Objective
Design an autopilot for airplane using Linear Quadratic Gaussian (LQG) controller on the
longitudinal flight dynamics of aircraft control system

Objective is to drive four longitudinal outputs (Airspeed, Angle of Attack, Pitch, Pitch Rate) to
follow a reference input with zero steady state errors.

Since it might be impossible to measure all the states in an aircraft for feedback, so we designed an
estimator for all four outputs and used in full-state feedback.

Synthesis and analysis of stability are done in MATLAB
