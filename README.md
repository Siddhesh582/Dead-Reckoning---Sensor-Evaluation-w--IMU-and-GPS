# Dead Reckoning & Sensor Evaluation with IMU and GPS

Overview
This project implements a robust sensor fusion system for vehicle navigation in urban environments where GPS signals are frequently degraded or blocked. By combining high-frequency IMU data with low-frequency GPS measurements, the system achieves reliable dead reckoning capabilities for autonomous vehicle applications.
Problem Statement
Urban navigation presents significant challenges for GPS-based systems due to signal blockage from tall buildings, electromagnetic interference, and multipath effects. This project addresses these challenges through advanced sensor fusion techniques that leverage the complementary strengths of inertial and satellite-based positioning systems.
Technical Approach
Hardware Setup

VN-100 IMU: 9-DOF inertial measurement unit (100 Hz sampling rate)
USB GNSS Receiver: GPS positioning reference (1 Hz update rate)
ROS2 Integration: Real-time data acquisition and logging
Test Platform: Instrumented vehicle for urban data collection

Key Algorithms
1. Magnetometer Calibration

Hard Iron Correction: Removes static magnetic bias from vehicle structure
Soft Iron Correction: Compensates for magnetic field distortions
Result: Transforms elliptical magnetic field distribution to circular

2. Complementary Filtering

Sensor Fusion: Combines gyroscope and magnetometer data for optimal yaw estimation
Filter Coefficient: α = 0.93 (93% gyroscope, 7% magnetometer)
Cutoff Frequency: 1.13 Hz for optimal noise reduction

3. Dead Reckoning Algorithm

Velocity Integration: Processes accelerometer data with bias correction
Trajectory Reconstruction: Estimates 2D vehicle path using heading data
Coordinate Transformation: Converts body frame to navigation frame

