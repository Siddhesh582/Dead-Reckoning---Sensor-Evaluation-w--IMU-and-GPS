🚗 ## Dead Reckoning & Sensor Evaluation with IMU and GPS
📌 # Overview
This project implements a robust sensor fusion system for vehicle navigation in urban environments where GPS signals are frequently degraded or blocked. By combining high-frequency IMU data with low-frequency GPS measurements, the system achieves reliable dead reckoning capabilities suitable for autonomous vehicle applications.

❗ # Problem Statement
Urban navigation presents significant challenges for GPS-based systems due to:

Signal blockage from tall buildings

Electromagnetic interference

Multipath effects

This project addresses these limitations using advanced sensor fusion techniques that combine the strengths of inertial and satellite-based positioning systems.

⚙️ # Technical Approach
🧰 Hardware Setup
VN-100 IMU – 9-DOF inertial measurement unit (100 Hz sampling rate)

USB GNSS Receiver – GPS positioning reference (1 Hz update rate)

ROS2 Integration – Real-time data acquisition and logging

Test Platform – Instrumented vehicle for urban data collection

📊 # Data Collection
Circular Motion Dataset – Controlled driving for calibration parameter extraction

Urban Navigation Dataset – 2–3 km route through Boston for validation

ROSbag Recording – Synchronized multi-sensor data logging

🧠 # Key Algorithms
1. Magnetometer Calibration
Hard Iron Correction – Removes static magnetic bias from vehicle structure

Soft Iron Correction – Compensates for magnetic field distortions

✅ Result: Transforms elliptical magnetic field distribution into a circular pattern

2. Complementary Filtering
Sensor Fusion – Combines gyroscope and magnetometer data for optimal yaw estimation

Filter Coefficient – α = 0.93 (93% gyroscope, 7% magnetometer)

Cutoff Frequency – 1.13 Hz for optimal noise reduction

3. Dead Reckoning Algorithm
Velocity Integration – Processes accelerometer data with bias correction

Trajectory Reconstruction – Estimates 2D vehicle path using heading data

Coordinate Transformation – Converts from body frame to navigation frame

✨ # Key Features
🟢 Real-time Processing – 100 Hz IMU data processing with ROS2

🧲 Robust Calibration – Systematic magnetometer bias correction

📐 Adaptive Filtering – Complementary filter for optimal sensor fusion

🗺️ Trajectory Validation – GPS comparison for performance assessment

🌆 Urban Testing – Validated in challenging electromagnetic environments

📌 # Technical Parameters
Parameter	Value
Hard Iron Offsets	X: -22.05, Y: -62.60
Soft Iron Matrix	0.0059, 0.0010; 0.0010, 0.0096
Filter Coefficient	α = 0.93
Scaling Factor	0.65 (trajectory alignment)
IMU Sampling Rate	100 Hz
GPS Sampling Rate	1 Hz

🚀 # Applications
Autonomous Vehicles – GPS-denied navigation systems

Mobile Robotics – Indoor/outdoor localization

Fleet Management – Enhanced vehicle tracking

Emergency Services – Reliable positioning in degraded signal zones

🧪 # Technologies Used
MATLAB – Algorithm development and signal processing

ROS2 – Real-time sensor integration

Signal Processing – Digital filtering and noise reduction

Navigation – Dead reckoning and coordinate transformations

🧠 # Skills Demonstrated
Multi-sensor data fusion and calibration

Digital signal processing and filtering

Real-time embedded systems programming

Navigation algorithm development

Performance analysis and validation

🔮 # Future Enhancements
Kalman Filter – Adaptive estimation for improved accuracy

Machine Learning – Enhanced sensor fusion capabilities

LiDAR Integration – Augmented positioning in dense urban zones

Real-time Embedded Deployment – For production-grade systems
