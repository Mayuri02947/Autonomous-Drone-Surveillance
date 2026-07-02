# Autonomous Drone Surveillance System

A ROS2 + Gazebo simulation of a quadcopter drone that patrols an area, streams live video, and detects intrusions using OpenCV-based motion detection.

## Project Overview

This project simulates an autonomous surveillance drone that patrols a fixed patrol path using position-based waypoint navigation, while continuously monitoring its front camera feed for movement or unknown objects. On detecting an intrusion, the system logs an alert, publishes a notification message, and saves a timestamped snapshot — exactly like a real perimeter-security drone.

**Objective:** Simulate a drone that patrols, watches, and alerts using ROS2, Gazebo, and OpenCV.

## Features

- Waypoint-based patrol using the drone's built-in position-control mode
- Real GPS telemetry logging (lat/lon/altitude) at every patrol step
- Live camera streaming from a simulated front-facing drone camera
- OpenCV-based motion detection using background subtraction (MOG2)
- Automatic intrusion alerting with cooldown to prevent alert spam
- Snapshot capture on every detected intrusion, saved with timestamps
- Modular ROS2 nodes — navigation and detection run independently and communicate only via topics

## System Architecture
[GPS Sensor] --> /simple_drone/gps/nav --> [Waypoint Navigator Node]
|
[Waypoint List] ---------------------------------->|
|
[Position Control Loop]
|
/simple_drone/cmd_vel --> [Gazebo Drone Plugin]
|
[Drone Motion]
[Front Camera] --> /simple_drone/front/image_raw --> [Object Detector Node]
|
[Background Subtraction]
|
/surveillance/alert --> [Alert + Snapshot]
/surveillance/detection_image --> [Live View]

## Tech Stack

| Tool | Version | Purpose |
|---|---|---|
| ROS2 | Humble | Robot middleware |
| Gazebo | Classic 11 | Physics simulation |
| Python | 3.10 | Node implementation |
| OpenCV | 4.x | Motion/object detection |
| sjtu_drone | ROS2 branch | Drone model + flight plugin |
| Ubuntu | 22.04 | Operating system |

## Package Structure
drone_surveillance/
├── drone_surveillance/
│   ├── init.py
│   ├── waypoint_navigator.py    # Patrol logic + GPS logging
│   └── object_detector.py       # Camera feed + detection + alerts
├── launch/
│   └── surveillance.launch.py   # Full surveillance system launch file
├── setup.py
├── package.xml
└── README.md

## Navigation — Position-Controlled Patrol

The drone flies in **position-control mode**, where each published command represents a target
(x, y, z, yaw) rather than a raw velocity. The onboard plugin handles the low-level stabilization
internally, so the patrol loop simply cycles through a fixed set of waypoints:
waypoints = [(5, 0, 5), (5, 5, 5), (0, 5, 5), (0, 0, 5)]
target = waypoints[i]

Each waypoint is held for a fixed duration before advancing to the next, looping continuously to
form a rectangular patrol path.

| Parameter | Value | Role |
|---|---|---|
| Patrol altitude | 5.0 m | Constant cruising height |
| Waypoint hold time | 6.0 s | Time spent at each corner before advancing |
| Command rate | 5 Hz | Frequency of position target re-publishing |

## GPS Telemetry Logging

Real simulated GPS is read from the drone's onboard GPS sensor on every patrol step and logged
alongside the local waypoint coordinates, giving a geo-referenced patrol trail:
lat, lon, alt = current_gps.latitude, current_gps.longitude, current_gps.altitude

## Object Detection — Background Subtraction

Rather than a static reference frame, the detector uses a **MOG2 background subtractor**, which
builds a running statistical model of the "empty" scene and flags anything that deviates from it:
foreground_mask = MOG2.apply(frame)
contours = find_contours(foreground_mask)

Detected regions are filtered by minimum area to reject sensor noise before being classified as
a potential intrusion.

| Parameter | Value | Role |
|---|---|---|
| History | 200 frames | Background model memory length |
| Variance threshold | 40 | Sensitivity to change |
| Minimum contour area | 800 px | Noise rejection threshold |
| Alert cooldown | 4.0 s | Prevents repeated alerts for the same event |

## Alerting Logic

On a valid detection, three things happen simultaneously:
log.warn("INTRUSION ALERT: ...")
publish(/surveillance/alert, alert_text)
save_snapshot(~/intrusion_snapshots/intrusion_<timestamp>.jpg)

## Drone Model Overview

The simulated drone (via `sjtu_drone`) consists of:

- **Body frame** — central quadcopter chassis with 4 rotor links
- **Front camera** — forward-facing RGB sensor used for surveillance detection
- **Bottom camera** — secondary downward-facing sensor (unused in current pipeline)
- **GPS sensor** — publishes NavSatFix telemetry (lat/lon/alt)
- **IMU** — provides orientation and acceleration data for internal stabilization
- **Position-control plugin** — accepts (x, y, z, yaw) targets and handles low-level flight control internally
