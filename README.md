# NAVIS: Navigational Assistance for Visually Impaired Shoppers (ROS2 Full Stack)

![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-blue)
![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%205-lightgrey)
![Build](https://img.shields.io/badge/Build-Colcon-success)

**Built With**  
![C++](https://img.shields.io/badge/Language-C++17-blue)
![ROS2](https://img.shields.io/badge/Middleware-ROS2-informational)
![RTABMap](https://img.shields.io/badge/SLAM-RTABMap-success)


---

A modular, ROS2-based robotic system for semi-autonomous navigation using synced stereo vision, real-time obstacle detection, waypoint management, and route planning. Designed for low-cost hardware and real-time responsiveness.

> 🚀 Used to successfully navigate a 4-aisle indoor environment with dynamic obstacle detection and directional guidance. [Demo Video](https://youtu.be/ynSF1JhdaX4?t=409) | [Project Resource List](https://github.com/ucf-sd-spsu25g10)

---

## 1. Key Features

* **Waypoint Orderer**  
  Read in grocery lists from a webapp and optimize route using S-shaped aisle traversal logic.
  
* **Waypoint Manager**  
  Tracks localization from RTAB-Map, determines proximity to next goal, and outputs control signals (e.g. "turn left", "approaching item").
  
* **Obstacle Detection**  
  Real-time stereo depth-based obstacle detection with audible warnings for hazards < 2m ahead.

* **Stereo Vision Driver**  
  Custom ROS2 node using OpenCV + GStreamer to provide synced stereo feeds from UVC cameras.

* **RTAB-Map Integration**  
  Tuned stereo odometry with low-computation real-time SLAM using disparity and point clouds.


---
## 2. Quick Start

```bash
# Clone and build the workspace
cd ~/ros2_ws/src
git clone --recurse-submodules https://github.com/ucf-sd-spsu25g10/navis_ros2.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

---
## 3. System Breakdown

```bash
ros2 launch navis_nav system.launch.py
```

Launched nodes (`package`: node)
* `stereo_cam`: stereo_node
  * Synced UVC camera driver
  * [Repository](https://github.com/adenm-10/stereo-cam?tab=readme-ov-file#8-troubleshooting)
* `rtabmap_slam`: rtabmap
  * RTAB-Map stereo SPLAM stack
  * [Repository](https://github.com/introlab/rtabmap_ros)
* `navis_nav`: control_out_node
  * A node to call specific speaker lines and buzzer strengths
* `navis_nav`: obstacle_detection
  * Based on generated disparity map, output closest obstacle distance
  * Written by Michael Castiglia
* `navis_nav`: waypoint_orderer
  * Retrieve an unordered list of groceries from web app, and order them in respect so S-Shape routing
* `navis_nav`: waypoint_manager
  * Uses RTAB-Map transform data and current pose to determine if the user is approaching a goal, and triggers directional output (left, right, arrived) to navigate the shopper.

---
## 4. Troubleshooting
1. No cameras detected:
   - **Check payload type, resolution, and framerate compatibility for both cameras**
     - Read about UVC camera configuration [here](https://github.com/adenm-10/stereo-cam?tab=readme-ov-file#4-configuration)
   - Verify camera permissions
   - Ensure all packages are properly installed

2. Poor performance:
    - Check poor performance section of stereo_cam readme for suggestions here:
      - https://github.com/adenm-10/stereo-cam?tab=readme-ov-file#8-troubleshooting

---
## 5. License

This package is released under the GNU General Public License v3.0 (GPLv3). See the [LICENSE](LICENSE) file for details.

This means:
- You can freely use and modify this software
- If you distribute the software or hardware containing this software, you must:
  - Make the source code available
  - License your modifications under GPLv3
  - Provide installation instructions
- Hardware sales are allowed, but software modifications must remain open source

---
## 6. References
* [ROS 2 Docs](https://docs.ros.org/en/jazzy/)
* [rtabmap\_ros](https://github.com/introlab/rtabmap_ros/tree/ros2)
* [image\_pipeline](https://docs.ros.org/en/ros2_packages/rolling/api/image_pipeline/)
* [stereo\_image\_proc](https://docs.ros.org/en/ros2_packages/rolling/api/stereo_image_proc/doc/index.html)

---

Developed by [Aden McKinney](https://github.com/adenm-10)     
University of Central Florida, Computer Engineering Senior Design