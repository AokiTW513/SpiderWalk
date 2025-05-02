# Simple script to control quadruped spider

This script sends the topic "/robot_arm" with the type "trajectory_msgs/JointTrajectoryPoint" to control joints.
Beware the order of joints: [FrontRightHip, FrontRightKnee, FrontLeftHip, FrontLeftKnee, BackLeftHip, BackLeftKnee, BackRightHip, BackRightKnee]
The urdf and mesh files are in spiderV3C_clear_to_unity

## Setup

**Install Dependencies:**
   ```bash
   pip install -r .\requirements.txt
   ```
## Usage

**Run ROSBridge Server:**
   Launch the ROSBridge server (adjust the command as per your ROS setup):
   ```bash
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml
   ```

**Start the Application:**
   ```bash
   python spider_walk.py
   ```