from ws_client import RosbridgeClient
import time
import math
import keyboard

initial_pose = [90,0,0,0,0,0] # initial angle(degrees) of joints
# [FrontRightHip, FrontRightKnee, FrontLeftHip, FrontLeftKnee, BackLeftHip, BackLeftKnee, BackRightHip, BackRightKnee]
joint_positions = initial_pose.copy() # initialize joint angle
wait_time = 0.5 # arbitrary time to let the joints finish rotating
hip_stride = 30 # amount the hip joints rotate while walking, larger makes the stride larger but also makes the body rotate more
knee_raise = 30 # amount knee joints raise while walking, larger can make the robot tilt more
duration_threshold = 0.1 # reset the robot after a duration of no input


def connect_rosbridge(ros_client):
    ros_ip = input("Enter ROSBridge WebSocket IP: ").strip()
    if ros_client.connect(ros_ip):
        ros_client.advertise_topic("/robot_arm", "trajectory_msgs/JointTrajectoryPoint")
    else:
        print(f"[ERROR] Failed to connect to ROSBridge for publishing goal_pose")
        connect_rosbridge(ros_client)

#publishes the /robot_arm topic 
def publishArm(ros_client, joint_positions_degree):    
    joint_positions_radian = [math.radians(degree) for degree in joint_positions_degree]
    ros_client.publish("/robot_arm",{"positions": joint_positions_radian})
    print(joint_positions_radian)


def resetPosition(ros_client):
    publishArm(ros_client, initial_pose)
    time.sleep(wait_time)

def wiggle(ros_client):
    joint_positions[0] = 60
    publishArm(ros_client, joint_positions)
    time.sleep(wait_time)  

    joint_positions[0] = 90
    publishArm(ros_client, joint_positions)
    time.sleep(wait_time)  

    joint_positions[0] = 120
    publishArm(ros_client, joint_positions)
    time.sleep(wait_time)  

    joint_positions[0] = 90
    publishArm(ros_client, joint_positions)
    time.sleep(wait_time)  

def main():
    ros_client = RosbridgeClient(rosbridge_port=9090)
    connect_rosbridge(ros_client=ros_client)
    resetPosition(ros_client)
    print("start moving")

    while True:
        wiggle(ros_client)


if __name__ == "__main__":
    main()