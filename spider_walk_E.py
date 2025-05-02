from ws_client import RosbridgeClient
import time
import math
import keyboard

joint_init_up = 45
joint_init_low = 60
joint_Hdelta = 5

initial_pose = [joint_init_up,joint_init_low,
                joint_init_up,joint_init_low,
                joint_init_up,joint_init_low,
                joint_init_up,joint_init_low]
joint_positions = initial_pose.copy()
wait_time = 0.1
wait_Tshort = 0.04



def connect_rosbridge(ros_client):
    ros_ip = input("Enter ROSBridge WebSocket IP: ").strip()
    if ros_client.connect(ros_ip):
        ros_client.advertise_topic("/robot_arm", "trajectory_msgs/JointTrajectoryPoint")
    else:
        print(f"[ERROR] Failed to connect to ROSBridge for publishing goal_pose")
        connect_rosbridge(ros_client)

def publishArm(ros_client, joint_positions):    
    joint_positions_radian = [math.radians(degree) for degree in joint_positions]
    ros_client.publish("/robot_arm",{"positions": joint_positions_radian})
    print(joint_positions_radian)


def resetPosition(ros_client):
    joint_positions = initial_pose.copy()
    publishArm(ros_client, joint_positions)
    time.sleep(wait_time)


def forward(ros_client):
    joint_positions[1] = 20
    joint_positions[5] = 75
    publishArm(ros_client, joint_positions)
    time.sleep(wait_time)

    joint_positions[0] = 15
    joint_positions[6] = 15
    joint_positions[2] = 60
    joint_positions[4] = 60
    publishArm(ros_client, joint_positions)
    time.sleep(wait_time)

    joint_positions[1] = 60
    joint_positions[2] = 80
    joint_positions[4] = 80
    publishArm(ros_client, joint_positions)
    time.sleep(wait_time)

    joint_positions[3] = 20
    joint_positions[7] = 75
    publishArm(ros_client, joint_positions)
    time.sleep(wait_time)

    joint_positions[2] = 15
    joint_positions[4] = 15
    joint_positions[0] = 60
    joint_positions[6] = 60
    publishArm(ros_client, joint_positions)
    time.sleep(wait_time)

    joint_positions[3] = 60
    joint_positions[0] = 80
    joint_positions[6] = 80
    publishArm(ros_client, joint_positions)
    time.sleep(wait_time)

    print("forward")

def backward(ros_client):
    joint_positions[1] = 15
    joint_positions[5] = 15
    publishArm(ros_client, joint_positions)

    joint_positions[0] = 75
    joint_positions[6] = 75
    joint_positions[2] = 15
    joint_positions[4] = 15
    publishArm(ros_client, joint_positions)

    joint_positions[1] = 60
    joint_positions[5] = 60
    publishArm(ros_client, joint_positions)

    joint_positions[3] = 15
    joint_positions[7] = 15
    publishArm(ros_client, joint_positions)

    joint_positions[0] = 15
    joint_positions[6] = 15
    joint_positions[2] = 75
    joint_positions[4] = 75
    publishArm(ros_client, joint_positions)

    joint_positions[3] = 60
    joint_positions[7] = 60
    publishArm(ros_client, joint_positions)

    print("backward")

def rotate_counterclockwise(ros_client):
    joint_positions[1] = 15
    joint_positions[5] = 15
    publishArm(ros_client, joint_positions)

    joint_positions[0] = 15
    joint_positions[4] = 15
    publishArm(ros_client, joint_positions)

    joint_positions[1] = 60
    joint_positions[5] = 60
    publishArm(ros_client, joint_positions)

    joint_positions[3] = 15
    joint_positions[7] = 15
    publishArm(ros_client, joint_positions)

    joint_positions[0] = 45
    joint_positions[6] = 45
    joint_positions[2] = 45
    joint_positions[4] = 45
    publishArm(ros_client, joint_positions)

    joint_positions[3] = 60
    joint_positions[7] = 60
    publishArm(ros_client, joint_positions)

    print("turn right")

def rotate_clockwise(ros_client):
    joint_positions[3] = 15
    joint_positions[7] = 15
    publishArm(ros_client, joint_positions)

    joint_positions[2] = 15
    joint_positions[6] = 15
    publishArm(ros_client, joint_positions)

    joint_positions[3] = 60
    joint_positions[7] = 60
    publishArm(ros_client, joint_positions)

    joint_positions[1] = 15
    joint_positions[5] = 15
    publishArm(ros_client, joint_positions)

    joint_positions[0] = 45
    joint_positions[6] = 45
    joint_positions[2] = 45
    joint_positions[4] = 45
    publishArm(ros_client, joint_positions)

    joint_positions[1] = 60
    joint_positions[5] = 60
    publishArm(ros_client, joint_positions)

    print("turn left")



def main():
    ros_client = RosbridgeClient(rosbridge_port=9090)
    connect_rosbridge(ros_client=ros_client)
    resetPosition(ros_client)
    print("start moving")

    resetPosition(ros_client)

    while True:
        
        if keyboard.is_pressed('w'):
            forward(ros_client)
        if keyboard.is_pressed('s'):
            backward(ros_client)
        if keyboard.is_pressed('a'):
            rotate_counterclockwise(ros_client)
        if keyboard.is_pressed('d'):
            rotate_clockwise(ros_client)
        if keyboard.is_pressed('q'):
            resetPosition(ros_client)   

if __name__ == "__main__":
    main()