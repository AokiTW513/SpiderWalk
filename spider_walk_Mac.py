from ws_client import RosbridgeClient
import time
import math
from pynput import keyboard

initial_pose = [45,60,45,60,45,60,45,60] # initial angle(degrees) of joints
# [FrontRightHip, FrontRightKnee, FrontLeftHip, FrontLeftKnee, BackLeftHip, BackLeftKnee, BackRightHip, BackRightKnee]
joint_positions = initial_pose.copy() # initialize joint angle
wait_time = 0.1 # arbitrary time to let the joints finish rotating
hip_stride = 30 # amount the hip joints rotate while walking, larger makes the stride larger but also makes the body rotate more
knee_raise = 30 # amount knee joints raise while walking, larger can make the robot tilt more
duration_threshold = 0.1 # reset the robot after a duration of no input

pressed_keys = set()

def on_press(key):
    global last_trigger_time
    try:
        if key.char == 'w':
            forward(ros_client)
        elif key.char == 's':
            backward(ros_client)
        elif key.char == 'a':
            rotate_counterclockwise(ros_client)
        elif key.char == 'd':
            rotate_clockwise(ros_client)
        # elif key.char == ' ':
        #     jump(ros_client)
        last_trigger_time = time.time()
    except AttributeError:
        pass

def on_release(key):
    if key == keyboard.Key.esc:
        return False  # 停止監聽

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


def forward(ros_client):
    joint_positions[1] = initial_pose[1] - knee_raise
    joint_positions[5] = initial_pose[5] - knee_raise
    publishArm(ros_client, joint_positions)
    time.sleep(wait_time)

    joint_positions[0] = initial_pose[0] - hip_stride
    joint_positions[6] = initial_pose[6] - hip_stride
    joint_positions[2] = initial_pose[2] + hip_stride
    joint_positions[4] = initial_pose[4] + hip_stride
    publishArm(ros_client, joint_positions)
    time.sleep(wait_time)

    joint_positions[1] = initial_pose[1]
    joint_positions[5] = initial_pose[5]
    publishArm(ros_client, joint_positions)
    time.sleep(wait_time)

    joint_positions[3] = initial_pose[3] - knee_raise
    joint_positions[7] = initial_pose[7] - knee_raise
    publishArm(ros_client, joint_positions)
    time.sleep(wait_time)

    joint_positions[0] = initial_pose[0] + hip_stride
    joint_positions[6] = initial_pose[6] + hip_stride
    joint_positions[2] = initial_pose[2] - hip_stride
    joint_positions[4] = initial_pose[4] - hip_stride
    publishArm(ros_client, joint_positions)
    time.sleep(wait_time)

    joint_positions[3] = initial_pose[3]
    joint_positions[7] = initial_pose[7]
    publishArm(ros_client, joint_positions)
    time.sleep(wait_time)

    print("forward")

def backward(ros_client):
    joint_positions[1] = initial_pose[1] - knee_raise
    joint_positions[5] = initial_pose[5] - knee_raise
    publishArm(ros_client, joint_positions)
    time.sleep(wait_time)

    joint_positions[0] = initial_pose[0] + hip_stride
    joint_positions[6] = initial_pose[6] + hip_stride
    joint_positions[2] = initial_pose[2] - hip_stride
    joint_positions[4] = initial_pose[4] - hip_stride
    publishArm(ros_client, joint_positions)
    time.sleep(wait_time)

    joint_positions[1] = initial_pose[1]
    joint_positions[5] = initial_pose[5]
    publishArm(ros_client, joint_positions)
    time.sleep(wait_time)

    joint_positions[3] = initial_pose[3] - knee_raise
    joint_positions[7] = initial_pose[7] - knee_raise
    publishArm(ros_client, joint_positions)
    time.sleep(wait_time)

    joint_positions[0] = initial_pose[0] - hip_stride
    joint_positions[6] = initial_pose[6] - hip_stride
    joint_positions[2] = initial_pose[2] + hip_stride
    joint_positions[4] = initial_pose[4] + hip_stride
    publishArm(ros_client, joint_positions)
    time.sleep(wait_time)

    joint_positions[3] = initial_pose[3]
    joint_positions[7] = initial_pose[7]
    publishArm(ros_client, joint_positions)
    time.sleep(wait_time)

    print("backward")

def rotate_counterclockwise(ros_client):
    joint_positions[1] = initial_pose[1] - knee_raise
    joint_positions[5] = initial_pose[5] - knee_raise
    publishArm(ros_client, joint_positions)
    time.sleep(wait_time)

    joint_positions[0] = initial_pose[0] - hip_stride
    joint_positions[4] = initial_pose[4] - hip_stride
    publishArm(ros_client, joint_positions)
    time.sleep(wait_time)    

    joint_positions[1] = initial_pose[1]
    joint_positions[5] = initial_pose[5]
    publishArm(ros_client, joint_positions)
    time.sleep(wait_time)

    joint_positions[3] = initial_pose[3] - knee_raise
    joint_positions[7] = initial_pose[7] - knee_raise
    publishArm(ros_client, joint_positions)
    time.sleep(wait_time)

    joint_positions[0] = initial_pose[0]
    joint_positions[4] = initial_pose[4]
    joint_positions[2] = initial_pose[2]
    joint_positions[6] = initial_pose[6]
    publishArm(ros_client, joint_positions)
    time.sleep(wait_time)

    joint_positions[3] = initial_pose[3]
    joint_positions[7] = initial_pose[7]
    publishArm(ros_client, joint_positions)
    time.sleep(wait_time)

    print("turn left")

def rotate_clockwise(ros_client):
    joint_positions[3] = initial_pose[3] - knee_raise
    joint_positions[7] = initial_pose[7] - knee_raise
    publishArm(ros_client, joint_positions)
    time.sleep(wait_time)

    joint_positions[2] = initial_pose[2] - knee_raise
    joint_positions[6] = initial_pose[6] - knee_raise
    publishArm(ros_client, joint_positions)
    time.sleep(wait_time)

    joint_positions[3] = initial_pose[3]
    joint_positions[7] = initial_pose[7]
    publishArm(ros_client, joint_positions)
    time.sleep(wait_time)

    joint_positions[1] = initial_pose[1] - knee_raise
    joint_positions[5] = initial_pose[5] - knee_raise
    publishArm(ros_client, joint_positions)
    time.sleep(wait_time)

    joint_positions[2] = initial_pose[2]
    joint_positions[6] = initial_pose[6]
    joint_positions[0] = initial_pose[0]
    joint_positions[4] = initial_pose[4]
    publishArm(ros_client, joint_positions)
    time.sleep(wait_time)    

    joint_positions[1] = initial_pose[1]
    joint_positions[5] = initial_pose[5]
    publishArm(ros_client, joint_positions)
    time.sleep(wait_time)

    print("turn right")

def jump(ros_client):
    joint_positions[2] = initial_pose[2]
    joint_positions[6] = initial_pose[6]
    joint_positions[0] = initial_pose[0]
    joint_positions[4] = initial_pose[4]
    joint_positions[1] = initial_pose[1] - knee_raise
    joint_positions[3] = initial_pose[3] - knee_raise
    joint_positions[5] = initial_pose[5] - knee_raise
    joint_positions[7] = initial_pose[7] - knee_raise
    publishArm(ros_client, joint_positions)
    time.sleep(wait_time)   

    joint_positions[1] = initial_pose[1] + knee_raise*2
    joint_positions[3] = initial_pose[3] + knee_raise*2
    joint_positions[5] = initial_pose[5] + knee_raise*2
    joint_positions[7] = initial_pose[7] + knee_raise*2
    publishArm(ros_client, joint_positions)
    time.sleep(wait_time)  

def main():
    global ros_client, last_trigger_time
    ros_client = RosbridgeClient(rosbridge_port=9090)
    connect_rosbridge(ros_client=ros_client)
    last_trigger_time = time.time()
    resetPosition(ros_client)
    print("start moving")

    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        while listener.running:
            if time.time() - last_trigger_time >= duration_threshold:
                resetPosition(ros_client)
                last_trigger_time = time.time()
            time.sleep(0.05)

if __name__ == "__main__":
    main()