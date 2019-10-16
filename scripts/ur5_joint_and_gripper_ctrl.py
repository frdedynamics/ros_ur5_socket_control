#!/usr/bin/env python
""" module docstring, yo! """

import sys
import socket
import time
import rospy
import math
import tf_conversions
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import Float64

HOST = ''
PORT = 30000
ROBOT_IP = '172.31.1.146'

PC_SOCK = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
PC_SOCK.settimeout(5)
PC_SOCK.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

CMD_GRIP = False
CMD_GRIP_POSITION = ""
CMD_TCP_POSITION = ""
CMD_JOINT_POSITIONS = ""

def callback(data):
    """ function docstring, yo! """
    global CMD_TCP_POSITION  # shame! shame! shame!
    angle = 2 * math.acos(data.orientation.w)
    x = data.orientation.x / math.sqrt(1 - data.orientation.w*data.orientation.w)
    y = data.orientation.y / math.sqrt(1 - data.orientation.w*data.orientation.w)
    z = data.orientation.z / math.sqrt(1 - data.orientation.w*data.orientation.w)

    Rx = x*angle
    Ry = y*angle
    Rz = z*angle

    X = data.position.x
    Y = data.position.y
    Z = data.position.z

    CMD_TCP_POSITION = "(%f, %f, %f, %f, %f, %f)" % (X, Y, Z, Rx, Ry, Rz)

def callback_grip(data):
    global CMD_GRIP
    CMD_GRIP = data.data

def callback_grip_pos(data):
    global CMD_GRIP_POSITION
    CMD_GRIP_POSITION = "(%f)" % (data.data)

def callback_joint(data):
    global CMD_JOINT_POSITIONS
    CMD_JOINT_POSITIONS = "(%f, %f, %f, %f, %f, %f)" % (data.position[0], data.position[1], data.position[2], data.position[3], data.position[4], data.position[5])


def talker_listener():
    global CMD_GRIP, CMD_JOINT_POSITIONS, CMD_TCP_POSITION  # shame! shame! shame!
    cmd_grip_prev = False
    cmd_tcp_position_prev = ""
    cmd_joint_positions_prev = ""
    cmd_grip_position_prev = ""
    cartesian_pose_pub = rospy.Publisher(
        'cartesian_pose',
        PoseStamped,
        queue_size=1
        )
    joint_state_pub = rospy.Publisher(
        'joint_state',
        JointState,
        queue_size=1
        )

    rospy.init_node('ur5_tcp_comms', anonymous=True)
    rospy.Subscriber("target_EE_pose", Pose, callback)
    rospy.Subscriber("cmd_joint", JointState, callback_joint)
    rospy.Subscriber("cmd_grip", Bool, callback_grip)
    rospy.Subscriber("cmd_grip_pos", Float64, callback_grip_pos)
    rate = rospy.Rate(50)

    rospy.loginfo("binding port..")
    PC_SOCK.bind((HOST, PORT))
    rospy.loginfo("listening..")
    PC_SOCK.listen(5)

    rospy.loginfo("check port..")
    SOCK, addr = PC_SOCK.accept() # Establish connection with client.
    SOCK.settimeout(15)
    rospy.loginfo("Connected to address: %s" ,addr)
    if addr[0] == ROBOT_IP:
        rospy.loginfo("Connected to ROBOT")
    else:
        rospy.loginfo("Client IP does not match ROBOT_IP")

    msg_joint_state = JointState()
    msg_joint_state.name = ['shoulder_pan_joint', 
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint']

    msg_tcp = PoseStamped()

    while not rospy.is_shutdown():
        try:
            SOCK.send("(2)")
            time.sleep(0.01)
            data = SOCK.recv(1024)
            # print data
            msg_joint_state.header.stamp = rospy.get_rostime()
            l = list(data[1:len(data)-1].split(','))
            msg_joint_state.position = [float(i) for i in l]
            joint_state_pub.publish(msg_joint_state)


            SOCK.send("(5)")
            time.sleep(0.01)
            data = SOCK.recv(1024)
            # print data
            msg_tcp.header.stamp = rospy.get_rostime()
            l = list(data[2:len(data)-1].split(','))
            l_float = [float(i) for i in l]
            msg_tcp.pose.position.x = float(l[0])
            msg_tcp.pose.position.y = float(l[1])
            msg_tcp.pose.position.z = float(l[2])

            Rx = float(l[3])
            Ry = float(l[4])
            Rz = float(l[5])

            angle = math.sqrt(Rx*Rx + Ry*Ry + Rz*Rz)
            unit_vector = [Rx/angle, Ry/angle, Rz/angle]

            tmp = [
                unit_vector[0]*math.sin(angle/2),
                unit_vector[1]*math.sin(angle/2),
                unit_vector[2]*math.sin(angle/2),
                math.cos(angle/2)
                ]

            msg_tcp.pose.orientation = tmp
            cartesian_pose_pub.publish(msg_tcp)

            if CMD_GRIP != cmd_grip_prev:
                if CMD_GRIP:
                    SOCK.send("(4)") # close
                    ack = SOCK.recv(1024)
                    if ack != "1":
                        rospy.logerr("cmd grip failed failed with cmd: %s", CMD_GRIP)
                else:
                    SOCK.send("(3)") # open
                    ack = SOCK.recv(1024)
                    if ack != "1":
                        rospy.logerr("cmd grip failed failed with cmd: %s", CMD_GRIP)

                cmd_grip_prev = CMD_GRIP

            if CMD_JOINT_POSITIONS != cmd_joint_positions_prev:
                SOCK.send("(1)")
                time.sleep(0.01)
                SOCK.send(CMD_JOINT_POSITIONS)
                cmd_joint_positions_prev = CMD_JOINT_POSITIONS
                ack = SOCK.recv(1024)
                if ack != "1":
                    rospy.logerr("cmd joint failed failed with cmd: %s", CMD_JOINT_POSITIONS)

            if CMD_TCP_POSITION != cmd_tcp_position_prev:
                SOCK.send("(6)")
                time.sleep(0.01)
                SOCK.send(CMD_TCP_POSITION)
                cmd_tcp_position_prev = CMD_TCP_POSITION
                ack = SOCK.recv(1024)
                if ack != "1":
                    rospy.logerr("cmd tcp failed failed with cmd: %s", CMD_TCP_POSITION)
                # print "new TCP position cmd: %s", CMD_TCP_POSITION

	    if CMD_GRIP_POSITION != cmd_grip_position_prev:
                SOCK.send("(7)")
                time.sleep(0.01)
                SOCK.send(CMD_GRIP_POSITION)
                cmd_grip_position_prev = CMD_GRIP_POSITION
                ack = SOCK.recv(1024)
                if ack != "1":
                    rospy.logerr("cmd grip position failed failed with cmd: %s", CMD_GRIP_POSITION)

        except socket.timeout as exception_e:
            rospy.logerr("socket timeout. check connection to UR5.")
        except Exception as exception_e:
            rospy.logerr("%s", exception_e)

        rate.sleep()


if __name__ == '__main__':
    MYARGV = rospy.myargv(argv=sys.argv)

    print("args I don't care about: ", MYARGV)
    try:
        talker_listener()
    except rospy.ROSInterruptException:
        PC_SOCK.close()
