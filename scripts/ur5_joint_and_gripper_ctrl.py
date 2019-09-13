#!/usr/bin/env python
""" module docstring, yo! """

import sys
import socket
import rospy
from sensor_msgs.msg import JointState

HOST = ''
PORT = 30000
SOCK = socket.socket(socket.AF_INET,
                     socket.SOCK_STREAM)

SOCK.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
SOCK.bind((HOST, PORT)) # Bind to the port 
SOCK.listen(5) # Now wait for client connection.
SOCK_ACCEPTED, addr = SOCK.accept() # Establish connection with client.
print SOCK_ACCEPTED
print addr

def talker():
    """ docsctring, yo! """
    
    joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=1)
    rospy.init_node('ur5_joint_state', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        try:
            SOCK_ACCEPTED.send("(2)")
            data = SOCK_ACCEPTED.recv(1024)
            print data
            print "Received", repr(data)
            msg = JointState()
            msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
            
            joint_state_pub.publish(msg)
        except Exception as exception_e:
            rospy.logerr("%s", exception_e)
        rate.sleep()

if __name__ == '__main__':
    MYARGV = rospy.myargv(argv=sys.argv)

    print("args I don't care about: ", MYARGV)
    try:
        talker()
    except rospy.ROSInterruptException:
        SOCK.close()
