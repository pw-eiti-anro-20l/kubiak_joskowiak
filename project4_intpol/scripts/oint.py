#!/usr/bin/env python

from project4_intpol.srv import oint_control, oint_clear
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
from time import sleep
from sys import stderr
import copy
import rospy

coordinate_system1_pose = None
intpol_pub = None
coordinate_system1_path_pub = None
coordinate_system1_path_msg = None

def linear_intpol(x0, x1, x_prev, t, T, time_step):
    return (x1 - x0)*t/T + x0

def quartic_vel_interpol(x0, x1, x_prev, t, T, time_step):
    a4 = 30*(x1 - x0)/(T**5)
    a3 = 60*(x0 - x1)/(T**4)
    a2 = 30*(x1 - x0)/(T**3)

    vel = a4*(t**4) + a3*(t**3) + a2*(t**2)

    print('current velocity: ', vel)

    return x_prev + vel*time_step


def init_config(position, orientation):
    global intpol_pub
    ps_msg = PoseStamped()
    ps_msg.header.frame_id = "base_link"
    ps_msg.pose.position.x = position[0]
    ps_msg.pose.position.y = position[1]
    ps_msg.pose.position.z = position[2]

    ps_msg.pose.orientation.x = orientation[0]
    ps_msg.pose.orientation.y = orientation[1]
    ps_msg.pose.orientation.w = orientation[2]
    ps_msg.pose.orientation.z = orientation[3]


    runout = 3
    rate = rospy.Rate(1)
    while runout > 0:
        ps_msg.header.stamp = rospy.get_rostime()
        intpol_pub.publish(ps_msg)
        runout -= 1
        rate.sleep()

def normalize(q):
    l = sum([qi**2 for qi in q])
    q = [qi/l for qi in q]
    return q

def handle_oint_control(req):
    global coordinate_system1_pose
    global intpol_pub
    global coordinate_system1_path_pub
    global coordinate_system1_path_msg


    coordinate_system1_posel = copy.copy(coordinate_system1_pose)

    if req.interpolation_mode == 'quartic':
        intpol_fun = quartic_vel_interpol
    elif req.interpolation_mode == 'linear':
        intpol_fun = linear_intpol
    else:
        print('no mode chosen/wrong mode name. available: quartic, linear' +
        'default: quartic')

    if req.move_time <= 0:
        # print('handling move_time <= 0 is not yet implemented')
        return 'handling move_time <= 0 is not yet implemented'

    target_quaternion = quaternion_from_euler(req.roll, req.pitch, req.yaw)

    coordinate_system1_target_pose = [req.x, req.y, req.z]
    coordinate_system1_target_pose.extend(target_quaternion)


    t = 0
    hz = 50
    dt = 1.0/hz
    rate = rospy.Rate(hz)
    intpol_config = copy.copy(coordinate_system1_posel)

    while t < req.move_time:
        print('interp at {}s'.format(t))
        ps_msg = PoseStamped()
        ps_msg.header.frame_id = 'base_link'
        ps_msg.header.stamp = rospy.get_rostime()

        for i in range(7):
            intpol_config[i] = intpol_fun(coordinate_system1_posel[i],
                        coordinate_system1_target_pose[i], intpol_config[i],
                        t, req.move_time, dt)
            print('x1: ', coordinate_system1_target_pose)

        # orientation = normalize(intpol_config[3:7])
        # for i in range(4):
        #     intpol_config[i+3] = orientation[i]

        ps_msg.pose.position.x = intpol_config[0]
        ps_msg.pose.position.y = intpol_config[1]
        ps_msg.pose.position.z = intpol_config[2]
        ps_msg.pose.orientation.x = intpol_config[3]
        ps_msg.pose.orientation.y = intpol_config[4]
        ps_msg.pose.orientation.z = intpol_config[5]
        ps_msg.pose.orientation.w = intpol_config[6]

        print('interpolated_config: ',intpol_config)

        coordinate_system1_path_msg.poses.append(ps_msg)
        intpol_pub.publish(ps_msg)

        t += dt
        rate.sleep()

    ps_msg = PoseStamped()
    ps_msg.header.stamp = rospy.get_rostime()
    ps_msg.header.frame_id = 'base_link'
    coordinate_system1_path_msg.header.stamp = rospy.get_rostime()
    coordinate_system1_path_msg.header.frame_id = 'base_link'

    ps_msg.pose.position.x = coordinate_system1_target_pose[0]
    ps_msg.pose.position.y = coordinate_system1_target_pose[1]
    ps_msg.pose.position.z = coordinate_system1_target_pose[2]
    ps_msg.pose.orientation.x = coordinate_system1_target_pose[3]
    ps_msg.pose.orientation.y = coordinate_system1_target_pose[4]
    ps_msg.pose.orientation.z = coordinate_system1_target_pose[5]
    ps_msg.pose.orientation.w = coordinate_system1_target_pose[6]
    intpol_pub.publish(ps_msg)
    # print(coordinate_system1_path_msg.poses)
    coordinate_system1_path_pub.publish(coordinate_system1_path_msg)

    return 'interpolation completed'

def handle_oint_clear(req):
    global coordinate_system1_path_msg
    global coordinate_system1_path_pub
    coordinate_system1_path_msg = Path()

    coordinate_system1_path_msg.header.stamp = rospy.get_rostime()
    coordinate_system1_path_msg.header.frame_id = 'base_link'
    coordinate_system1_path_pub.publish(coordinate_system1_path_msg)
    return 'end effector path cleared'

def coordinate_system1_pose_callback(ps_msg):
    global coordinate_system1_pose
    coordinate_system1_pose = [
        ps_msg.pose.position.x,
        ps_msg.pose.position.y,
        ps_msg.pose.position.z,
        ps_msg.pose.orientation.x,
        ps_msg.pose.orientation.y,
        ps_msg.pose.orientation.z,
        ps_msg.pose.orientation.w
        ]



def oint_init():
    global coordinate_system1_pose
    global intpol_pub
    global coordinate_system1_path_pub
    global coordinate_system1_path_msg

    coordinate_system1_path_msg = Path()
    rospy.init_node('oint')
    intpol_pub = rospy.Publisher('/oint/coordinate_system1_pose', PoseStamped, queue_size=1)
    config_sub = rospy.Subscriber('/oint/coordinate_system1_pose', PoseStamped,
                                    coordinate_system1_pose_callback)

    coordinate_system1_path_pub = rospy.Publisher('/oint/coordinate_system1_path', Path,
                                                                queue_size=1)

    oint_control_srv = rospy.Service('oint_control_srv', oint_control,
                                    handle_oint_control)

    oint_clear_srv = rospy.Service('oint_clear_srv', oint_clear,
                                    handle_oint_clear)

    init_config([0]*3, [0, 1, 0, 0])
    print('oint ready')

    rospy.spin()

if __name__ == '__main__':
    oint_init()
