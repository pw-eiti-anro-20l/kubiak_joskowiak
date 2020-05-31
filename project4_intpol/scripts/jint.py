#!/usr/bin/env python

from project4_intpol.srv import jint_control, jint_clear
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from time import sleep
from sys import stderr
import copy
import rospy

x0_config = None
intpol_pub = None
end_effector_pose = None
end_effector_path_pub = None
end_effector_path = None

def linear_intpol(x0, x1, x_prev, t, T, time_step):
    return (x1 - x0)*t/T + x0

def quartic_vel_interpol(x0, x1, x_prev, t, T, time_step):
    a4 = 30*(x1 - x0)/(T**5)
    a3 = 60*(x0 - x1)/(T**4)
    a2 = 30*(x1 - x0)/(T**3)

    vel = a4*(t**4) + a3*(t**3) + a2*(t**2)

    print('current velocity: ', vel)

    return x_prev + vel*time_step

def check_limit(joint_config):

    param_names = ['d1', 'd2', 'd3']

    for i,pn in enumerate(param_names):
        di = rospy.get_param(pn)
        if joint_config[i] < -di or joint_config[i] > 0:
            print('Warn: {} only takes values between {} and {}'.format(pn, 0, -di))
            joint_config[i] = max(-di, joint_config[i])
            joint_config[i] = min(joint_config[i], 0)

    return joint_config

def check_acceleration(target_config, T, accel_limit):
    global x0_config

    # analytic solution
    for i in range(3):
        max_accel = abs((x0_config[i] - target_config[i]) * 5.7735/T**2)
        if max_accel > accel_limit:
            return False

    return True

def init_config(config):
    global intpol_pub
    js_msg = JointState()
    js_msg.name = ['joint1', 'joint2', 'joint3', 'tool_joint']
    js_msg.position = config

    runout = 3
    rate = rospy.Rate(1)
    while runout > 0:
        js_msg.header.stamp = rospy.get_rostime()
        intpol_pub.publish(js_msg)
        runout -= 1
        rate.sleep()

def handle_jint_control(req):
    global x0_config
    global intpol_pub
    global end_effector_pose
    global end_effector_path_pub
    global end_effector_path_msg


    x0_configl = copy.copy(x0_config)
    x1_config = check_limit([req.joint1, req.joint2, req.joint3, 0.0])
    print('target_config: ', x1_config)

    if req.interpolation_mode == 'quartic':
        intpol_fun = quartic_vel_interpol
        if not check_acceleration(x1_config, req.move_time, req.accel_limit):
            return 'acceleration limit will be exceeded'
    elif req.interpolation_mode == 'linear':
        intpol_fun = linear_intpol
    else:
        return ('no mode chosen/wrong mode name. available: quartic, linear' +
        'default: quartic')

    if req.move_time <= 0:
        return 'handling move_time <= 0 is not yet implemented'





    js_msg = JointState()
    js_msg.name = ['joint1', 'joint2', 'joint3', 'tool_joint']
    t = 0
    hz = 20
    dt = 1/hz
    rate = rospy.Rate(hz)
    intpol_config = list(copy.copy(x0_configl))

    while t < req.move_time:
        print('interp at {}s'.format(t))
        js_msg.header.stamp = rospy.get_rostime()

        for i in range(3):
            intpol_config[i] = intpol_fun(x0_configl[i], x1_config[i],
                                          intpol_config[i], t, req.move_time, dt)

        js_msg.position = intpol_config
        print('interpolated_config: ',intpol_config)

        intpol_pub.publish(js_msg)
        end_effector_path_msg.poses.append(end_effector_pose)

        rate.sleep()

        t += dt

    js_msg.header.stamp = rospy.get_rostime()
    end_effector_path_msg.header.stamp = rospy.get_rostime()
    end_effector_path_msg.header.frame_id = 'base_link'
    js_msg.position = x1_config
    intpol_pub.publish(js_msg)
    end_effector_path_pub.publish(end_effector_path_msg)

    return 'interpolation completed'

def handle_jint_clear(req):
    global end_effector_path_msg
    global end_effector_path_pub
    end_effector_path_msg = Path()

    end_effector_path_msg.header.stamp = rospy.get_rostime()
    end_effector_path_msg.header.frame_id = 'base_link'
    end_effector_path_pub.publish(end_effector_path_msg)


    return 'end effector path cleared'

def config_callback(js_msg):
    global x0_config
    x0_config = js_msg.position

def end_effector_pose_callback(ps_msg):
    global end_effector_pose
    end_effector_pose = ps_msg


def jint_init():
    global x0_config
    global intpol_pub
    global end_effector_pose
    global end_effector_path_pub
    global end_effector_path_msg


    x0_config = [0.0]*4
    end_effector_path_msg = Path()
    rospy.init_node('jint')
    intpol_pub = rospy.Publisher('/joint_states', JointState, queue_size=1)
    config_sub = rospy.Subscriber('/joint_states', JointState, config_callback)
    end_effector_pose_sub = rospy.Subscriber('/kdl_dkin/pose', PoseStamped,
                                                end_effector_pose_callback)
    end_effector_path_pub = rospy.Publisher('/jint/end_effector_path', Path,
                                                                queue_size=1)

    jint_control_srv = rospy.Service('jint_control_srv', jint_control,
                                    handle_jint_control)

    jint_clear_srv = rospy.Service('jint_clear_srv', jint_clear,
                                    handle_jint_clear)

    init_config([0.0]*4)
    print('jint ready')

    rospy.spin()

if __name__ == '__main__':
    jint_init()
