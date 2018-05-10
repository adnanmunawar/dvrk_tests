import rospy
from geometry_msgs.msg import WrenchStamped, Wrench, PoseStamped, Pose
from sensor_msgs.msg import PointCloud
from PyKDL import Frame, Rotation, Vector
import numpy as np
import numpy.linalg as la

wrench = WrenchStamped()
trans = Frame()
pcloud = PointCloud()
p = Pose()


def pose_cb(data):
    global p
    p = data.pose
    trans = Frame(Rotation.Quaternion(p.orientation.x,
                                      p.orientation.y,
                                      p.orientation.z,
                                      p.orientation.w),
                  Vector(p.position.x,
                         p.position.y,
                         p.position.z))


def wrench_cb(data):
    global p
    wrench.wrench = data
    force_squared = la.norm([data.force.x, data.force.y, data.force.z])
    if force_squared > 0.01:
        pcloud.points.append(p.position)


rospy.init_node('wrench_visualization')
w_sub = rospy.Subscriber('/dvrk/MTMR/set_wrench_body', Wrench, wrench_cb, queue_size=10)
p_sub = rospy.Subscriber('/dvrk/MTMR/position_cartesian_current', PoseStamped, pose_cb, queue_size=10)
w_pub = rospy.Publisher('/wrench_visualization/wrench', WrenchStamped, queue_size=10)
pc_pub = rospy.Publisher('/wrench_visualization/point_cloud', PointCloud, queue_size=10)

rate = rospy.Rate(100)

while not rospy.is_shutdown():
    if wrench is not None:
        force = Vector(wrench.wrench.force.x,
                       wrench.wrench.force.y,
                       wrench.wrench.force.z)

        body_force = trans.M.Inverse() * force
        body_wrench = WrenchStamped()
        pcloud.header.frame_id = 'MTMR_top_panel'
        body_wrench.header.frame_id = 'MTMR_wrist_roll_link'
        body_wrench.wrench.force.x = body_force[0]
        body_wrench.wrench.force.y = body_force[1]
        body_wrench.wrench.force.z = body_force[2]
        w_pub.publish(body_wrench)
        pc_pub.publish(pcloud)
    rate.sleep()

w_sub.unregister()
p_sub.unregister()
w_pub.unregister()
pc_pub.unregister()
