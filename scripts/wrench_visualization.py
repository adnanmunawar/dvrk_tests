import rospy
from geometry_msgs.msg import WrenchStamped, Wrench, PoseStamped
from PyKDL import Frame, Rotation, Vector

wrench = WrenchStamped()
trans = Frame()


def wrench_cb(data):
    wrench.wrench = data


def pose_cb(data):
    global trans
    p = data.pose
    trans = Frame(Rotation.Quaternion(p.orientation.x,
                                      p.orientation.y,
                                      p.orientation.z,
                                      p.orientation.w),
                  Vector(p.position.x,
                         p.position.y,
                         p.position.z))


rospy.init_node('wrench_visualization')
w_sub = rospy.Subscriber('/dvrk/MTMR/set_wrench_body', Wrench, wrench_cb, queue_size=10)
p_sub = rospy.Subscriber('/dvrk/MTMR/position_cartesian_current', PoseStamped, pose_cb, queue_size=10)
pub = rospy.Publisher('/wrench_visualization/wrench', WrenchStamped, queue_size=10)

rate = rospy.Rate(100)

while not rospy.is_shutdown():
    if wrench is not None:
        force = Vector(wrench.wrench.force.x,
                       wrench.wrench.force.y,
                       wrench.wrench.force.z)

        body_force = trans.M.Inverse() * force
        body_wrench = WrenchStamped()
        body_wrench.header.frame_id = 'MTMR_wrist_roll_link'
        body_wrench.wrench.force.x = body_force[0]
        body_wrench.wrench.force.y = body_force[1]
        body_wrench.wrench.force.z = body_force[2]
        pub.publish(body_wrench)
    rate.sleep()

w_sub.unregister()
p_sub.unregister()
pub.unregister()
