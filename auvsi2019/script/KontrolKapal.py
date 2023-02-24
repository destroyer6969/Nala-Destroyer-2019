#!/usr/bin/env python
from Mision import Mision
from auvsi2019.msg import MissionAttr
import rospy


misi1 = Mision()
misi1.changeStop(5)
misi1.changeSpeed(1788)
misi1.changeWaypoint(11)

def kontrolKapal():
    pub = rospy.Publisher('misi1', MissionAttr)
    rospy.init_node('kontrolKapal', anonymous=False)
    rate = rospy.Rate(10)
    msg = MissionAttr()
    msg.speed = misi1.speed
    msg.stop = misi1.stop
    msg.waypoint = misi1.waypoint

    while not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep




if __name__ == '__main__':
    try:
        kontrolKapal()
    except rospy.ROSInterruptException:
        pass

