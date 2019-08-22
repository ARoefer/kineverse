import rospy
import time

class Time(object):
    def __new__(cls, secs=0, nsecs=0):
        return rospy.Time(secs, nsecs)

    @classmethod
    def now(cls):
        if rospy.rostime.is_rostime_initialized():
            return rospy.Time.now()
        return rospy.Time(time.time())