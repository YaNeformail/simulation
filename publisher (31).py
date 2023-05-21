import rospy
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from snar.msg import Num


# class Integ():
#     def __init__(self):
#         self.x = 0
#
#     def integrate(self, V):
#         self.x += V * 0.1
#         return self.x
#
#
# class Wheel():
#     def __init__(self):
#         self.b = 0.1 / (0.1 + 1)
#         self.x = 0
#
#     def integrate_wheels(self, w):
#         self.x = self.b * self.x + (1 - self.b) * w
#         return self.x


class MyPublisher():
    def __init__(self):

        self.odom = rospy.Publisher('odom', Odometry, queue_size=10)
        self.num = rospy.Publisher('topic', Num, queue_size=10)
        self.subscription = rospy.Subscriber('/cmd_vel', Twist, self.update)

        self.x = 0
        self.y = 0
        self.z = 0
        self.ang = 0
        self.wl = 0
        self.wr = 0
        
        # self.Integx = Integ()
        # self.Integy = Integ()
        # self.IntegO = Integ()
        # self.Wl = Wheel()
        # self.Wr = Wheel()
        # self.stepl = Integ()
        # self.stepr = Integ()

    def integrate(self, V):
        self.x += V * 0.1
        return self.x

    def integrate_wheels(self, w):
        b = 0.1 / (0.1 + 1)
        self.x = b * self.x + (1 - b) * w
        return self.x

    def update(self, msg):
        wlt = (2 * msg.linear.x / 0.033 - msg.angular.z * 0.287 / 0.033) / 2
        wrt = (2 * msg.linear.x / 0.033 + msg.angular.z * 0.287 / 0.033) / 2

        wl = self.integrate_wheels(wlt)
        wr = self.integrate_wheels(wrt)

        V = (0.033 / 2) * (wl + wr)
        ang = (0.033 / 0.287) * (wr - wl)

        x = self.integrate(math.cos(self.x) * V)
        y = self.integrate(math.sin(self.x) * V)
        z = self.integrate(ang)

        odometry = Odometry()
        odometry.header.frame_id = "odom"
        odometry.header.stamp = rospy.Time.now()
        odometry.pose.pose.orientation.w = float(math.cos(z / 2))
        odometry.pose.pose.orientation.z = float(math.sin(z / 2))
        odometry.pose.pose.position.x = float(x)
        odometry.pose.pose.position.y = float(y)
        self.odom.publish(odometry)

        num = Num()
        num.header.stamp = rospy.Time.now()
        num.header.frame_id = "step"
        num.l = int(((4096 / (3.14 * 2)) * ((self.integrate(wl)) % 3.14 * 2)))
        num.r = int(((4096 / (3.14 * 2)) * ((self.integrate(wr)) % 3.14 * 2)))
        self.num.publish(num)


def main():
    rospy.init_node('asdf')
    rospy.loginfo('Created a node')

    my_publisher = MyPublisher()

    rospy.spin()


if __name__ == '__main__':
    main()
