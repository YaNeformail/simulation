#!/usr/bin/python
import rospy
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from snar.msg import Num
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class MyPublisher():
    def __init__(self):
        # инициализация публикаторов и подписчика
        self.odom = rospy.Publisher('odom', Odometry, queue_size=10)
        self.num = rospy.Publisher('topic', Num, queue_size=10)
        self.path = rospy.Publisher('path', Path, queue_size=10)
        self.subscription = rospy.Subscriber('/cmd_vel', Twist, self.update)

        # инициализация начальных условий
        self.x = 0
        self.y = 0
        self.z = 0
        self.ang_speed_l = 0
        self.ang_speed_r = 0
        self.ang_step_l = 0
        self.ang_step_r = 0

    # реализация интегрирования
    def integration(self, value, linearSpeed):
        temp = getattr(self, value) + linearSpeed * 0.1
        setattr(self, value, temp)
        return temp

    # закон изменения величины от предыдущего значения к нынешнему
    def transitProcess(self, value, ang_speed):
        temp = getattr(self, value) * 0.0909 + (1 - 0.0909) * ang_speed
        setattr(self, value, temp)
        return temp

    # основной метод преобразования данных
    def update(self, msg):
        # входные воздействия
        w_left_t = (2 * msg.linear.x / 0.033 - msg.angular.z * 8.697) / 2
        w_right_t = (2 * msg.linear.x / 0.033 + msg.angular.z * 8.697) / 2

        # угловые скорости
        w_left = self.transitProcess("ang_speed_l", w_left_t)
        w_right = self.transitProcess("ang_speed_r", w_right_t)

        # линейная и угловая скорости робота соответственно
        V = 0.0165 * (w_left + w_right)
        angular = 0.115 * (w_right - w_left)

        # положения робота по каждой из осей
        x = self.integration("x", math.cos(self.z) * V)
        y = self.integration("y", math.sin(self.z) * V)
        z = self.integration("z", angular)

        # публикация данных одометрии
        odometry = Odometry()
        odometry.header.frame_id = "odom"
        odometry.header.stamp = rospy.Time.now()
        odometry.pose.pose.orientation.w = float(math.cos(z / 2))
        odometry.pose.pose.orientation.z = float(math.sin(z / 2))
        odometry.pose.pose.position.x = float(x)
        odometry.pose.pose.position.y = float(y)
        self.odom.publish(odometry)

        # публикация данных пути
        path = Path()
        path.header.frame_id = "base_link"
        path.header.stamp = rospy.Time.now()
        pose = PoseStamped()
        pose.header.frame_id = "odom"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position = odometry.pose.pose.position
        pose.pose.orientation = odometry.pose.pose.orientation
        path.poses.append(pose)
        self.path.publish(path)

        # публикация пользовательских сообщений о показаниях энкодеров
        num = Num()
        num.header.stamp = rospy.Time.now()
        num.header.frame_id = "step"
        num.l = int(652.229 * (self.integration("ang_step_l", w_left) % 3.14 * 2))
        num.r = int(652.229 * (self.integration("ang_step_r", w_right) % 3.14 * 2))
        self.num.publish(num)

# основная функция, использует основной класс и создает узел
def main():
    rospy.init_node('asdf')
    rospy.loginfo('Created a node')

    my_publisher = MyPublisher()

    rospy.spin()


if __name__ == '__main__':
    main()
