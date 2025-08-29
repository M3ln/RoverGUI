from rclpy.node import Node
from rover_interface.msg import Ackermann
from sensor_msgs.msg import Joy
from math import atan2, pi

VELOCITY_STEP = 40
ANGLE_STEP = 3
DISTANCE_BETWEEN_AXES = 0.25

class RobotGuiNode(Node):
    def __init__(self):
        super().__init__('robot_gui')
        self.publisher_ = self.create_publisher(Ackermann, 'robot/cmd_control', 10)
        self._subscription = self.create_subscription(Joy, 'robot/joy', self.sub_cb, 10)
        self.msg = Ackermann()
        self.reset_msg()

    def sub_cb(self, msg_joy):
        """
        Обработчик сообщений Joy, приходящих от ноды джойстика 
        """
        x = msg_joy.axes[3]
        y = msg_joy.axes[4]

        if x !=0 or y != 0:
            angle = int(round(atan2(x, y) * 180 / pi))
        else:
            angle = self.msg.linear_vel

        direction = int(msg_joy.axes[1])
        stop = msg_joy.buttons[1]
        if stop:
            self.stop_moving()
        else:
            self.msg.linear_vel += direction * VELOCITY_STEP

            if self.msg.linear_vel > 40:
                self.msg.linear_vel = 40
            elif self.msg.linear_vel < -40:
                self.msg.linear_vel = -40

            if angle > 90:
                angle = 90
            elif angle < -90:
                angle = -90

            self.msg.angle = angle
            self.publisher_.publish(self.msg)
            self.get_logger().info('Sending command from joystick to move ...')

    def reset_msg(self):
        """
        Cброс линейнонй скорости для остановки
        """
        self.msg.linear_vel = 0

    def move_forward(self):
        """
        Увеличение линейной скорости на конкретный шаг
        """
        # self.reset_msg()
        self.msg.linear_vel += VELOCITY_STEP
        self.msg.linear_vel = min(self.msg.linear_vel, 40)
        self.publisher_.publish(self.msg)
        self.get_logger().info('Sending command to move forward...')

    def move_backward(self):
        """
        Уменьшение линейно скорости на конкретный шаг
        """
        # self.reset_msg()
        self.msg.linear_vel -= VELOCITY_STEP
        self.msg.linear_vel = max(self.msg.linear_vel, -40)
        self.publisher_.publish(self.msg)
        self.get_logger().info('Sending command to move backward...')
        
    def move_left(self):
        """
        Увеличение угла поворота рулевой рейки на конкретный шаг
        для движения налево
        """
        # self.reset_msg()
        self.msg.angle += ANGLE_STEP
        self.publisher_.publish(self.msg)
        self.msg.angle = min(self.msg.angle, 90)
        self.get_logger().info('Sending command to move left...')

    def move_right(self):
        """
        Уменьшение угла поворота рулевой рейки на конкретный шаг
        для движения направо
        """
        # self.reset_msg()
        self.msg.angle -= ANGLE_STEP
        self.msg.angle = max(self.msg.angle, -90)
        self.publisher_.publish(self.msg)
        self.get_logger().info('Sending command to move right...')

    def rotate_clockwise(self):
        pass

    def rotate_counterclockwise(self):
        pass

    def stop_moving(self):
        """
        Остановка движения
        """
        self.reset_msg()
        self.publisher_.publish(self.msg)
        self.get_logger().info('Sending command to stop moving...')