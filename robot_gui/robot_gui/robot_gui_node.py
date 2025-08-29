from rclpy.node import Node
from rover_interface.msg import Ackermann
from sensor_msgs.msg import Joy
from math import atan2, pi

SENSITIVITY_COEF = 3.0

class RobotGuiNode(Node):
    def __init__(self):
        super().__init__('robot_gui')
        self.publisher_ = self.create_publisher(Ackermann, '/robot/cmd_control', 10)
        self._subscription = self.create_subscription(Joy, '/robot/joy', self.sub_cb, 10)
        self.msg = Ackermann()
        self.ping_timer = None
        
        self.linear_vel_step = 20
        self.angle_step = 3

        self.update_tree = None

        self.button_animation_callback = None
        self.reset_msg()

    def set_button_animation_callback(self, cb):
        self.button_animation_callback = cb

    def set_update_tree(self, func):
        self.update_tree = func

    def set_angle_step(self, angle_step):
        self.angle_step = angle_step

    def set_vel_step(self, vel_step):
        self.linear_vel_step = vel_step

    def set_timer(self, period, callback):
        if self.ping_timer is not None:
            self.ping_timer.destroy()
        self.ping_timer = self.create_timer(period, callback)

    def animate_buttons(self, move = True):
        if not move:
            self.button_animation_callback('stop')
            return
        
        if self.msg.angle < 0:
            self.button_animation_callback('left')
        elif self.msg.angle > 0:
            self.button_animation_callback('right')

        if self.msg.linear_vel > 0:
            self.button_animation_callback('forward')
        elif self.msg.linear_vel < 0:
            self.button_animation_callback('backward')

    def sub_cb(self, msg_joy):
        """
        Обработчик сообщений Joy, приходящих от ноды джойстика 
        """
        self.get_logger().info("Getting command from joystick...")
        x = msg_joy.axes[3]
        y = msg_joy.axes[4] 

        direction = int(msg_joy.axes[1])
        stop = msg_joy.buttons[1]
        if stop:
            self.stop_moving()
            #self.animate_buttons(move = False)
        else:
            if (x !=0 or y != 0):
                y *= SENSITIVITY_COEF
                angle = int(round((atan2(-x, y)) * 180 / pi))
            else:
                angle = 0 #self.msg.angle

            self.msg.linear_vel += direction * self.linear_vel_step
            if self.msg.linear_vel > 100:
                 self.msg.linear_vel = 100
            elif self.msg.linear_vel < -25:
                 self.msg.linear_vel = -25
            
            if angle > 90:
                angle = 90
            elif angle < -90:
                angle = -90
        
            self.msg.angle = angle
            if self.update_tree:
                self.update_tree(self.msg)
            self.publisher_.publish(self.msg)
            #self.animate_buttons()
            self.get_logger().info('Sending command from joystick to move ...')
        #self.get_logger().info("Joystick callback time: %.2f ms" % duration)

    def reset_msg(self):
        """
        Cброс линейнонй скорости для остановки
        """
        self.msg.linear_vel = 0
        self.msg.angle = 0
        self.publisher_.publish(self.msg)

    def move_forward(self):
        """
        Увеличение линейной скорости на конкретный шаг
        """
        # self.reset_msg()
        self.msg.linear_vel += self.linear_vel_step
        self.msg.linear_vel = min(self.msg.linear_vel, 100)
        self.publisher_.publish(self.msg)
        self.get_logger().info('Sending command to move forward...')

    def move_backward(self):
        """
        Уменьшение линейно скорости на конкретный шаг
        """
        # self.reset_msg()
        self.msg.linear_vel -= self.linear_vel_step
        self.msg.linear_vel = max(self.msg.linear_vel, -100)
        self.publisher_.publish(self.msg)
        self.get_logger().info('Sending command to move backward...')
        
    def move_left(self):
        """
        Увеличение угла поворота рулевой рейки на конкретный шаг
        для движения налево
        """
        # self.reset_msg()
        self.msg.angle += self.angle_step
        self.publisher_.publish(self.msg)
        self.msg.angle = min(self.msg.angle, 90)
        self.get_logger().info('Sending command to move left...')

    def move_right(self):
        """
        Уменьшение угла поворота рулевой рейки на конкретный шаг
        для движения направо
        """
        # self.reset_msg()
        self.msg.angle -= self.angle_step
        self.msg.angle = max(self.msg.angle, -90)
        self.publisher_.publish(self.msg)
        self.get_logger().info('Sending command to move right...')

    def stop_moving(self):
        """
        Остановка движения
        """
        self.msg.linear_vel = 0
        self.publisher_.publish(self.msg)
        self.get_logger().info('Sending command to stop moving...')