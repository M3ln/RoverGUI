#!/usr/bin/env python3
import rclpy
import sys
import os
from PyQt5 import QtWidgets, QtCore, QtGui
from .robot_gui import Ui_robot_gui
from .robot_gui_node import RobotGuiNode
from rclpy.executors import MultiThreadedExecutor
from threading import Thread
from ping3 import ping
import json

class RobotGUI(Ui_robot_gui):
    def __init__(self, window):
        super().__init__()
        self.seq_ping = 0
        self.json_path = "/home/%s/robot_ws/src/robot_gui/resource/robots.json" % os.environ['USER']
        self.robots_data = {}

        self.setupUi(window)
        self.load_robots_data()
        self.angle_step_slider.setRange(0, 30)
        self.vel_step_slider.setRange(0, 20)
        self.vel_step_slider.setValue(20)
        self.angle_step_slider.setValue(3)

        # self.effect = QtWidgets.QGraphicsColorizeEffect(self.button_forward)
        # self.button_forward.setGraphicsEffect(self.effect)

        self.button_animations = {
            'forward': QtCore.QPropertyAnimation(self.button_forward, b"color"),
            'backward': QtCore.QPropertyAnimation(self.button_backward, b"color"),
            'left': QtCore.QPropertyAnimation(self.button_left, b"color"),
            'right': QtCore.QPropertyAnimation(self.button_right, b"color"),
            'stop': QtCore.QPropertyAnimation(self.button_stop, b"color")
        }

    def update_color(self, color):
        self.button_forward.setStyleSheet(f"""
            QPushButton {{
                background-color: {color.name()};
                color: white;
                border: none;
                font-size: 16px;
            }}
        """)

    def animate_button(self, button_name):
        if button_name in self.button_animations.keys():
            button_animation = self.button_animations[button_name]
            button_animation.setDuration(100)
            button_animation.setStartValue(QtGui.QColor("#4CAF50"))
            button_animation.setEndValue(QtGui.QColor("#F44336"))
            button_animation.valueChanged.connect(self.update_color)
            button_animation.start()

    def set_angle_step_callback(self, func):
        self.angle_step_slider.valueChanged.connect(lambda: self.set_angle_step(func))

    def set_vel_step_callback(self, func):
        self.vel_step_slider.valueChanged.connect(lambda: self.set_vel_step(func))

    def set_vel_step(self, func):
        func(self.vel_step_slider.value())
        self.command_info.topLevelItem(2).child(0).setText(1, str(
            self.vel_step_slider.value()
        ))

    def set_angle_step(self, func):
        func(self.angle_step_slider.value())
        self.command_info.topLevelItem(3).child(0).setText(1, str(
            self.angle_step_slider.value()
        ))

    def load_robots_data(self):
        with open(self.json_path, 'r') as file:
            self.robots_data = json.load(file)
        
        for robot in self.robots_data['robots']:
            self.robot_cb.addItem(robot['name'])

    def get_robot_ip(self):
        robot_name = self.robot_cb.currentText()
        for robot in self.robots_data['robots']:
            if robot['name'] == robot_name:
                return robot['ip']
        return None 
    
    def set_ping(self, msg):
        self.ping_output.setText(msg)

    def set_shortcuts(self):
        self.button_forward.setShortcut('W')
        self.button_backward.setShortcut('S')
        self.button_left.setShortcut('D')
        self.button_right.setShortcut('A')
        # self.button_cwrot.setShortcut('E')
        # self.button_counter_cwrot.setShortcut('Q')
        self.button_stop.setShortcut('Space')

    def show_tree(self):
        self.command_info.topLevelItem(0).setExpanded(True)
        self.command_info.topLevelItem(1).setExpanded(True)
        self.command_info.topLevelItem(2).setExpanded(True)
        self.command_info.topLevelItem(3).setExpanded(True)
        self.command_info.topLevelItem(2).child(0).setText(1, str(
            self.vel_step_slider.value()
        ))
        self.command_info.topLevelItem(3).child(0).setText(1, str(
            self.angle_step_slider.value()
        ))
        self.command_info.resizeColumnToContents(0)

    def set_callbacks(self, moves, msg):
        self.button_forward.clicked.connect(lambda: self.move(moves['forward'], msg))
        self.button_backward.clicked.connect(lambda: self.move(moves['backward'], msg))
        self.button_left.clicked.connect(lambda: self.move(moves['left'], msg))
        self.button_right.clicked.connect(lambda: self.move(moves['right'], msg))
        # self.button_cwrot.clicked.connect(lambda: self.move(moves[4], msg))
        # self.button_counter_cwrot.clicked.connect(lambda: self.move(moves[5], msg))
        self.button_stop.clicked.connect(lambda: self.move(moves['stop'], msg))

    def move(self, func, msg):
        func()
        self.update_tree(msg)  

    def update_tree(self, msg):
        #self.command_info.topLevelItem(0).child(0).setText(0, _translate("robot_gui", "z"))
        self.command_info.topLevelItem(0).child(0).setText(1, str(msg.linear_vel))
        #self.command_info.topLevelItem(1).child(0).setText(0, _translate("robot_gui", "z"))
        self.command_info.topLevelItem(1).child(0).setText(1, str(msg.angle))

    def ping_cb(self):
        ip = self.get_robot_ip()
    
        if not ip:
            return

        print("Connecting to " + ip + "...")        
        response = ping(
            self.get_robot_ip(),
            timeout = 0.4,
            unit = 'ms',
            size = 56,
            seq = self.seq_ping
        )

        if response:
            self.set_ping("%.2f ms" % response)
        else:
            self.set_ping("Нет соединения!")

def main(args=None):
    # Инициализация ROS2
    rclpy.init(args=args)
    
    try:
        # Создаем ROS2-ноду
        node = RobotGuiNode()
        node.get_logger().info('ROS2-нода создана')

        # Настраиваем Qt-приложение
        app = QtWidgets.QApplication(sys.argv)
        window = QtWidgets.QMainWindow()
        ui = RobotGUI(window)

        # Устанавливаем колбэки для кнопок управления
        ui.set_callbacks({
            'forward': node.move_forward,    # Вперед
            'backward': node.move_backward,  # Назад
            'left': node.move_left,         # Влево
            'right': node.move_right,       # Вправо
            'stop': node.stop_moving         # Стоп
        }, node.msg)
        
        ui.set_shortcuts()                  # Горячие клавиши
        ui.show_tree()                      # Отображаем дерево (если есть)
        ui.set_angle_step_callback(node.set_angle_step)  # Угол поворота
        ui.set_vel_step_callback(node.set_vel_step)     # Скорость
        node.set_update_tree(ui.update_tree)
        node.set_button_animation_callback(ui.animate_button)

        # Запускаем ROS2-исполнитель в отдельном потоке
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        ros_thread = Thread(target=executor.spin, daemon=True)  # Демон-поток (завершится с программой)
        ros_thread.start()
        node.get_logger().info('ROS2-исполнитель запущен в фоне')

        # 🔥 Важно: QTimer вместо threading.Timer
        ping_timer = QtCore.QTimer()
        ping_timer.timeout.connect(ui.ping_cb)  # Таймер будет вызывать ping_cb
        ping_timer.start(1000)  # Интервал в мс (1000 мс = 1 сек)

        # Запускаем графический интерфейс
        window.show()
        node.get_logger().info('Графический интерфейс запущен')
        
        exit_code = app.exec_()  # Основной цикл Qt
        
    except Exception as e:
        # Логируем критические ошибки
        node.get_logger().fatal(f'Ошибка: {str(e)}')
        exit_code = 1
    finally:
        # Корректное завершение работы
        node.get_logger().info('Завершение работы...')
        
        if 'ping_timer' in locals():
            ping_timer.stop()  # Останавливаем таймер
        
        if 'node' in locals():
            node.destroy_node()   # Уничтожаем ноду
        
        if 'executor' in locals():
            executor.shutdown()   # Останавливаем исполнитель ROS2
        
        rclpy.shutdown()         # Завершаем работу ROS2
    
    sys.exit(exit_code)          # Выход с кодом завершения


if __name__ == '__main__':
    main()