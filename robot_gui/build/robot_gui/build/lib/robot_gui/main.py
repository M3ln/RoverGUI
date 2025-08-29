#!/usr/bin/env python3
import rclpy
import sys
from PyQt5 import QtWidgets
from .robot_gui import Ui_robot_gui
from .robot_gui_node import RobotGuiNode
from rclpy.executors import MultiThreadedExecutor
from threading import Thread

class RobotGUI(Ui_robot_gui):
    def __init__(self):
        super().__init__()

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

def main(args=None):
    rclpy.init(args=args)
    node = RobotGuiNode()

    app = QtWidgets.QApplication(sys.argv)
    window = QtWidgets.QMainWindow()
    ui = RobotGUI()

    ui.setupUi(window)
    ui.set_callbacks({
        'forward' : node.move_forward,
        'backward' : node.move_backward,
        'left' : node.move_left,
        'right' : node.move_right,
        'stop' : node.stop_moving
    }, node.msg)
    ui.set_shortcuts()
    ui.show_tree()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    thread = Thread(target=executor.spin)
    thread.start()
    node.get_logger().info('Spinned ROS2 Node...')

    try:
        window.show()
        sys.exit(app.exec_())
    finally:
        node.get_logger().info('Shutting down ROS2 node...')
        node.destroy_node()
        executor.shutdown()

if __name__ == '__main__':
    main()
