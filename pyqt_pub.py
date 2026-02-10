
import sys
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Empty

from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QLineEdit,
    QComboBox, QSpinBox, QPushButton, QVBoxLayout,
    QHBoxLayout, QTableWidget, QTableWidgetItem
    )
from PyQt5.QtCore import QTimer

ITEM_LIST = [
    'original', 'almond', 'crunky', 'choco_filled', 'white_cookie'
]


class ItemPublisher(Node):
    def __init__(self):
        super().__init__('item_ui_publisher')
        self.item_pub = self.create_publisher(String, '/item_command', 10)
        self.stop_pub = self.create_publisher(Bool, '/robot_stop', 10)
        self.reset_pub = self.create_publisher(Empty, '/robot_reset', 10)
        self.sent = False

    def publish_items_once(self, items):
        if self.sent:
            return
        
        msg = String()
        msg.data = json.dumps(items)
        self.item_pub.publish(msg)
        self.sent = True
        self.get_logger().info(f'Item command sent.')

    def publish_stop(self):
        msg = Bool()
        msg.data = True
        self.stop_pub.publish(msg)
        self.get_logger().warn('STOP sent.')

    def reset(self):
        self.sent = False
        self.reset_pub.publish(Empty())
        self.get_logger().info('Robot/UI reset.')


class ItemInputUI(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.node = ros_node
        self.sent = False
        self.setWindowTitle("ROS2 작업 지시 UI")
        self.resize(460, 360)
        self.init_ui()

    def init_ui(self):
        # 입력부
        self.item_combo = QComboBox()
        self.item_combo.addItems(ITEM_LIST)
        self.count_input = QSpinBox()
        self.count_input.setRange(1, 10)

        self.add_btn = QPushButton("추가")
        self.add_btn.clicked.connect(self.add_item)

        input_layout = QHBoxLayout()
        input_layout.addWidget(QLabel('물건'))
        input_layout.addWidget(self.item_combo)
        input_layout.addWidget(QLabel('갯수'))
        input_layout.addWidget(self.count_input)
        input_layout.addWidget(self.add_btn)

        # 테이블
        self.table = QTableWidget(0, 2)
        self.table.setHorizontalHeaderLabels(['물건', '갯수'])
        self.table.horizontalHeader().setStretchLastSection(True)

        # 버튼
        self.start_btn = QPushButton("▶ 작업 시작")
        self.start_btn.clicked.connect(self.start_task)

        self.stop_btn = QPushButton("🛑 로봇 정지")
        self.stop_btn.setStyleSheet("background-color: red; color: white;")
        self.stop_btn.clicked.connect(self.stop_robot)

        self.restart_btn = QPushButton("🔄 재시작")
        self.restart_btn.setEnabled(False)
        self.restart_btn.clicked.connect(self.restart)

        btn_layout = QHBoxLayout()
        btn_layout.addWidget(self.start_btn)
        btn_layout.addWidget(self.stop_btn)
        btn_layout.addWidget(self.restart_btn)

        self.status_label = QLabel('상태: 대기 중')

        layout = QVBoxLayout()
        layout.addLayout(input_layout)
        layout.addWidget(self.table)
        layout.addLayout(btn_layout)
        layout.addWidget(self.status_label)

        self.setLayout(layout)

    def add_item(self):
        if self.sent:
            return
        
        name = self.item_combo.currentText()
        if not name:
            return
        
        row = self.table.rowCount()
        self.table.insertRow(row)
        self.table.setItem(row, 0, QTableWidgetItem(name))
        self.table.setItem(row, 1, QTableWidgetItem(str(self.count_input.value())))

        # self.item_input.clear()
        self.count_input.setValue(1)

    def get_items(self):
        return [
            {
                "name": self.table.item(r, 0).text(),
                "count": int(self.table.item(r, 1).text())
            }
            for r in range(self.table.rowCount())
        ]
    
    def start_task(self):
        if self.sent:
            return
        
        items = self.get_items()
        if not items:
            self.status_label.setText("상태: 물건 없음")
            return
        
        self.node.publish_items_once(items)
        self.sent = True

        # UI 잠금
        self.set_input_enabled(False)
        self.start_btn.setEnabled(False)
        self.restart_btn.setEnabled(True)

        self.status_label.setText("상태: 작업 진행 중")

    def stop_robot(self):
        self.node.publish_stop()
        self.status_label.setText("상태: 정지 신호 전송")

    def restart(self):
        # ROS / UI 상태 초기화
        self.node.reset()
        self.sent = False

        self.table.setRowCount(0)
        self.set_input_enabled(True)
        self.start_btn.setEnabled(True)
        self.restart_btn.setEnabled(False)

        self.status_label.setText("상태: 대기 중")

    def set_input_enabled(self, enabled):
        self.item_combo.setEnabled(enabled)
        self.count_input.setEnabled(enabled)
        self.add_btn.setEnabled(enabled)
        self.table.setEnabled(enabled)


def main():
    # ROS2 초기화
    rclpy.init()

    # ROS2 노드 생성
    node = ItemPublisher()

    # Qt 어플리케이션 생성
    app = QApplication(sys.argv)

    # UI 생성 (ROS 노드 전달)
    ui = ItemInputUI(node)
    ui.show()

    # ROS2 spin을 Qt 타이머로 처리
    ros_timer = QTimer()
    ros_timer.timeout.connect(
        lambda: rclpy.spin_once(node, timeout_sec=0.01)
    )
    ros_timer.start(10)  # 10ms

    # Qt 메인 루프 실행
    exit_code = app.exec_()

    # 종료 처리
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
