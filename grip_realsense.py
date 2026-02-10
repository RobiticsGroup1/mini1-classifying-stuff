import cv2
import time
import json
import numpy as np
from ultralytics import YOLO

import rclpy
import pyrealsense2 as rs
from rclpy.node import Node

import message_filters
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String, Bool

import DR_init
from gripper_drl_controller import GripperController


ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"
VELOCITY, ACC = 50, 50

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

object_dict = {"0": 'original', "1": 'almond', "2": 'crunky', "3": 'choco_filled', "4": 'white_cookie'}
object_dict_re = {'original': '0', 'almond': '1', 'crunky': '2', 'choco_filled': '3', 'white_cookie': '4'}

# Load YOLO model
model = YOLO('/home/been/camera_ws/training/runs/detect/train/weights/best.pt')


class RobotControllerNode(Node):
    def __init__(self):
        super().__init__("robot_controller_node")

        self.bridge = CvBridge()

        self.get_logger().info("ROS2 구독자 설정 시작")

        self.intrinsics = None
        self.latest_cv_color = None
        self.latest_cv_depth = None
        self.latest_cv_depth_mm = None
        self.latest_cv_vis = None

        self.state = 'IDLE'  # IDLE, RUNNING, STOPPED
        self.task_received = False
        self.items = None

        self.depth_scale = 0.001

        self.color_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/color/image_raw'
        )
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/aligned_depth_to_color/image_raw'
        )
        self.info_sub = message_filters.Subscriber(
            self, CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info'
        )

        self.item_sub = self.create_subscription(
            String, '/item_command', self.item_callback, 10
        )

        self.stop_sub = self.create_subscription(
            Bool, '/robot_stop', self.stop_callback, 10
        )

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub, self.info_sub], 
            queue_size=10, 
            slop=0.1
        )
        self.ts.registerCallback(self.synced_callback)

        self.get_logger().info("컬러/뎁스/카메라정보 토픽 구독 대기 중...")
        self.get_logger().info("화면이 나오지 않으면 Launch 명령어를 확인하세요.")

        self.gripper = None
        try:
            from DSR_ROBOT2 import wait
            self.gripper = GripperController(node=self, namespace=ROBOT_ID)
            wait(2)
            if not self.gripper.initialize():
                self.get_logger().error("Gripper initialization failed. Exiting.")
                raise Exception("Gripper initialization failed")
            self.get_logger().info("그리퍼를 활성화합니다...")
            self.gripper_is_open = True
            self.gripper.move(700)
            
        except Exception as e:
            self.get_logger().error(f"An error occurred during gripper setup: {e}")
            rclpy.shutdown()

        self.get_logger().info("RealSense ROS2 구독자와 로봇 컨트롤러가 초기화되었습니다.")

    def synced_callback(self, color_msg, depth_msg, info_msg):
        try:
            self.latest_cv_color = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            self.latest_cv_depth_mm = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")
        
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge 변환 오류: {e}")
            return

        if self.intrinsics is None:
            self.intrinsics = rs.intrinsics()
            self.intrinsics.width = info_msg.width
            self.intrinsics.height = info_msg.height
            self.intrinsics.ppx = info_msg.k[2]
            self.intrinsics.ppy = info_msg.k[5]
            self.intrinsics.fx = info_msg.k[0]
            self.intrinsics.fy = info_msg.k[4]
            
            if info_msg.distortion_model == 'plumb_bob' or info_msg.distortion_model == 'rational_polynomial':
                self.intrinsics.model = rs.distortion.brown_conrady
            else:
                self.intrinsics.model = rs.distortion.none
            
            self.intrinsics.coeffs = list(info_msg.d)
            self.get_logger().info("카메라 내장 파라미터(Intrinsics) 수신 완료.")

    def item_callback(self, msg: String):
        # 이미 작업 중이면 무시 (중복 방지)
        if self.task_received:
            self.get_logger().warn("Item command already received. Ignoring.")
            return
        
        try:
            self.items = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("Invalid JSON received.")
            return
        
        self.task_received = True
        self.state = 'RUNNING'

        self.get_logger().info(f"Received item command: {self.items}")

        items = self.items
        print(f"items : {items}")

        class_id = object_dict_re[items[0]['name']]
        count = items[0]['count']

        for i in range(count):
            from DSR_ROBOT2 import get_current_posx, movel, wait, movej
            from DR_common2 import posx, posj

            if self.latest_cv_depth_mm is None or self.intrinsics is None:
                self.get_logger().warn("아직 뎁스 프레임 또는 카메라 정보가 수신되지 않았습니다.")
                return
            
            print("초기 자세로 복귀합니다.")
            home_posj = posj(0, -45, 90, 0, 135, 0)
            movej(home_posj, vel=40, acc=40)
            self.gripper.move(0)
            wait(2)
            
            try:
                img = self.latest_cv_color.copy()

                # masking
                img[200:600, 0:300] = (0, 0, 0)

                self.latest_cv_vis = self.latest_cv_color.copy()
                depth_img = self.latest_cv_depth_mm.copy()
                object_loc_dict = {'0': [], '1': [], '2': [], '3': [], '4': []}
                
                # Detect objects using YOLO
                results = model(img)

                # Process the results
                for result in results:
                    boxes = result.boxes
                    for box in boxes:
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype('int')
                        confidence = box.conf[0].cpu().numpy()
                        cls_id = str(int(box.cls[0].cpu().numpy()))

                        if confidence < 0.5:
                            continue

                        # Calculate the distance to the object
                        object_depth = np.median(depth_img[y1:y2, x1:x2])  # mm
                        label= f"{object_dict[cls_id]} / {object_depth*0.001:.2f}m"
                        object_loc_dict[str(int(cls_id))] = [int((x1+x2)/2), int((y1+y2)/2), object_depth]
                        
                        # Draw a rectangle around the object
                        cv2.rectangle(self.latest_cv_vis, (x1, y1), (x2, y2), (252, 119, 30), 2)

                        # Draw the bounding box
                        cv2.putText(self.latest_cv_vis, label, (x1, y1-10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (252, 119, 30), 2)

                depth_mm = object_loc_dict[class_id][-1]

            except:
                self.get_logger().warn("Detection Error")
                return
            
            if depth_mm == 0:
                print(f"해당 {model.names[class_id]}의 깊이를 측정할 수 없습니다 (값: 0).")
                return
            
            depth_m = float(depth_mm) / 1000.0
            u = object_loc_dict[class_id][0]
            v = object_loc_dict[class_id][1]

            point_3d = rs.rs2_deproject_pixel_to_point(self.intrinsics, [u, v], depth_m)
            
            x_mm = point_3d[1] * 1000
            y_mm = point_3d[0] * 1000
            z_mm = point_3d[2] * 1000

            # transform camera to base
            final_x = x_mm + 510  # 620
            final_y = y_mm + 175  # 0
            final_z = 990 - z_mm  # 950
            if (final_z <= 110):
                final_z = 110

            print("--- 변환된 최종 3D 좌표 ---")
            print(f"픽셀 좌표: (u={u}, v={v}), Depth: {depth_m*1000:.1f} mm")
            print(f"로봇 목표 좌표: X={final_x:.1f}, Y={final_y:.1f}, Z={final_z:.1f}\n")

            self.move_robot_and_control_gripper(final_x, final_y, final_z)
            print("=" * 50)

    def stop_camera(self):
        pass

    def stop_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().warn("STOP signal received!")
            self.state = 'STOPPED'
            self.emergency_stop()

    def terminate_gripper(self):
        if self.gripper:
            self.gripper.terminate()
    
    def move_robot_and_control_gripper(self, x, y, z):
        from DSR_ROBOT2 import get_current_posx, movel, wait, movej
        from DR_common2 import posx, posj
        try:
            current_pos = get_current_posx()[0]
            target_pos_list_up = [x, y, z + 100, current_pos[3], current_pos[4], current_pos[5]]
            target_pos_list = [x, y, z, current_pos[3], current_pos[4], current_pos[5]]
            p_start = posj(0, -45, 90, 0, 135, 0)

            movel(posx(target_pos_list_up), vel=80, acc=ACC)
            wait(0.5)

            self.get_logger().info(f"목표 지점으로 이동합니다: {target_pos_list}")
            movel(posx(target_pos_list), vel=80, acc=ACC)
            wait(0.5)

            # gripper 물건 잡기
            self.gripper.move(700)
            wait(4)

            # 들어올리기
            movel(posx(target_pos_list_up), vel=VELOCITY, acc=ACC)
            wait(1)

            # 물건을 놓는 지점으로 이동
            movel(posx(536.9, -309.5, target_pos_list_up[2], current_pos[3], current_pos[4], current_pos[5]), vel=80, acc=70)
            wait(0.5)

            movel(posx(536.9, -309.5, 150.0, current_pos[3], current_pos[4], current_pos[5]), vel=80, acc=70)
            wait(0.5)

            # gripper 물건 놓기
            self.gripper.move(0)
            wait(4)

            self.get_logger().info("초기 자세로 복귀합니다.")
            movej(p_start, VELOCITY, ACC)

        except Exception as e:
            self.get_logger().error(f"로봇 이동 및 그리퍼 제어 중 오류 발생: {e}")


def main(args=None):
    rclpy.init(args=args)

    dsr_node = rclpy.create_node("dsr_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = dsr_node

    try:
        from DSR_ROBOT2 import get_current_posx, movel, wait, movej
        from DR_common2 import posx, posj
    except ImportError as e:
        print(f"DSR_ROBOT2 라이브러리를 임포트할 수 없습니다: {e}")
        rclpy.shutdown()
        exit(1)

    robot_controller = RobotControllerNode()

    rclpy.spin(robot_controller)

    print("프로그램을 종료합니다")
    robot_controller.terminate_gripper()
    robot_controller.destroy_node()
    dsr_node.destroy_node()
    rclpy.shutdown()
    print("종료 완료")


if __name__ == "__main__":
    main()
