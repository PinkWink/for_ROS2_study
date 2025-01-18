# sudo apt install ros-jazzy-tf-transformations

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler
import tf2_ros

class MyTfBroadcaster(Node):
    def __init__(self):
        super().__init__('my_tf_broadcaster')
        
        # TransformBroadcaster 생성
        self.br = tf2_ros.TransformBroadcaster(self)
        
        # 타이머 설정(0.1초 간격)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # 각도 t를 0으로 초기화
        self.t = 0.0

    def timer_callback(self):
        # t를 조금씩 증가
        self.t += 0.05
        
        # 반지름 2m인 원 위를 돌도록 x, y 계산
        x = 2.0 * math.cos(self.t)
        y = 2.0 * math.sin(self.t)
        z = 0.0
        
        # 원의 중심 (0,0)을 바라보도록 yaw(= heading) 계산
        # (x, y) -> (0,0)의 방향벡터는 (-x, -y)
        # atan2(-y, -x)를 써도 되고, atan2(y, x)에 π를 더해도 됩니다.
        yaw = math.atan2(-y, -x)
        
        # 오일러 각 -> 쿼터니언 변환 (roll=0, pitch=0, yaw=계산값)
        q = quaternion_from_euler(0.0, 0.0, yaw)
        
        # TransformStamped 메시지 생성
        t = TransformStamped()
        
        # 현재 시간 정보
        t.header.stamp = self.get_clock().now().to_msg()
        
        # 부모 프레임과 자식 프레임 설정
        t.header.frame_id = 'world'
        t.child_frame_id = 'moving_frame'
        
        # 위치 설정
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        
        # 자세(쿼터니언) 설정
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        # TF 브로드캐스트
        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = MyTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
