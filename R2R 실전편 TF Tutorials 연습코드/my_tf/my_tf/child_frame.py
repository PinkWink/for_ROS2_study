import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler
import tf2_ros

class ChildTfBroadcaster(Node):
    def __init__(self):
        super().__init__('child_tf_broadcaster')
        
        # TransformBroadcaster 생성
        self.br = tf2_ros.TransformBroadcaster(self)
        
        # 타이머 설정(0.1초 간격)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # 각도 t를 0으로 초기화
        self.t = 0.0

    def timer_callback(self):
        # t를 조금씩 증가
        self.t += 0.05
        
        # 반지름 1m, 2배 빠른 각도(= 2 * t)
        angle = 2.0 * self.t
        
        # 부모 프레임인 moving_frame을 기준으로,
        # 반지름 1m 원을 그리면서 회전
        x = 1.0 * math.cos(angle)
        y = 1.0 * math.sin(angle)
        z = 0.0
        
        # (x,y)가 (0,0)에 대해 바라보는 방향(yaw) 계산
        # (원점 -> (x,y))의 반대방향: (x,y) -> (0,0) 는 (-x, -y)
        yaw = math.atan2(-y, -x)
        
        # 오일러 각 -> 쿼터니언 변환 (roll=0, pitch=0, yaw=계산값)
        q = quaternion_from_euler(0.0, 0.0, yaw)
        
        # TransformStamped 메시지 생성
        t = TransformStamped()
        
        # 현재 시간 정보
        t.header.stamp = self.get_clock().now().to_msg()
        
        # 부모 프레임과 자식 프레임 설정
        t.header.frame_id = 'moving_frame'
        t.child_frame_id = 'child_frame'
        
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
    node = ChildTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
