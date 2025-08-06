# 일단 테스트용
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int32
from std_srvs.srv import Trigger, TriggerRequest  
from your_pkg.srv import ThermalWorker, ThermalWorkerRequest
from your_pkg.srv import MicWorker, MicWorkerRequest
from your_pkg.srv import SiliconeWorker, SiliconeWorkerRequest

class ModeSelector(object):
    def __init__(self):
        self.node_name = 'mode_selector'
        rospy.init_node(self.node_name)

       
        self.mode = 0

        
        self.mode_pub = rospy.Publisher('/current_mode', Int32, queue_size=10)

        
        self.thermal_cli  = rospy.ServiceProxy('/thermal_worker',  ThermalWorker)
        self.mic_cli      = rospy.ServiceProxy('/mic_worker',      MicWorker)
        self.silicone_cli = rospy.ServiceProxy('/silicone_worker', SiliconeWorker)

        # 주기 실행(1초)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.run_state_machine)

        rospy.loginfo("[%s] started.", self.node_name)

    def run_state_machine(self, event):
        # 현재 모드 퍼블리시
        self.mode_pub.publish(self.mode)

        try:
            if self.mode == 0:
                rospy.loginfo_throttle(2.0, "Mode 0: 라인트레이싱 중... (thermal check)")
                # 서비스 대기(짧게)
                rospy.wait_for_service('/thermal_worker', timeout=0.5)
                resp = self.thermal_cli(ThermalWorkerRequest())
                if getattr(resp, 'abnormal', False):
                    self.mode = 1

            elif self.mode == 1:
                rospy.loginfo_throttle(2.0, "Mode 1: 타겟 정렬 후 mic 분석")
                rospy.wait_for_service('/mic_worker', timeout=0.5)
                resp = self.mic_cli(MicWorkerRequest())
                # 예: result 필드가 'abnormal' 이면 2로, 아니면 0으로
                if getattr(resp, 'result', '') == 'abnormal':
                    self.mode = 2
                else:
                    self.mode = 0

            elif self.mode == 2:
                rospy.loginfo_throttle(2.0, "Mode 2: 이미지 캡처 후 실리콘 작업")
                rospy.wait_for_service('/silicone_worker', timeout=0.5)
                resp = self.silicone_cli(SiliconeWorkerRequest())
                if getattr(resp, 'success', False):
                    self.mode = 0

        except rospy.ROSException as e:
            # 서비스가 준비 안 된 경우 등등
            rospy.logwarn_throttle(2.0, "Service not available yet: %s", str(e))
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", str(e))
        except Exception as e:
            rospy.logerr("Unexpected error: %s", str(e))

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    node = ModeSelector()
    node.spin()

