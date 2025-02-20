import rclpy
from rclpy.node import Node
from kk_driver_msg.msg import PwmCmd  # カスタムメッセージのインポート
import threading



class PwmCmdPublisher(Node):
    def __init__(self):
        super().__init__('strider_adjust_node')
        self.publisher_ = self.create_publisher(PwmCmd, '/pwm/cmd', 8)
        # self.timer = self.create_timer(0.1, self.publish_pwm_cmd)
        self.get_logger().info('PWM Cmd Publisher Node has been started.')

    def publishCmd(self, msg):
        rev = [-1, 1,-1, -1, -1, 1, 1, -1]
        common_center = 750
        center = [750, 750, 750, 750, 750, 750, 650, 750]
        self.get_logger().info(f'Published(bef adjust): {msg}')
        for idx in range(len(msg.port)):
            adjust = center[msg.port[idx]] - common_center;
            pos = (msg.pos[idx] - common_center + adjust) * rev[msg.port[idx]] + common_center
            if (pos < 0):
                pos = 0
            msg.pos[idx] = pos
        self.publisher_.publish(msg)



    def publish_All(self):
        # メッセージの作成とデータ設定
        duty = int(input("Dutyを指定してください(-250~250)"))
        msg = PwmCmd()
        msg.child_id = 0  # 任意の値を設定
        msg.port = [0, 1, 2, 3, 4, 5, 6, 7]  # 複数のポートを設定
        msg.pos = [750, 750 + duty, 750, 750 - duty, 750, 750+ duty, 750, 750 - duty]  # 各ポートに対応するポジション
        msg.spd = [7, 7, 7, 7, 7, 7, 7, 7]  # 各ポートに対応する速度
        self.publishCmd(msg)
    
    def publishDown(self):
        # メッセージの作成とデータ設定
        msg = PwmCmd()
        msg.child_id = 0  # 任意の値を設定
        msg.port = [0, 1, 2, 3, 4, 5, 6, 7]  # 複数のポートを設定
        msg.pos = [1200, 300, 1200, 300, 1200, 300, 1200, 300]  # 各ポートに対応するポジション
        msg.spd = [7, 7, 7, 7, 7, 7, 7, 7]  # 各ポートに対応する速度
        self.publishCmd(msg)

    def publishCenter(self):
        # メッセージの作成とデータ設定
        msg = PwmCmd()
        msg.child_id = 0  # 任意の値を設定
        msg.port = [0, 1, 2, 3, 4, 5, 6, 7]  # 複数のポートを設定
        msg.pos = [750, 750, 750, 750, 750, 750, 750, 750]  # 各ポートに対応するポジション
        msg.spd = [7, 7, 7, 7, 7, 7, 7, 7]  # 各ポートに対応する速度
        self.publishCmd(msg)

    def publishUp(self):
        # メッセージの作成とデータ設定
        msg = PwmCmd()
        msg.child_id = 0  # 任意の値を設定
        msg.port = [0, 1, 2, 3, 4, 5, 6, 7]  # 複数のポートを設定
        msg.pos =  [900, 600, 900, 600, 900, 600, 900, 600]  # 各ポートに対応するポジション
        msg.spd = [7, 7, 7, 7, 7, 7, 7, 7]  # 各ポートに対応する速度
        self.publishCmd(msg)

    def publish_Once(self):
        cmd = input("ポートかコマンドを指定してください")
        if (cmd == "r"):
            self.publishUp()
        elif (cmd == "f"):
            self.publishDown()
        elif (cmd == "q"):
            self.publish_All()
        elif (cmd == "a"):
            self.publish_All()
        elif (cmd == "c"):
            self.publishCenter()
        else:
            port = int(cmd)
            duty = int(input("Dutyを指定してください(1-180)"))
            msg = PwmCmd()
            msg.child_id = 0  # 任意の値を設定
            msg.port = [port]  # 複数のポートを設定
            msg.pos = [int(duty)]  # 各ポートに対応するポジション
            msg.spd = [15]  # 各ポートに対応する速度
            # メッセージのパブリッシュ
            self.publishCmd(msg)
    def loop(self):
        self.publishDown()
        while(True):
            self.publish_Once()

def main(args=None):
    rclpy.init(args=args)
    node = PwmCmdPublisher()
    node_loop = threading.Thread(target=node.loop)
    node_loop.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly.')
    except Exception as e:
        node.get_logger().error(f'Exception in node: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


# 300~1200 =>750