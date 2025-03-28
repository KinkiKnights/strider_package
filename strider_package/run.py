import rclpy
from rclpy.node import Node
from kk_driver_msg.msg import PwmCmd  # カスタムメッセージのインポート
from kk_driver_msg.msg import Core
import threading
import math
import time

import numpy as np

class StriderWalkIK():
    def __init__(self, node):
        
        # ACOSテーブル事前計算
        self.MIN_CALC = 0.001;
        self.MIN_DIST = 0.001;
        self.ROOT = 0.5;
        self.LEG = 0.5;
        idx_max = 1 / self.MIN_DIST;
        self.distable = [0] * int(idx_max);
        for rad in np.arange(0.0, math.pi, self.MIN_CALC):
            dist = math.sqrt(self.ROOT * self.ROOT + self.LEG * self.LEG - 2 * self.LEG * self.ROOT * math.cos(rad))
            for widx in range(int(self.dist_idx(dist)), int(idx_max), 1):
                self.distable[widx] = rad;
        print(self.distable)

        # 初期化処理
        self.node = node;
        self.is_run = False
        self.last_x = [0, 0, 0, 0]

    # 機構メッセージのPublish
    def publishCmd(self, msg):
        rev = [-1, 1, 1, -1, -1, 1, 1, -1]
        common_center = 750
        center = [720, 870, 770, 950, 650, 940, 470, 940]
        # center = [700, 700, 700, 710, 750, 700, 600, 720]
        # self.node.get_logger().info(f'Published(bef adjust): {msg}')
        for idx in range(len(msg.port)):
            adjust = center[msg.port[idx]] - common_center;
            pos = (msg.pos[idx] - common_center + adjust) * rev[msg.port[idx]] + common_center
            if (pos < 0):
                pos = 0
            msg.pos[idx] = pos
        self.node.publisher_.publish(msg)
    # 距離テーブルインデックス算出
    def dist_idx(self,dist):
        idx = int(dist / self.MIN_DIST);
        if (len(self.distable) <= idx):
            idx = len(self.distable) - 1;
        return idx;
    # 足逆運動学計算
    def calc(self, x, y):

        tmp_dist = math.sqrt(x * x + y * y)
        tmp_rad = math.atan2(y, -x)
        tmp_rad_leg = self.distable[self.dist_idx(tmp_dist)]
        rad_root = tmp_rad - (math.pi - tmp_rad_leg) / 2
        rad_leg = math.pi - tmp_rad_leg + rad_root
        # print(rad_root / math.pi  * 180, rad_leg / math.pi  * 180)

        return rad_root, rad_leg
    # Duty変換
    def rad2Duty(self, rad):
        pi_duty = 900
        half_duty = 750;
        return int(pi_duty * (rad / math.pi - 1/2) + half_duty)
    # 機構破損防止計算
    def checkBreak(self, radroot, radleg):
        if (radroot > radleg):
            print("Axis Broken!!", radroot, radleg);
            return False
        return True
    # 機構動作
    def mov(self, idx, x, y, speed = 7):
        print("Move to", idx, x, y)
        self.last_x[idx] = x
        root, leg = self.calc(x,y)
        if (self.checkBreak(root, leg)):
            msg = PwmCmd()
            msg.child_id = 0
            msg.port = [idx * 2, idx * 2 + 1]
            msg.pos = [self.rad2Duty(leg), self.rad2Duty(root)]
            msg.spd = [speed, speed]
            self.publishCmd(msg)
            return True
        return False
        
    def movUP(self, idx, x, y, speed = 7):
        self.last_x[idx] = x
        root, leg = self.calc(x,y)
        if (self.checkBreak(root, leg)):
            msg = PwmCmd()
            msg.child_id = 0
            msg.port = [idx * 2, idx * 2 + 1]
            if (x < -0.5):
                msg.pos = [self.rad2Duty(3.1), self.rad2Duty(-0.5)]
            else:
                msg.pos = [self.rad2Duty(3.1), self.rad2Duty(root)]
            msg.spd = [speed, speed]
            self.publishCmd(msg)
            return True
        return False
        
    
    def start(self):        
        self.jump_idx = [1.0, 1.0, 1.0, 1.0]
        self.base_y = 0.9
        self.jump_y = 0.3
        self.base_x = 0
        self.grand_x = [0.35, 0.05, 0.2, -0.1]
        self.x_offset = [ -0.15, -0.15, 0, 0]
        self.y_offset = [-0.03, -0.03, -0.02, -0.03]

        self.LIMIT_X = -0.2
        self.START_X = 0.4
        self.UPDATE_X = -0.01

        self.UPDATE_JUMP = 0.1

    def update(self):
        self.run = True;
        self.first = True;
        self.start();
        
        self.wait_time = 0.15
        self.diff = -0.03
        self.start = 0.35
        self.finish = -0.35
        self.forward = 0.05
        self.backward = -0.1
        self.y_gain = 0.5
        self.y_gain_raise = 0.1

        self.stand_y = [0.8, 0.8, 0.8,0.8]
        self.raise_y = 0.2

        self.next_diff = 0.2

        self.dist = 0;
        self.center = (self.start + self.finish) / 2
        quat = (self.start - self.finish) / 4
        stepA = self.center + quat
        stepB = self.center - quat

        self.leg = [stepA, stepB, stepB, stepA]
        self.y_adjust = [0,0.035,0.03,0.03]
        self.y_kick = [0.1,0.1,0.1,0.1]

        """
        初期動作
        """
        self.mov(0, -0.1, 0.1, 8);
        self.mov(1, -0.1, 0.1, 8);
        self.mov(2, -0.1, 0.1, 8);
        self.mov(3, -0.1, 0.1, 8);
        i = input("歩行開始します")
        self.mov(0, -0.25, 0.7, 9);
        self.mov(1, -0.25, 0.7, 9);
        self.mov(2, -0.25, 0.7, 9);
        self.mov(3, -0.25, 0.7, 9);
        time.sleep(3)
        while(self.run):
            time.sleep(1/ all_speed / control_hz);
            if not (self.is_run or self.first):
                continue;
            self.first = False;
            sequence_count = sequence_count + 1;
            for idx in range(4):
                frame_count = (sequence_count + sequence_offset[idx]) % sequence_full
                if (frame_count < jump_full):
                    x = (start_x - finish_x) / jump_full * frame_count + finish_x  + self.x_offset[idx];

                    if (jump_mode[idx]):
                        # self.mov(idx, x, jump_y + self.y_offset[idx], 11);
                        self.movUP(idx, start_x + self.x_offset[idx], base_y[step_mode[idx]] + self.y_offset[idx], 15);
                    else:
                        """
                        初回ジャンプ処理
                        """
                        if (idx < 2): 
                            """
                            # 前足ステップ判定
                            """
                            while not(self.in_step):
                                time.sleep(0.1);
                            if (self.is_step):
                                step_mode[idx] = 1;
                            elif(self.last_step):
                                step_mode[idx] = 2;
                                self.last_step = False
                            else:
                                step_mode[idx] = 0;
                            if (self.back):
                                leg_mode[idx + 2] = True;
                                self.back = False;
                            else :
                                leg_mode[idx + 2] = False;
                            self.in_step = False
                            self.last_step = self.is_step;
                        else:
                            """
                            # 後足ステップ判定
                            """
                            step_mode[idx] = step_mode[idx - 2]
                            if (leg_mode[idx]):
                                step_mode[idx] = 1;
                                leg_mode[idx] = True;
                        if (step_mode[idx] == 3):
                            """
                            降壇用ジャンプ
                            """
                            self.mov(idx, finish_x +down_returd, jump_y , 15);
                            time.sleep(1)
                            self.movUP(idx, finish_x + self.x_offset[idx] +jump_returd,  jump_y + self.y_offset[idx], 15);
                            time.sleep(1)
                        else:
                            """
                            通常ジャンプ
                            """
                            self.mov(idx, finish_x +jump_returd, jump_y , 15);
                            time.sleep(1)
                            self.movUP(idx, finish_x + self.x_offset[idx] +jump_returd,  jump_y + self.y_offset[idx], 15);
                            time.sleep(1)
                    jump_mode[idx] = True;
                else:
                    x = (finish_x - start_x) / (sequence_full - jump_full) * (frame_count - jump_full) + start_x  + self.x_offset[idx];
                    
                    
                    if (jump_mode[idx]):
                        self.movUP(idx, start_x + self.x_offset[idx], base_y[step_mode[idx]] + self.y_offset[idx], 15);
                        time.sleep(jump_wait)
                        self.mov(idx, start_x, base_y[step_mode[idx]] + self.y_offset[idx], 8);
                        time.sleep(jump_wait)
                    else:
                        self.mov(idx, x, base_y[step_mode[idx]] + self.y_offset[idx], 11);
                    jump_mode[idx] = False;


class PwmCmdPublisher(Node):
    def __init__(self):
        super().__init__('strider_adjust_node')
        self.publisher_ = self.create_publisher(PwmCmd, '/pwm/cmd', 8)
        self.subscription = self.create_subscription(
            Core, '/core', self.button_callback, 10)
        self.IK = StriderWalkIK(self)
    
    def button_callback(self, msg):
        global is_button
        is_button = not(msg.limit)

    def loop(self):
        
        self.IK.mov(0, 0, 1.0);
        self.IK.mov(1, 0, 1.0);
        self.IK.mov(2, 0, 1.0);
        self.IK.mov(3, 0, 1.0);
        # self.publishDown()
        while(True):
            ctrl_loop = threading.Thread(target=self.ctrl_th)
            ctrl_loop.start()
            self.IK.update();

    def ctrl_th(self):
        self.run_flag = False
        input("Enterで動作開始します")
        self.IK.is_run = False
        self.IK.back = False
        while(True):
            input("q:Stop, Enter")
            self.IK.is_run = True
            while(True):
                i = input("")
                if (i == "5"):
                    break;
                if (i == "0"):
                    self.IK.back = False
                    self.IK.is_step = True;
                    self.IK.in_step = True;
                elif (i == "1"):
                    self.IK.back = True
                    self.IK.is_step = False;
                    self.IK.in_step = True;
                elif (i == "2"):
                    self.IK.back = False
                    self.IK.is_step = False;
                    self.IK.in_step = True;
                    self.IK.last_step = True;
                else:
                    self.IK.back = False
                    self.IK.is_step = False;
                    self.IK.in_step = True;

            self.IK.is_run = False

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

