import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import Twist

STEP = 0.1          # 1回のキー入力で変化する速度の刻み
MAX_LINEAR = 1.0    # 並進速度の上限 [m/s]
MAX_ANGULAR = 1.0   # 角速度の上限 [rad/s]

HELP = 'f:前進  b:後退  l:左旋回  r:右旋回  s:停止  q:終了  (入力後Enter)'


class CommandPublisher(Node):

    def __init__(self):
        super().__init__('command_publisher_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # 浮動小数点の累積誤差を避けるため、速度は「段数(整数)」で保持する
        self.linear_steps = 0
        self.angular_steps = 0
        self.max_linear_steps = round(MAX_LINEAR / STEP)
        self.max_angular_steps = round(MAX_ANGULAR / STEP)

    def handle_key(self, key):
        """キーに応じて速度を更新しpublishする。有効なキーならTrueを返す。"""
        if key == 'f':
            self.linear_steps += 1
        elif key == 'b':
            self.linear_steps -= 1
        elif key == 'l':
            self.angular_steps += 1
        elif key == 'r':
            self.angular_steps -= 1
        elif key == 's':
            self.linear_steps = 0
            self.angular_steps = 0
        else:
            return False

        # 上限・下限でクランプ
        self.linear_steps = max(-self.max_linear_steps,
                                min(self.max_linear_steps, self.linear_steps))
        self.angular_steps = max(-self.max_angular_steps,
                                 min(self.max_angular_steps, self.angular_steps))
        self.publish_velocity()
        return True

    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = round(self.linear_steps * STEP, 2)
        msg.angular.z = round(self.angular_steps * STEP, 2)
        self.publisher_.publish(msg)
        self.get_logger().info(
            f'並進速度={msg.linear.x:.2f} [m/s], 角速度={msg.angular.z:.2f} [rad/s]')

    def stop(self):
        """ゼロ速度をpublishする(終了時用)。"""
        self.linear_steps = 0
        self.angular_steps = 0
        self.publish_velocity()


def main(args=None):
    # SIGINTをrclpyに奪わせず、Python標準のKeyboardInterruptとして受け取る
    # (終了時に停止指令をpublishするため)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = CommandPublisher()
    print(HELP)
    try:
        while True:
            key = input('>>> ').strip().lower()
            if key == 'q':
                break
            if key == '':
                continue
            if not node.handle_key(key):
                print(f'無効なキーです: {key!r}')
                print(HELP)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        if rclpy.ok():
            node.stop()
            time.sleep(0.1)  # 停止指令が送信されるのを少し待つ
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
