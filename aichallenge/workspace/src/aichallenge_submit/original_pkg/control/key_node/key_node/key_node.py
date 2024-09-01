import rclpy
from rclpy.node import Node
import pyautogui
import time

class KeyboardSimulatorNode(Node):

    def __init__(self):
        super().__init__('keyboard_simulator')
        self.simulate_keypress()  # ノードの初期化時に一度だけキー操作をシミュレート

    def simulate_keypress(self):
        self.get_logger().info('Simulating Alt+Tab to switch window...')
        # pyautogui.hotkey('alt', 'tab')  # Alt+Tabでウィンドウを切り替える
        time.sleep(1)  # ウィンドウが切り替わるのを待つ
        self.get_logger().info('Simulating key press...')
        pyautogui.press('r')  # 'r'キーを押す

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardSimulatorNode()
    rclpy.shutdown()  # 処理が完了したらノードを終了

if __name__ == '__main__':
    main()