#!/usr/bin/env python3
import can
import struct
import time
import argparse

class MotorCommandSender:
    def __init__(self, interface='socketcan', channel='can0', bitrate=500000):
        """
        USBCANデバイス初期化
        interface: 'socketcan' (Linux) or 'pcan' (Windows PEAK) など
        """
        self.bus = can.interface.Bus(
            interface=interface, 
            channel=channel, 
            bitrate=bitrate
        )
        self.seq = 0
        
    def send_motor_command(self, board_id, motor_id, mode, value, timeout=10):
        """
        モーターコマンド送信
        board_id: 1-4
        motor_id: 1-8  
        mode: 0x00=Disable, 0x01=Speed, 0x02=BoundedAngle, 0x03=UnboundedAngle
        value: float値 (rad/s または rad)
        timeout: タイムアウト [10ms単位]
        """
        # CAN ID生成: 0x0XY (X=board_id, Y=motor_id)
        can_id = (board_id << 4) | motor_id
        
        # float→int32変換 (1000倍スケール)
        int_value = int(value * 1000)
        
        # データフレーム構築
        data = struct.pack('<BBiBB', 
                          mode,           # Byte0: MODE
                          0,              # Byte1: OPT (固定0)
                          int_value,      # Byte2-5: VALUE (little-endian int32)
                          self.seq & 0xFF,# Byte6: SEQ
                          timeout & 0xFF  # Byte7: TMO
                          )
        
        # CAN送信
        msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        self.bus.send(msg)
        
        print(f"Sent: ID=0x{can_id:02X}, mode={mode}, value={value:.3f}, seq={self.seq}")
        self.seq = (self.seq + 1) % 256
        
    def speed_control(self, board_id, motor_id, speed_rad_s):
        """速度制御コマンド"""
        self.send_motor_command(board_id, motor_id, 0x01, speed_rad_s)
        
    def angle_control(self, board_id, motor_id, angle_rad, bounded=True):
        """角度制御コマンド"""
        mode = 0x02 if bounded else 0x03
        self.send_motor_command(board_id, motor_id, mode, angle_rad)
        
    def disable_motor(self, board_id, motor_id):
        """モーター停止"""
        self.send_motor_command(board_id, motor_id, 0x00, 0.0)

def main():
    parser = argparse.ArgumentParser(description='Motor Command Sender via USB-CAN')
    parser.add_argument('--board', '-b', type=int, default=1, help='Board ID (1-4)')
    parser.add_argument('--motor', '-m', type=int, default=1, help='Motor ID (1-8)')
    parser.add_argument('--interface', '-i', default='socketcan', help='CAN interface')
    parser.add_argument('--channel', '-c', default='can0', help='CAN channel')
    
    args = parser.parse_args()
    
    # CANバス接続
    sender = MotorCommandSender(interface=args.interface, channel=args.channel)
    
    print(f"Connected to CAN bus. Board ID: {args.board}")
    print("Commands:")
    print("  s <speed>    - Speed control [rad/s]")
    print("  a <angle>    - Angle control [rad] (bounded)")
    print("  u <angle>    - Angle control [rad] (unbounded)")
    print("  d            - Disable motor")
    print("  q            - Quit")
    
    try:
        while True:
            cmd = input(f"Motor{args.motor}> ").strip().split()
            if not cmd:
                continue
                
            if cmd[0] == 'q':
                break
            elif cmd[0] == 's' and len(cmd) == 2:
                speed = float(cmd[1])
                sender.speed_control(args.board, args.motor, speed)
            elif cmd[0] == 'a' and len(cmd) == 2:
                angle = float(cmd[1])
                sender.angle_control(args.board, args.motor, angle, bounded=True)
            elif cmd[0] == 'u' and len(cmd) == 2:
                angle = float(cmd[1])
                sender.angle_control(args.board, args.motor, angle, bounded=False)
            elif cmd[0] == 'd':
                sender.disable_motor(args.board, args.motor)
            else:
                print("Invalid command")
                
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        sender.bus.shutdown()

if __name__ == "__main__":
    main()