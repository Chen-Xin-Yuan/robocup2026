#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
上位机模拟脚本：十字对准 (align) 三阶段通信

协议（USART2, 115200，小端 float，无校验）：
  阶段1 开始   车 -> 上位机 : AA 01 0A                  (请求回传 x/y/yaw)
  阶段2 数据   上位机 -> 车 : AA 0C <x:4B> <y:4B> <yaw:4B> 0A   (纠正帧, 共15字节)
  阶段3 结束   上位机 -> 车 : AA 00 0A                  (完毕帧 -> 车 align_flag=0)
  确认   车 -> 上位机 : "ALIGN_DONE\r\n"

用法：把 COMx 改成实际串口号，然后运行；车进入对准后会发请求帧，脚本按三阶段回传。
"""
import serial
import struct
import time


def pack_correct(x, y, yaw):
    """纠正帧: AA 0C + x/y/yaw(小端float) + 0A"""
    return bytes([0xAA, 0x0C]) + struct.pack('<fff', x, y, yaw) + bytes([0x0A])


def pack_done():
    """完毕帧: AA 00 0A"""
    return bytes([0xAA, 0x00, 0x0A])


def wait_request(ser, timeout=5.0):
    """滑动一个字节地找车的请求帧 AA 01 0A"""
    deadline = time.time() + timeout
    buf = b''
    while time.time() < deadline:
        b = ser.read(1)
        if not b:
            continue
        buf = (buf + b)[-3:]
        if buf == b'\xAA\x01\x0A':
            return True
    return False


def main():
    ser = serial.Serial('COMx', 115200, timeout=0.2)

    # ============ 阶段1: 开始 ============
    print('=== 阶段1: 开始 ===')
    print('等待车发送请求帧 AA 01 0A ...')
    if not wait_request(ser):
        print('超时：没等到车的请求帧，请确认车已进入对准流程')
        return
    print('收到请求帧，进入数据传送阶段\r\n')

    # ============ 阶段2: 传送数据 ============
    print('=== 阶段2: 传送数据（典型序列：从大偏移逐步收敛）===')
    corrections = [
        (15.0, 20.0, 8.0),   # 十字偏右15cm、前方20cm、逆时针8°
        (8.0, 10.0, 3.0),
        (2.5, 3.0, 1.0),
        (0.5, 0.8, 0.3),
    ]
    for i, (x, y, yaw) in enumerate(corrections, 1):
        frame = pack_correct(x, y, yaw)
        ser.write(frame)
        print('  第%d帧: %s  (x=%s y=%s yaw=%s)'
              % (i, frame.hex(' ').upper(), x, y, yaw))
        time.sleep(0.6)

    # ============ 阶段3: 结束 ============
    print('\n=== 阶段3: 结束 ===')
    done = pack_done()
    ser.write(done)
    print('  发送完毕帧: %s  -> 车 align_flag=0 退出循环' % done.hex(' ').upper())
    line = ser.readline()
    print('  车回传确认: %r' % line)


if __name__ == '__main__':
    main()
