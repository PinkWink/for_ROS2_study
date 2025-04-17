#!/usr/bin/env python3
# pip3 install pyserial

import serial
import time

# 시리얼 포트 설정
#  - USB-to-Serial 케이블 사용 시: '/dev/ttyUSB0'
#  - GPIO UART 사용 시:    '/dev/ttyAMA0' 또는 '/dev/serial0'
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600
TIMEOUT  = 1  # 읽기 타임아웃 (초)

def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
        # 포트가 열릴 때까지 잠깐 대기
        time.sleep(2)
        print(f"Opened serial port {SERIAL_PORT} at {BAUD_RATE}bps")
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return

    try:
        while True:
            # 한 줄 읽기 (줄끝 \r\n 포함)
            raw = ser.readline()
            if not raw:
                # 타임아웃 시 빈 바이트열 반환 → 반복
                continue

            # 디코딩 및 개행(\r\n) 제거
            line = raw.decode('utf-8', errors='replace').strip()
            print(f"Received: {line}")

    except KeyboardInterrupt:
        print("\nStopping serial read.")
    finally:
        ser.close()
        print("Serial port closed.")

if __name__ == "__main__":
    main()
