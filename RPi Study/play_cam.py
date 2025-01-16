import argparse
import time
from picamera2 import Picamera2, Preview
from libcamera import Transform

def main():
    # 1. argparse로 재생 시간 인자 받기
    parser = argparse.ArgumentParser(description="Play camera feed for a specified time.")
    parser.add_argument(
        "-t", 
        "--time", 
        type=float,          # 정수뿐만 아니라 부동소수점도 가능하게
        default=3.0,         # 기본값 3초
        help="Time (in seconds) to play camera feed."
    )
    args = parser.parse_args()

    # 2. PiCamera2 설정
    picam2 = Picamera2()

    config = picam2.create_video_configuration(
        main={"size": (320, 240)}, 
        transform=Transform(hflip=True, vflip=True)
    )
    picam2.configure(config)

    picam2.start_preview(Preview.QT)
    picam2.start()

    # 4. 지정된 시간만큼 재생
    print(f"Camera is playing for {args.time} seconds...")
    time.sleep(args.time)

    # 5. 종료
    picam2.stop()
    picam2.stop_preview()  
    picam2.close()

if __name__ == '__main__':
    main()
