import argparse
import time
from datetime import datetime
from picamera2 import Picamera2
from libcamera import Transform

def str2bool(v):
    """
    문자열을 Boolean 값으로 변환하는 함수입니다.
    예: 'true', 't', '1' -> True, 'false', 'f', '0' -> False
    """
    if isinstance(v, bool):
        return v
    if v.lower() in ('true', 't', '1'):
        return True
    elif v.lower() in ('false', 'f', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

def main():
    # 1. argparse로 해상도, 파일명, hflip, vflip, delay 옵션 받기
    parser = argparse.ArgumentParser(
        description="Capture a single photo on Raspberry Pi with optional image flipping and delayed capture."
    )
    parser.add_argument(
        "-r", "--resolution",
        nargs=2,
        type=int,
        metavar=("WIDTH", "HEIGHT"),
        default=[640, 480],
        help="Photo resolution (width height). Default: 640 480."
    )
    parser.add_argument(
        "-f", "--filename",
        type=str,
        default="",
        help="Output file name. If not provided, current date and time will be used."
    )
    parser.add_argument(
        "--hflip",
        type=str2bool,
        default=True,
        help="Set horizontal flip on the image. Default: True."
    )
    parser.add_argument(
        "--vflip",
        type=str2bool,
        default=True,
        help="Set vertical flip on the image. Default: True."
    )
    parser.add_argument(
        "-d", "--delay",
        type=int,
        default=0,
        help="Delay in seconds before capturing the photo. If specified, a countdown will be printed. Default: 0 (capture immediately)."
    )
    args = parser.parse_args()

    resolution = (args.resolution[0], args.resolution[1])

    # 2. 파일명 결정 (입력하지 않으면 현재 날짜와 시간을 파일명으로 사용)
    if not args.filename:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{timestamp}.jpg"
    else:
        filename = args.filename
        if not (filename.lower().endswith(".jpg") or filename.lower().endswith(".jpeg")):
            filename += ".jpg"

    # 3. Picamera2 설정 (스틸 이미지 캡처 모드) 및 이미지 flip 설정
    picam2 = Picamera2()
    transform = Transform(hflip=args.hflip, vflip=args.vflip)
    config = picam2.create_still_configuration(main={"size": resolution}, transform=transform)
    picam2.configure(config)

    # 4. 카메라 시작 및 웜업 (자동 노출/화이트밸런스 조절 시간)
    picam2.start()
    time.sleep(2)

    # 5. 딜레이가 있으면 카운트다운 출력 후 촬영
    if args.delay > 0:
        for i in range(args.delay, 0, -1):
            print(i)
            time.sleep(1)

    # 6. 사진 캡처 후 파일로 저장
    picam2.capture_file(filename)
    print(f"Photo captured and saved as {filename}")

    # 7. 종료
    picam2.stop()
    picam2.close()

if __name__ == '__main__':
    main()
