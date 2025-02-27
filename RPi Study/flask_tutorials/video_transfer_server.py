import cv2
import time
import threading
from flask import Flask, Response, render_template
from picamera2 import Picamera2
from libcamera import Transform

app = Flask(__name__)

# 1. Picam2를 전역으로 한 번만 초기화
picam2 = Picamera2()
video_config = picam2.create_video_configuration(
    main={"size": (320, 240)}, 
    transform=Transform(hflip=True, vflip=True)
)
picam2.configure(video_config)
picam2.start()

# 2. 최신 프레임을 저장할 전역 변수와 락 생성
global_frame = None
frame_lock = threading.Lock()

def capture_frames():
    """백그라운드 스레드에서 지속적으로 프레임을 캡처하여 global_frame 갱신"""
    global global_frame
    while True:
        # Picam2로부터 프레임 캡처 (numpy 배열)
        frame_data = picam2.capture_array()
        
        # 필요한 경우 OpenCV 처리를 수행 (예: 색상 변환)
        frame_data = cv2.cvtColor(frame_data, cv2.COLOR_RGB2BGR)
        
        # 프레임을 JPEG로 인코딩
        ret, buffer = cv2.imencode('.jpg', frame_data)
        if not ret:
            continue
        frame = buffer.tobytes()
        
        # 스레드 안전하게 전역 변수 업데이트
        with frame_lock:
            global_frame = frame
        
        # 너무 빠른 루프로 인한 과부하를 방지하기 위해 짧게 대기
        time.sleep(0.01)

# 3. 백그라운드 스레드 시작
capture_thread = threading.Thread(target=capture_frames)
capture_thread.daemon = True
capture_thread.start()

def gen_frames():
    """각 클라이언트의 요청마다 호출되어 global_frame을 스트리밍"""
    while True:
        with frame_lock:
            frame = global_frame
        if frame is None:
            continue
        yield ( b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.01)

@app.route('/')
def index():
    # 클라이언트에서 영상 스트리밍을 확인할 HTML 템플릿 렌더링
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    # 여러 클라이언트가 접근해도 동일한 global_frame을 읽어 스트리밍
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)
    finally:
        picam2.stop()
