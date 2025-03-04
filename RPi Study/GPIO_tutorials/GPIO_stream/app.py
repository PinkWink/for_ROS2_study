import cv2
import time
import threading
import atexit
from flask import Flask, Response, render_template, jsonify
from picamera2 import Picamera2
from libcamera import Transform
from pwRPi import UltrasonicSensor  # 초음파 센서 모듈

class VideoStream:
    def __init__(self, resolution=(320, 240), hflip=True, vflip=True, sleep_time=0.01):
        self.picam2 = Picamera2()
        video_config = self.picam2.create_video_configuration(
            main={"size": resolution},
            transform=Transform(hflip=hflip, vflip=vflip)
        )
        self.picam2.configure(video_config)
        self.picam2.start()

        self.global_frame = None
        self.frame_lock = threading.Lock()
        self.sleep_time = sleep_time
        self.running = False
        self.capture_thread = None

    def start(self):
        """비디오 스트리밍을 시작합니다."""
        self.running = True
        self.capture_thread = threading.Thread(target=self._capture_frames)
        self.capture_thread.daemon = True
        self.capture_thread.start()

    def _capture_frames(self):
        """카메라에서 프레임을 캡처하여 내부 버퍼에 저장합니다."""
        while self.running:
            frame_data = self.picam2.capture_array()
            frame_data = cv2.cvtColor(frame_data, cv2.COLOR_RGB2BGR)
            ret, buffer = cv2.imencode('.jpg', frame_data)
            if not ret:
                continue
            frame = buffer.tobytes()
            with self.frame_lock:
                self.global_frame = frame
            time.sleep(self.sleep_time)

    def gen_frames(self):
        """스트리밍할 프레임 제너레이터를 반환합니다."""
        while self.running:
            with self.frame_lock:
                frame = self.global_frame
            if frame is None:
                time.sleep(self.sleep_time)
                continue
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            time.sleep(self.sleep_time)

    def stop(self):
        """스트리밍을 중지하고 카메라 리소스를 해제합니다."""
        self.running = False
        if self.capture_thread:
            self.capture_thread.join()
        self.picam2.stop()

class SensorHandler:
    def __init__(self):
        self.ultrasonic_sensor = UltrasonicSensor()

    def get_sensor_value(self):
        """초음파 센서의 값을 반환합니다."""
        return self.ultrasonic_sensor.get_value()

    def cleanup(self):
        """센서 리소스를 해제합니다."""
        self.ultrasonic_sensor.cleanup()

def create_app():
    """Flask 애플리케이션과 관련 리소스를 초기화합니다."""
    app = Flask(__name__)
    
    # VideoStream과 SensorHandler 인스턴스 생성 및 시작
    video_stream = VideoStream()
    video_stream.start()
    sensor_handler = SensorHandler()

    @app.route('/')
    def index():
        return render_template('index.html')

    @app.route('/video_feed')
    def video_feed():
        return Response(video_stream.gen_frames(),
                        mimetype='multipart/x-mixed-replace; boundary=frame')

    @app.route('/sensor')
    def sensor():
        value = sensor_handler.get_sensor_value()
        return jsonify(sensor_value=value)

    def shutdown():
        """애플리케이션 종료 시 리소스 정리"""
        video_stream.stop()
        sensor_handler.cleanup()

    atexit.register(shutdown)
    return app

if __name__ == '__main__':
    app = create_app()
    app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)
