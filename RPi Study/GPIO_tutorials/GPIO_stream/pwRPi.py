import RPi.GPIO as GPIO
import time
import threading

# 핀 번호 (예시: TRIG=23, ECHO=24)
TRIG = 23
ECHO = 24

class UltrasonicSensor:
    def __init__(self):
        self.value = 0.0
        self.lock = threading.Lock()
        self.running = True
        # GPIO 초기화
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(TRIG, GPIO.OUT)
        GPIO.setup(ECHO, GPIO.IN)
        # 센서 값을 주기적으로 읽는 백그라운드 스레드 시작
        self.thread = threading.Thread(target=self._update_sensor)
        self.thread.daemon = True
        self.thread.start()

    def _update_sensor(self):
        while self.running:
            try:
                distance = self._get_distance()
                with self.lock:
                    self.value = distance
            except Exception as e:
                print("초음파 센서 읽기 에러:", e)
            time.sleep(0.1)

    def _get_distance(self):
        # 센서 트리거: 10µs HIGH 펄스 발생
        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)

        # 펄스 시작 시간 측정
        pulse_start = time.time()
        while GPIO.input(ECHO) == 0:
            pulse_start = time.time()

        # 펄스 종료 시간 측정
        while GPIO.input(ECHO) == 1:
            pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 34300 / 2  # cm 단위 계산
        return distance

    def get_value(self):
        with self.lock:
            return self.value

    def cleanup(self):
        self.running = False
        self.thread.join()
        GPIO.cleanup([TRIG, ECHO])
