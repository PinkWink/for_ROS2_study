from flask import Flask, request, jsonify
import threading
import time
import math

app = Flask(__name__)

# 초기 파라미터 설정
period = 2.0          # 주기 (초)
amplitude = 1.0       # 진폭
sample_time = 0.05    # 샘플 간격 (초)
sine_value = 0.0      # 계산된 사인 함수의 결과
t = 0.0               # 현재 시간 (주기 내의 위치)

# 다중 스레드 환경에서의 안전을 위한 락 객체
param_lock = threading.Lock()

def sine_updater():
    global t, sine_value, period, amplitude
    while True:
        with param_lock:
            # 사인 함수 계산: amplitude * sin(2πt/period)
            sine_value = amplitude * math.sin(2 * math.pi * t / period)
        time.sleep(sample_time)
        t += sample_time
        # t가 주기를 넘어가면, 주기 내의 위치로 재설정
        if t >= period:
            t = t % period

@app.route('/sine')
def get_sine():
    """
    현재 사인 함수 값과 현재 파라미터(주기, 진폭)를 JSON 형식으로 반환
    """
    with param_lock:
        return jsonify({
            "sine_value": sine_value,
            "period": period,
            "amplitude": amplitude
        })

@app.route('/update_params', methods=['POST'])
def update_params():
    """
    클라이언트로부터 새로운 주기와 진폭 값을 받아 업데이트
    JSON 데이터 예시: {"period": 3.0, "amplitude": 2.0}
    """
    global period, amplitude, t
    data = request.get_json()
    response = {}
    with param_lock:
        if 'period' in data:
            try:
                new_period = float(data['period'])
                if new_period <= 0:
                    return jsonify({"status": "error", "message": "Period must be positive"}), 400
                period = new_period
                # 주기가 변경되면, 위상(t)을 0으로 재설정하여 부드럽게 시작
                t = 0.0
                response['period'] = period
            except ValueError:
                return jsonify({"status": "error", "message": "Invalid period value"}), 400

        if 'amplitude' in data:
            try:
                new_amplitude = float(data['amplitude'])
                amplitude = new_amplitude
                response['amplitude'] = amplitude
            except ValueError:
                return jsonify({"status": "error", "message": "Invalid amplitude value"}), 400

    return jsonify({"status": "success", "updated": response})

if __name__ == '__main__':
    # 백그라운드 스레드로 사인 값 업데이트 시작
    updater_thread = threading.Thread(target=sine_updater, daemon=True)
    updater_thread.start()
    # 모든 네트워크 인터페이스에서 접속할 수 있도록 실행 (포트 5000)
    app.run(host='0.0.0.0', port=5000)
