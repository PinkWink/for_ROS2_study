from flask import Flask, jsonify
import threading
import time

app = Flask(__name__)
counter = 0  # 전역 카운터 변수

def counter_updater():
    global counter
    while True:
        time.sleep(1)  # 1초 대기
        counter += 1  # 카운터 1 증가
        print(f"Counter updated: {counter}")  # 서버 콘솔에 현재 값 출력 (선택)

@app.route('/counter')
def get_counter():
    """현재 카운터 값을 JSON 형태로 반환"""
    return jsonify({"counter": counter})

if __name__ == '__main__':
    # 백그라운드 스레드로 카운터 업데이트 시작
    updater_thread = threading.Thread(target=counter_updater, daemon=True)
    updater_thread.start()
    
    # 모든 네트워크 인터페이스에서 접속 가능하도록 설정
    app.run(host='0.0.0.0', port=5000)
