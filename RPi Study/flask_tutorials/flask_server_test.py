import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import requests
import time

# 라즈베리파이의 IP 주소와 Flask 서버 포트 (실제 IP 주소로 변경)
server_ip = "192.168.0.80"  # 예: "192.168.1.100"
base_url = f"http://{server_ip}:5000"
sine_url = f"{base_url}/sine"

# 최대 200개의 데이터를 저장할 리스트
max_length = 200
data_buffer = []

# 초기 그래프 설정
fig, ax = plt.subplots()
line, = ax.plot([], [], lw=2)
ax.set_xlim(0, max_length)
ax.set_ylim(-5, 5)  # y축 범위는 필요에 따라 조정 가능
ax.set_title("Real-Time Sine Wave Data")
ax.set_xlabel("Sample")
ax.set_ylabel("Sine Value")

def update(frame):
    global data_buffer
    try:
        # 쿼리 파라미터에 현재 시간을 추가해 캐시를 우회
        response = requests.get(sine_url, params={"t": str(time.time())})
        if response.ok:
            sine_data = response.json()
            sine_value = sine_data.get("sine_value", 0)
            data_buffer.append(sine_value)
            # 최대 길이 유지
            if len(data_buffer) > max_length:
                data_buffer.pop(0)
        else:
            print("사인 데이터 가져오기 실패:", response.text)
    except Exception as e:
        print("데이터 요청 중 오류 발생:", e)
    
    # 그래프 업데이트: x축은 샘플 번호, y축은 저장된 사인 값
    line.set_data(range(len(data_buffer)), data_buffer)
    ax.set_xlim(0, max_length)
    return line,

# 50ms마다 update 함수를 호출하여 그래프를 갱신 (실시간 업데이트)
ani = FuncAnimation(fig, update, interval=50)
plt.show()
