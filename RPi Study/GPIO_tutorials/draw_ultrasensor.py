import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import requests
import time

# 라즈베리파이의 IP 주소와 Flask 서버 포트 (실제 IP 주소로 변경)
server_ip = "192.168.0.80"  # 예: "192.168.1.100"
base_url = f"http://{server_ip}:5000"
url = f"{base_url}/sensor"

# 최대 100개의 데이터를 저장할 리스트
max_length = 100
data_buffer = []
time_buffer = []  # 시간 정보를 저장할 리스트

# 코드 시작 시각 (x축 0초 기준)
start_time = time.time()

# 초기 그래프 설정
fig, ax = plt.subplots()
line, = ax.plot([], [], lw=2)
ax.set_xlabel("Time (s)")
ax.set_ylabel("Ultra Sensor")
ax.set_title("Real-Time Ultra Sensor Data")
ax.grid(True)

def update(frame):
    global data_buffer, time_buffer, start_time
    try:
        # 센서 데이터 요청 전 시각 기록
        request_time = time.time()
        response = requests.get(url, params={"t": str(request_time)})
        if response.ok:
            ultra_data = response.json()
            ultra_value = ultra_data.get("sensor_value", 0)
            data_buffer.append(ultra_value)
            
            # 경과 시간을 계산하여 추가 (시작 시각으로부터 몇 초 경과했는지)
            elapsed_time = time.time() - start_time
            time_buffer.append(elapsed_time)
            
            # 버퍼 길이가 max_length를 초과하면 가장 오래된 데이터 제거
            if len(data_buffer) > max_length:
                data_buffer.pop(0)
                time_buffer.pop(0)
        else:
            print("사인 데이터 가져오기 실패:", response.text)
    except Exception as e:
        print("데이터 요청 중 오류 발생:", e)
    
    # 그래프 업데이트: x축은 시간, y축은 센서 값
    line.set_data(time_buffer, data_buffer)
    
    # x축 범위를 설정: 초기 5초 동안은 [0, 5]로 고정, 이후엔 최근 5초 구간으로 이동
    if time_buffer:
        current_time = time_buffer[-1]
        if current_time < 5:
            ax.set_xlim(0, 5)
        else:
            ax.set_xlim(current_time - 5, current_time)
    
    # y축 범위는 데이터 최대값에 맞게 동적으로 설정 (약간의 여유를 둠)
    if data_buffer:
        ax.set_ylim(0, max(data_buffer) * 1.2)
    
    return line,

# 50ms마다 update 함수를 호출하여 그래프를 갱신 (실시간 업데이트)
ani = FuncAnimation(fig, update, interval=50)
plt.show()
