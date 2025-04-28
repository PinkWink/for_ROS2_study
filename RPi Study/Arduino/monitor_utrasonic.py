import serial, numpy as np, matplotlib.pyplot as plt, matplotlib.animation as animation

# === 설정 ===
PORT = '/dev/cu.usbserial-210'  # 실제 포트로 바꿔주세요
BAUD = 9600
ser = serial.Serial(PORT, BAUD, timeout=1)

# === 반원 레이더 플롯 설정 ===
fig = plt.figure()
ax  = fig.add_subplot(111, projection='polar')
ax.set_thetalim(0, np.pi)   # 0→π (반원)
ax.set_rmax(0.5)              # 최대 0.5 m
ax.grid(True)

# === 플롯 요소들 ===
points,       = ax.plot([], [], 'o',  color='cyan', markersize=3)
dashed_line,  = ax.plot([], [], '--', color='cyan', linewidth=1)
current_line, = ax.plot([], [], '-',  color='green', linewidth=2)

# 각도→거리 매핑 저장소
angle_dist_map = {}

def init():
    points.set_data([], [])
    dashed_line.set_data([], [])
    current_line.set_data([], [])
    return points, dashed_line, current_line

def animate(_):
    raw = ser.readline().decode('utf-8').strip()
    if not raw:
        return points, dashed_line, current_line

    try:
        angle_deg, distance_cm = map(float, raw.split(','))
    except ValueError:
        return points, dashed_line, current_line

    # cm → m
    distance = distance_cm / 100.0
    if not (0 <= distance <= 1):
        return points, dashed_line, current_line

    # sweep 주기마다(clear on 0°) 맵 리셋
    if int(angle_deg) == 0:
        angle_dist_map.clear()

    # 플롯용 θ 계산 (0°→π, 180°→0)
    theta = np.pi - np.deg2rad(angle_deg)

    # 같은 θ 값은 새 거리로 덮어쓰기
    angle_dist_map[theta] = distance

    # 정렬해서 꺼내기
    sorted_thetas = sorted(angle_dist_map.keys())
    sorted_dists  = [angle_dist_map[t] for t in sorted_thetas]

    # 업데이트
    points.set_data(sorted_thetas, sorted_dists)
    dashed_line.set_data(sorted_thetas, sorted_dists)
    current_line.set_data([theta, theta], [0, distance])

    return points, dashed_line, current_line

ani = animation.FuncAnimation(
    fig, animate, init_func=init,
    interval=20, blit=True
)
plt.show()
