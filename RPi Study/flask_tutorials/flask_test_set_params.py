import requests
import argparse

def update_server_params(server_ip, period, amplitude):
    """
    주어진 서버 IP에 대해, 주기와 진폭 값을 업데이트하는 POST 요청을 보냅니다.
    """
    base_url = f"http://{server_ip}:5000"
    update_url = f"{base_url}/update_params"
    data = {"period": period, "amplitude": amplitude}
    
    try:
        response = requests.post(update_url, json=data)
        response.raise_for_status()  # HTTP 오류 발생 시 예외 발생
        print("Parameter update successful:")
        print(response.json())
    except requests.exceptions.RequestException as e:
        print("Error updating parameters:", e)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Update period and amplitude parameters on the Flask server.")
    parser.add_argument("--server_ip", type=str, default="192.168.0.80", help="IP address of the Flask server")
    parser.add_argument("--period", type=float, required=True, help="New period value (in seconds)")
    parser.add_argument("--amplitude", type=float, required=True, help="New amplitude value")
    
    args = parser.parse_args()
    update_server_params(args.server_ip, args.period, args.amplitude)
