// Arduino Incremental Serial Sender

int counter = 0;          // 전송할 숫자를 저장할 변수

void setup() {
    Serial.begin(9600);     // 시리얼 통신 시작 (9600 bps)
    while (!Serial) {
        ; // 시리얼 포트가 연결될 때까지 대기 (일부 보드에서만 필요)
    }
}

void loop() {
    Serial.println(counter); // 현재 숫자를 시리얼로 전송 (뒤에 줄바꿈 포함)
    counter++;               // 숫자 1 증가
    delay(1000);             // 1초(1000ms) 대기
}
