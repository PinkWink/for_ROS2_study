import tensorflow as tf

# 사용 가능한 GPU 확인 (CUDA 장치가 있다면 자동으로 사용됩니다)
gpus = tf.config.list_physical_devices('GPU')
if gpus:
    print("GPU devices available:", gpus)
else:
    print("No GPU devices available.")

# MNIST 데이터셋 불러오기
mnist = tf.keras.datasets.mnist
(x_train, y_train), (x_test, y_test) = mnist.load_data()

# 채널 차원 추가: (28, 28) -> (28, 28, 1)
x_train = x_train.reshape((-1, 28, 28, 1))
x_test = x_test.reshape((-1, 28, 28, 1))

# 데이터 정규화: 0~1 범위
x_train, x_test = x_train / 255.0, x_test / 255.0

# LeNet 유사 CNN 모델 생성
model = tf.keras.models.Sequential([
    # 첫 번째 Convolution 레이어: 32개의 필터, 5x5 커널, 'same' padding
    tf.keras.layers.Conv2D(filters=32, kernel_size=(5, 5), activation='relu', input_shape=(28, 28, 1), padding='same'),
    # 평균 풀링 레이어 (2x2 풀 사이즈)
    tf.keras.layers.AveragePooling2D(pool_size=(2, 2)),
    
    # 두 번째 Convolution 레이어: 16개의 필터, 5x5 커널
    tf.keras.layers.Conv2D(filters=16, kernel_size=(5, 5), activation='relu'),
    # 두 번째 평균 풀링 레이어 (2x2 풀 사이즈)
    tf.keras.layers.AveragePooling2D(pool_size=(2, 2)),
    
    # Flatten하여 Fully-connected 레이어로 연결
    tf.keras.layers.Flatten(),
    # 첫 번째 Dense 레이어: 120 유닛
    tf.keras.layers.Dense(120, activation='relu'),
    # 두 번째 Dense 레이어: 84 유닛
    tf.keras.layers.Dense(84, activation='relu'),
    # 출력 Dense 레이어: 10 클래스 (softmax)
    tf.keras.layers.Dense(10, activation='softmax')
])

# 모델 컴파일: Adam 옵티마이저, sparse categorical crossentropy 손실 함수 사용
model.compile(optimizer='adam',
                loss='sparse_categorical_crossentropy',
                metrics=['accuracy'])

# 모델 학습
model.fit(x_train, y_train, epochs=5, batch_size=32)

# 모델 평가
test_loss, test_acc = model.evaluate(x_test, y_test, verbose=2)
print('Test accuracy:', test_acc)
