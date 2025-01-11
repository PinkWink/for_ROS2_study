# Pycam install

### Dependency

```
sudo apt install -y libcamera-dev libepoxy-dev libjpeg-dev libtiff5-dev libpng-dev
sudo apt install -y qtbase5-dev libqt5core5a libqt5gui5 libqt5widgets5
sudo apt install libavcodec-dev libavdevice-dev libavformat-dev libswresample-dev
sudo apt install -y libboost-dev
sudo apt install -y libgnutls28-dev openssl libtiff5-dev pybind11-dev
sudo apt install -y qtbase5-dev libqt5core5a libqt5gui5 libqt5widgets5
sudo apt install -y meson cmake
sudo apt install -y python3-yaml python3-ply
sudo apt install -y libglib2.0-dev libgstreamer-plugins-base1.0-dev
```

### Dependency
```
sudo apt install -y cmake libboost-program-options-dev libdrm-dev libexif-dev
sudo apt install -y meson ninja-build
sudo apt install -y libavcodec-dev
```

### rpicam-apps build
```
cd rpicam-apps
meson setup build -Denable_libav=enabled -Denable_drm=enabled -Denable_egl=enabled -Denable_qt=enabled -Denable_opencv=disabled -Denable_tflite=disabled -Denable_hailo=disabled
meson compile -C build
sudo meson install -C build
```

### 99-pinky_camera.rules
```
SUBSYSTEM=="video4linux" KERNEL=="video*", MODE="0666"
SUBSYSTEM=="media" KERNEL=="media*", MODE="0666"
SUBSYSTEM=="video4linux" KERNEL=="v4l-subdev*", MODE="0666"
SUBSYSTEM=="dma_heap", MODE="0666"
```

### Dependency
```
sudo apt install -y python3-libcamera
sudo apt install -y python3-pyqt5 python3-prctl libatlas-base-dev ffmpeg python3-pip
```

### kms 설치
```
sudo apt install -y  libfmt-dev libdrm-dev libfmt-dev
git clone https://github.com/tomba/kmsxx.git
cd kmsxx/
git submodule update --init
meson build -Dpykms=enabled
sudo ninja -C build install
```