# 상류 패키지 패치

apt로 받은 ROS 패키지의 결함을 고친 것들이다. 워크스페이스 오버레이로 빌드해
`/opt/ros/jazzy`의 바이너리를 덮는다. 상류가 고쳐지면 이 디렉터리에서 지운다.

## `usb_cam-0.8.1-epoch-time-shift.patch`

**증상.** `/camera/image_raw`의 `header.stamp`가 실제 촬영 시각보다 과거로
찍힌다. 한 세션 안에서는 ±5 ms로 고정이고, 노드를 다시 띄울 때마다 값이
바뀐다. 실측 0.284 / 0.406 / 0.496 / 0.510 / 0.545 / 0.812 / 0.920 s —
전부 [0, 1) 구간이다.

**영향.** 이 stamp는 `aruco_opencv` → `marker_track` → `follow`까지 그대로
전달된다. `follow`의 `marker_timeout_s = 0.4` 신선도 게이트가 정상 표본을
전부 stale로 기각해 **추종이 아예 출력을 내지 못한다.**

**원인.** `utils.hpp`의 `get_epoch_time_shift_us()`에서 `epoch_time.tv_usec`는
이미 마이크로초인데 1000으로 나눠 밀리초로 만든 뒤 마이크로초 합계에 더한다.
그래서 `epoch_us`가 노드 기동 순간의 소수점 이하 초만큼 작아지고, 그 오차가
모든 프레임의 stamp에 상수로 실린다. 값이 세션마다 달라지는 이유가 이것이다.

**결백한 것들.** 커널·드라이버·카메라는 정상이다. `v4l2-ctl --stream-mmap
--verbose`로 재면 C920은 1280x720 MJPG에서 4버퍼·32 ms 간격으로 정확히 30 fps를
내고, DQBUF 시점의 버퍼 타임스탬프 지연은 **8 ms**다. `uvcvideo`는
`hwtimestamps=0`, `clock=CLOCK_MONOTONIC`이고 chrony는 NTP 대비 0.3 ms다.
부하도 아니다 — 검출기를 끄고 usb_cam만 띄우면 설정 프레임률을 100% 따라가면서
지연은 오히려 더 컸다. libavcodec 경로도 아니다 — `pixel_format: raw_mjpeg`으로
디코드를 빼도 같은 값이 나온다.

**수정 후.** `/camera/image_raw` stamp 나이 0.51 → **0.043 s**,
`/marker_track` 0.5~0.9 → **0.034 s**. 검출률·해상도·초점은 그대로다.

**적용.**

```bash
# rpi5
cd ~/opt && git clone --depth 1 -b 0.8.1 https://github.com/ros-drivers/usb_cam.git usb_cam_src
cp -r ~/opt/usb_cam_src ~/ros2_ws/src/usb_cam && rm -rf ~/ros2_ws/src/usb_cam/.git
cd ~/ros2_ws/src/usb_cam && patch -p1 < <저장소>/software/patches/usb_cam-0.8.1-epoch-time-shift.patch
cd ~/ros2_ws && colcon build --packages-select usb_cam \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

빌드 의존은 `libavcodec-dev`·`libavutil-dev`·`libswscale-dev`뿐이고 rpi5에 이미
있다. **sudo가 필요 없다.** 오버레이가 먹었는지는
`ros2 pkg prefix usb_cam`이 `~/ros2_ws/install/usb_cam`을 가리키는지로 본다.

**대안을 버린 이유.** `v4l2_camera` 0.7.1은 stamp를 정직하게 찍지만 지원 인코딩이
`mono8`/`rgb8`/`yuv422*`뿐이라 MJPG를 디코드하지 못한다. C920은 1280x720에서
YUYV가 최대 10 fps(MJPG만 30 fps)라, 교체하면 20 Hz 제어 루프에 못 미친다.
