# Project KAT Firmware — SITL 환경 설정 (Ubuntu 22.04)

아무것도 설치되지 않은 Ubuntu 22.04 에서 이 문서를 위에서부터 그대로 따라 하면,
이 레포의 펌웨어 (ArduCopter 4.6.2 + TDCN 비행모드) 를 Gazebo SITL 로 돌리고
Mission Planner 와 TDCN 파이썬 스크립트 (GCS 모사, 실시간 그림, 로그 분석) 까지
연결할 수 있다.

- 명령은 모두 일반 사용자 터미널에서 실행한다.  `sudo` 가 붙은 줄만 비밀번호를 묻는다.
- 처음 한 번만 하면 되는 설치이고, 인터넷 연결이 필요하다 (전부 합쳐 30분 ~ 1시간).
- 경로는 `~/Desktop/KAT_v4/ardupilot` 기준으로 적었다.  다른 곳에 받아도 되지만
  그러면 아래 명령의 경로를 같이 바꿔야 한다.


## 검증한 버전

이 조합으로 동작을 확인했다.

| 항목 | 버전 |
|---|---|
| Ubuntu | 22.04.5 LTS |
| 펌웨어 | ArduCopter V4.6.2 + TDCN (이 레포 `main`) |
| Python | 3.10.12 (Ubuntu 기본) |
| Gazebo | Harmonic (gz-sim 8.15.0) |
| ardupilot_gazebo 플러그인 | `082a0fe` (2026-04-02) |
| Mission Planner | 1.3.83, Mono 6.8.0.105 |
| Python 패키지 | pymavlink 2.4.49, MAVProxy 1.8.74, numpy 1.26.4, matplotlib 3.5.1, pyqtgraph 0.14.0, PyQt5 5.15.11 |


## 설치 순서

1. 기본 도구
2. 펌웨어 받기
3. ArduPilot 빌드 환경
4. 펌웨어 빌드
5. Gazebo Harmonic
6. ardupilot_gazebo 플러그인
7. TDCN 스크립트용 파이썬 패키지
8. Mission Planner
9. 처음 실행해 보기


## 1. 기본 도구

```bash
sudo apt update
sudo apt install -y git curl wget unzip lsb-release gnupg
```


## 2. 펌웨어 받기

```bash
mkdir -p ~/Desktop/KAT_v4
cd ~/Desktop/KAT_v4
git clone https://github.com/Jung-Universe/Project_KAT_Firmware.git ardupilot
cd ardupilot
git submodule update --init --recursive
```

서브모듈 (mavlink, waf, DroneCAN 등) 은 ArduPilot 원본 저장소에서 받아 온다.
마지막 줄이 빠지면 4 단계 빌드가 실패한다.


## 3. ArduPilot 빌드 환경

ArduPilot 이 제공하는 설치 스크립트를 그대로 쓴다.  컴파일러, SITL 용 패키지,
pymavlink / MAVProxy 를 설치하고, `~/.profile` 에 이 트리의 `Tools/autotest` 를
PATH 로 추가한다.

```bash
cd ~/Desktop/KAT_v4/ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

끝나면 **로그아웃 후 다시 로그인** 한다 (PATH 와 그룹 설정이 모든 터미널에 적용된다).

> 이 PC 에 다른 ArduPilot 트리가 이미 있으면 `~/.profile` 의 PATH 에 그 트리가
> 먼저 들어 있을 수 있다.  `sim_vehicle.py` 는 자기가 들어 있는 트리를 빌드하고
> 돌리므로, 이 문서에서는 항상 `./Tools/autotest/sim_vehicle.py` 처럼 이 트리의
> 것을 직접 실행한다.


## 4. 펌웨어 빌드

```bash
cd ~/Desktop/KAT_v4/ardupilot
./waf configure --board sitl
./waf copter
```

`'copter' finished successfully` 가 나오면 된다 (처음은 몇 분 걸린다).
`configure` 는 처음 한 번만 하고, 코드를 고친 뒤에는 `./waf copter` 만 다시 한다.


## 5. Gazebo Harmonic

Gazebo 공식 저장소에서 설치한다.

```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update
sudo apt install -y gz-harmonic
```

확인:

```bash
gz sim --versions      # 8.x 가 나오면 된다
```


## 6. ardupilot_gazebo 플러그인

Gazebo 와 ArduPilot SITL 을 잇는 플러그인과 iris 모델 / 월드를 받는다.

```bash
sudo apt install -y libgz-sim8-dev rapidjson-dev
sudo apt install -y libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl

mkdir -p ~/gz_ws/src
cd ~/gz_ws/src
git clone https://github.com/ArduPilot/ardupilot_gazebo
cd ardupilot_gazebo
git checkout 082a0fe          # 검증한 버전.  최신으로 쓰려면 이 줄을 건너뛴다

export GZ_VERSION=harmonic
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
```

Gazebo 가 플러그인과 모델을 찾도록 환경 변수를 `~/.bashrc` 에 넣는다.

```bash
echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/gz_ws/src/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/gz_ws/src/ardupilot_gazebo/models:$HOME/gz_ws/src/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc
source ~/.bashrc
```

확인 (활주로 위에 iris 가 보이면 된다.  확인 후 창을 닫는다):

```bash
gz sim -v4 -r iris_runway.sdf
```

> NVIDIA 그래픽이 따로 있는 노트북 (하이브리드 그래픽) 에서 화면이 느리거나
> 깨지면 앞에 환경 변수를 붙여 NVIDIA 로 그리게 한다.
>
> ```bash
> __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia gz sim -v4 -r iris_runway.sdf
> ```


## 7. TDCN 스크립트용 파이썬 패키지

pymavlink / MAVProxy / matplotlib 은 3 단계에서 이미 설치됐다.  실시간 그림
(`tdcn_live.py`) 에 필요한 pyqtgraph 와 PyQt5 를 더한다.

```bash
python3 -m pip install --user "numpy<2" pyqtgraph==0.14.0 PyQt5==5.15.11
```

**`"numpy<2"` 를 빼면 안 된다.**  pyqtgraph 는 numpy 1.25 이상을 요구해서, 그냥
설치하면 pip 가 numpy 를 2.x 로 올린다.  그런데 3 단계에서 apt 로 들어온
matplotlib 3.5.1 은 numpy 1.x 용이라, numpy 가 2.x 가 되면 `tdcn_log_analyze.py`
가 `ImportError: numpy.core.multiarray failed to import` 로 멈춘다.

확인:

```bash
python3 -c "import numpy, matplotlib.pyplot, pyqtgraph, pymavlink; print('OK', numpy.__version__)"
# OK 1.26.4
```


## 8. Mission Planner

Ubuntu 에서는 Mono 로 Windows 판을 실행한다.

```bash
sudo apt install -y mono-complete
mkdir -p ~/MissionPlanner
cd ~/MissionPlanner
wget https://firmware.ardupilot.org/Tools/MissionPlanner/MissionPlanner-1.3.83.zip
unzip MissionPlanner-1.3.83.zip
mono MissionPlanner.exe
```

`MissionPlanner.exe` 가 `~/MissionPlanner` 바로 아래에 있어야 한다 (압축 안에
폴더가 한 겹 더 있으면 그 안으로 들어가 실행한다).  처음 실행은 느리다.


## 9. 처음 실행해 보기

터미널을 여러 개 띄워 아래 순서로 실행한다.

| 순서 | 무엇 | 포트 |
|---|---|---|
| 1 | Gazebo | — |
| 2 | SITL + MAVProxy | 14550 (Mission Planner), 14551, 14560 으로 내보낸다 |
| 3 | Mission Planner | UDP 14550 |
| 4 | `tdcn_gcs_NEU.py` (TDCN 명령) | UDP 14551 |
| 5 | `tdcn_live.py` (실시간 그림) | UDP 14560 |

**터미널 1 — Gazebo**

```bash
gz sim -v4 -r iris_runway.sdf
```

**터미널 2 — SITL**

```bash
cd ~/Desktop/KAT_v4/ardupilot
./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console \
    --out=udp:127.0.0.1:14551 --out=udp:127.0.0.1:14560 -w
```

- 처음 한 번은 맨 뒤 `-w` 로 파라미터를 기본값으로 만든다.  다음부터는 `-w` 를
  빼야 바꾼 파라미터가 유지된다.
- 14550 은 `sim_vehicle.py` 가 기본으로 열어 주므로 `--out` 에 다시 넣지 않는다
  (넣으면 Mission Planner 가 모든 메시지를 두 번씩 받는다).
- 이 펌웨어가 맞는지는 MAVProxy 창 (터미널 2) 에서 확인한다.
  `param show TDCN_LIVE_HZ` 가 `10` 이면 이 레포의 TDCN v4 펌웨어다.

**터미널 3 — Mission Planner**

```bash
mono ~/MissionPlanner/MissionPlanner.exe
```

오른쪽 위 연결 방식을 `UDP` 로 고르고 CONNECT → 포트 `14550`.

**터미널 4 — TDCN 명령 (GCS 모사)**

TDCN 은 비행모드 29 번이다.  Mission Planner 의 모드 목록에는 이름이 없으므로
MAVProxy 창 (터미널 2) 에서 번호로 바꾼다.

```text
mode 29
```

그다음 스크립트를 띄워 state 번호를 입력한다 (1 부터 순서대로, 또는 12 = 1~6
자동 진행).  자세한 사용법은 `TDCN/tdcn_gcs_NEU.py` 맨 위 설명에 있다.

```bash
cd ~/Desktop/KAT_v4/ardupilot/TDCN
python3 tdcn_gcs_NEU.py
```

**터미널 5 — 실시간 그림**

```bash
cd ~/Desktop/KAT_v4/ardupilot/TDCN
python3 tdcn_live.py
```

TDCN 에 들어가기 전에는 창 세 개에 "TDCN 진입 대기 중" 이 뜨고, 들어가면 그림이
그려진다.  휠 = 확대 (그 창은 따라가기를 멈춘다), Space = 최신 구간으로 복귀.

**비행 후 — 로그 분석**

```bash
cd ~/Desktop/KAT_v4/ardupilot/TDCN
python3 tdcn_log_analyze.py          # ardupilot/logs 의 가장 최근 .BIN
```


## 다음부터 실행할 때

```bash
# 1. Gazebo
gz sim -v4 -r iris_runway.sdf

# 2. SITL (-w 없이)
cd ~/Desktop/KAT_v4/ardupilot
./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console \
    --out=udp:127.0.0.1:14551 --out=udp:127.0.0.1:14560

# 3. Mission Planner (UDP 14550)
mono ~/MissionPlanner/MissionPlanner.exe

# 4. TDCN 스크립트
cd ~/Desktop/KAT_v4/ardupilot/TDCN
python3 tdcn_gcs_NEU.py
python3 tdcn_live.py
python3 tdcn_log_analyze.py
```

코드를 고쳤으면 SITL 을 띄우기 전에 `./waf copter` 로 다시 빌드한다.


## 문제 해결

| 증상 | 원인 / 해결 |
|---|---|
| `tdcn_log_analyze.py` 가 `numpy.core.multiarray failed to import` | numpy 가 2.x 로 올라갔다.  `python3 -m pip install --user "numpy<2"` |
| `param show TDCN_LIVE_HZ` 에 값이 없다 / TDCN 이 이상하게 동작한다 | 다른 ArduPilot 트리가 돌고 있다.  `./Tools/autotest/sim_vehicle.py` 로 실행했는지, `~/.profile` 의 PATH 를 확인한다 |
| SITL 이 `No JSON sensor message received` 만 반복한다 | Gazebo 가 안 떠 있거나 일시정지 상태다.  `gz sim` 을 `-r` 로 띄웠는지 확인한다 |
| Gazebo 에 iris 가 없다 / 플러그인을 못 찾는다 | 6 단계 환경 변수가 이 터미널에 없다.  새 터미널을 열거나 `source ~/.bashrc` |
| Mission Planner 가 연결되지 않는다 | 연결 방식이 UDP, 포트 14550 인지 확인한다.  SITL 이 먼저 떠 있어야 한다 |
| `tdcn_gcs_NEU.py` / `tdcn_live.py` 가 수신 대기만 한다 | SITL 명령에 `--out=udp:127.0.0.1:14551` / `14560` 을 넣었는지 확인한다 |
| `Address already in use` / 5760 포트 사용 중 | 다른 SITL 이 떠 있다.  끄거나 `-I1` 로 다른 인스턴스 번호를 준다 (포트가 10 씩 밀린다) |


## 알려진 문제

- **Gazebo iris 에서 아두파일럿 자체의 roll 이 약 16 Hz 로 진동한다.**  지금까지의
  TDCN SITL 비행 전부 (CLAW 를 끈 `TDCN_CLAW_ON_OFF = 0` 포함) 에서 이륙부터 roll 만
  흔들리고 pitch 는 깨끗하다.  iris 모델은 roll 관성이 pitch 의 약 절반이고 로터
  팔이 옆으로 더 길어서, roll / pitch 가 같은 기본 게인 (`ATC_RAT_RLL_*` =
  `ATC_RAT_PIT_*`) 이면 roll 쪽 루프 이득이 3 배쯤 크다.  CLAW 게인을 판단하기
  전에 roll rate 게인부터 이 모델에 맞춰야 한다.
