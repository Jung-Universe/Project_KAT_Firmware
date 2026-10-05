# Project KAT Firmware

ArduPilot 4.6.2 (ArduCopter) 기반 펌웨어.  TDCN 비행모드 (29 번) 와 CLAW IBSC 제어기가
들어 있다.  아무것도 설치되지 않은 Ubuntu 22.04 에서 이 문서를 1 번부터 6 번까지
그대로 따라 하면 Gazebo SITL 로 TDCN 을 돌리고 Mission Planner 와 TDCN 파이썬
스크립트까지 연결할 수 있다.

| 위치 | 내용 |
|---|---|
| `ArduCopter/mode_tdcn*.cpp` | TDCN 비행모드 (시나리오 state 0 ~ 13, 로그, 실시간 송신) |
| `ArduCopter/mode_tdcn_CLAW_*` | CLAW IBSC 제어기와 게인 |
| `TDCN/` | GCS 모사, 실시간 그림, 로그 분석 스크립트 |
| `TDCN/command_for_IBSC.txt` | 실행 명령 모음 (SITL / 실기체 Windows GCS) |


## 1. 필요한 것과 버전

이 조합으로 동작을 확인했다.  다른 버전도 대부분 되지만, 막히면 이 버전에 맞춘다.

| 항목 | 버전 | 설치 위치 (이 문서 기준) |
|---|---|---|
| Ubuntu | 22.04.5 LTS | — |
| 펌웨어 | ArduCopter V4.6.2 + TDCN (이 레포 `tdcn-v4` 브랜치) | `~/Desktop/KAT_v4/ardupilot` |
| Python | 3.10.12 (Ubuntu 기본) | — |
| Gazebo | Harmonic (gz-sim 8.15.0) | apt |
| ardupilot_gazebo 플러그인 | `082a0fe` (2026-04-02) | `~/gz_ws/src/ardupilot_gazebo` |
| Mission Planner | 1.3.83 (Mono 6.8.0.105 로 실행) | `~/MissionPlanner` |
| pymavlink / MAVProxy | 2.4.49 / 1.8.74 | pip (2 단계에서 자동) |
| numpy | 1.26.4 (**2 미만으로 고정**, 5 단계 참고) | pip |
| matplotlib | 3.5.1 | apt (2 단계에서 자동) |
| pyqtgraph / PyQt5 | 0.14.0 / 5.15.11 | pip |

- 명령은 모두 일반 사용자 터미널에서 실행한다.  `sudo` 가 붙은 줄만 비밀번호를 묻는다.
- 처음 설치는 인터넷 연결이 필요하고 전부 합쳐 30분 ~ 1시간 걸린다.
- 다른 경로에 받아도 되지만, 그러면 아래 명령의 경로를 같이 바꿔야 한다.


## 2. SITL (펌웨어 빌드와 실행)

### 2-1. 기본 도구

```bash
sudo apt update
sudo apt install -y git curl wget unzip lsb-release gnupg
```

### 2-2. 펌웨어 받기

```bash
mkdir -p ~/Desktop/KAT_v4
cd ~/Desktop/KAT_v4
git clone https://github.com/Jung-Universe/Project_KAT_Firmware.git ardupilot
cd ardupilot
git submodule update --init --recursive
```

서브모듈 (mavlink, waf, DroneCAN 등) 은 ArduPilot 원본 저장소에서 받아 온다.
마지막 줄이 빠지면 빌드가 실패한다.

### 2-3. ArduPilot 빌드 환경

ArduPilot 이 제공하는 설치 스크립트를 쓴다.  컴파일러, SITL 용 패키지,
pymavlink / MAVProxy / matplotlib 을 설치하고, `~/.profile` 에 이 트리의
`Tools/autotest` 를 PATH 로 추가한다.

```bash
cd ~/Desktop/KAT_v4/ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

끝나면 **로그아웃 후 다시 로그인** 한다 (PATH 와 그룹 설정이 모든 터미널에 적용된다).

### 2-4. 빌드

```bash
cd ~/Desktop/KAT_v4/ardupilot
./waf configure --board sitl
./waf copter
```

`'copter' finished successfully` 가 나오면 된다 (처음은 몇 분 걸린다).
`configure` 는 처음 한 번만, 코드를 고친 뒤에는 `./waf copter` 만 다시 한다.

### 2-5. SITL 단독 실행 확인 (Gazebo 없이)

ArduPilot 내장 물리 모델로 먼저 띄워 빌드가 맞는지 본다.

```bash
cd ~/Desktop/KAT_v4/ardupilot
./Tools/autotest/sim_vehicle.py -v ArduCopter --console -w
```

MAVProxy 창 (`MAV>` 프롬프트) 에서:

```text
param show TDCN_LIVE_HZ
```

`TDCN_LIVE_HZ 10.0` 이 나오면 이 레포의 TDCN v4 펌웨어가 도는 것이다.  확인했으면
`Ctrl+C` 로 끈다.

> `sim_vehicle.py` 는 자기가 들어 있는 트리를 빌드하고 돌린다.  PC 에 다른
> ArduPilot 트리가 있으면 PATH 에 그쪽이 먼저 잡힐 수 있으므로, 이 문서는 항상
> `./Tools/autotest/sim_vehicle.py` 처럼 이 트리의 것을 직접 실행한다.


## 3. Gazebo

### 3-1. Gazebo Harmonic 설치

```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update
sudo apt install -y gz-harmonic
gz sim --versions      # 8.x 가 나오면 된다
```

### 3-2. ardupilot_gazebo 플러그인

Gazebo 와 ArduPilot SITL 을 잇는 플러그인과 iris 모델 / 월드다.

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

### 3-3. Gazebo 실행

```bash
gz sim -v4 -r iris_runway.sdf
```

활주로 위에 iris 가 보이면 된다.  `-r` 은 열자마자 시뮬레이션을 시작한다는 뜻이다
(빼면 왼쪽 아래 재생 버튼을 눌러야 SITL 이 붙는다).

> NVIDIA 그래픽이 따로 있는 노트북 (하이브리드 그래픽) 에서 화면이 느리거나
> 깨지면 앞에 환경 변수를 붙여 NVIDIA 로 그리게 한다.
>
> ```bash
> __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia gz sim -v4 -r iris_runway.sdf
> ```

### 3-4. SITL 과 연결 확인

Gazebo 를 띄운 채로 다른 터미널에서:

```bash
cd ~/Desktop/KAT_v4/ardupilot
./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console -w
```

MAVProxy 창에 `EKF3 IMU0 is using GPS` 가 나온 뒤 (1분 정도) 아래를 치면 Gazebo
의 iris 가 떠오른다.

```text
mode guided
arm throttle
takeoff 5
```

확인했으면 `mode land` 로 내리고 둘 다 끈다.  처음 한 번은 `-w` 로 파라미터를
gazebo-iris 기본값으로 만들고, 다음부터는 `-w` 를 뺀다 (빼야 바꾼 파라미터가 유지된다).


## 4. Mission Planner

Ubuntu 에서는 Mono 로 Windows 판을 실행한다.

```bash
sudo apt install -y mono-complete
mkdir -p ~/MissionPlanner
cd ~/MissionPlanner
wget https://firmware.ardupilot.org/Tools/MissionPlanner/MissionPlanner-1.3.83.zip
unzip MissionPlanner-1.3.83.zip
mono MissionPlanner.exe
```

- `MissionPlanner.exe` 가 `~/MissionPlanner` 바로 아래에 있어야 한다 (압축 안에 폴더가
  한 겹 더 있으면 그 안으로 들어가 실행한다).  처음 실행은 느리다.
- **연결**: SITL 을 띄운 뒤 오른쪽 위 연결 방식을 `UDP` 로 고르고 CONNECT → 포트
  `14550`.  `sim_vehicle.py` 가 14550 은 기본으로 열어 주므로 SITL 쪽에 따로 넣지 않는다.
- TDCN (29 번) 은 Mission Planner 모드 목록에 이름이 없다.  모드 전환은 MAVProxy
  창에서 `mode 29` 로 한다 (5 단계).


## 5. 파이썬 스크립트 (TDCN/)

### 5-1. 설치

pymavlink / MAVProxy / matplotlib 은 2-3 단계에서 이미 들어왔다.  실시간 그림에
필요한 pyqtgraph 와 PyQt5 를 더한다.

```bash
python3 -m pip install --user "numpy<2" pyqtgraph==0.14.0 PyQt5==5.15.11
python3 -c "import numpy, matplotlib.pyplot, pyqtgraph, pymavlink; print('OK', numpy.__version__)"
# OK 1.26.4
```

**`"numpy<2"` 를 빼면 안 된다.**  pyqtgraph 가 numpy 1.25 이상을 요구해서 그냥
설치하면 pip 가 numpy 를 2.x 로 올리는데, apt 의 matplotlib 3.5.1 은 numpy 1.x 용이라
`tdcn_log_analyze.py` 가 `ImportError: numpy.core.multiarray failed to import` 로 멈춘다.

### 5-2. 스크립트

| 스크립트 | 하는 일 | 받는 포트 |
|---|---|---|
| `tdcn_gcs_NEU.py` | GCS 모사.  TDCN state 명령과 state 6 타겟 (home 기준 N/E, 헤딩, 고도) 전송 | UDP 14551 |
| `tdcn_live.py` | 비행 중 실시간 그림 (현재 상태 / 목표 / 믹서 입력 3 장) | UDP 14560 |
| `tdcn_log_analyze.py` | 비행 후 .BIN 로그 분석 (같은 3 장 + 요약) | — |
| `tdcn_gcs_NEU_ship.py` | 선박 탑재 GCS 모사 (state 6 에서 배 궤적을 계속 전송) | UDP 14551 |
| `tdcn_ship_log_analyze.py` | 선박 추종 비행 로그 분석 | — |

스크립트가 받는 포트는 SITL 을 띄울 때 `--out` 으로 열어 줘야 한다 (6 단계 명령에
들어 있다).  각 스크립트의 자세한 옵션은 파일 맨 위 설명과 `--help` 에 있다.

### 5-3. 사용법

**`tdcn_gcs_NEU.py` — TDCN 명령**

1. MAVProxy 창에서 `mode 29` 로 TDCN 에 들어간다 (지상에서 들어가면 state 0 부터).
2. 스크립트를 띄우고 state 번호를 차례로 입력한다.

```bash
cd ~/Desktop/KAT_v4/ardupilot/TDCN
python3 tdcn_gcs_NEU.py
```

- `1` ~ `11`: 시나리오 state.  바로 다음 번호만 받는다 (현재 단계가 끝나야 넘어간다).
- `6`: 추종 비행.  타겟 N / E (m), 헤딩 (deg), 고도 (m) 를 추가로 입력한다.
- `12`: state 1 ~ 6 자동 진행, `13`: state 6 ~ 11 자동 진행.
- 비행 중 Loiter 로 뺐다가 공중에서 다시 `mode 29` 로 들어오면 state 5 부터 시작하고
  home 위 (0, 0, `TDCN_TKO_ALT`), 헤딩 0 으로 돌아간다.  그다음 `6` 부터 입력한다.
- `TDCN_CLAW_ON_OFF` = 1 이면 state 6 에서 CLAW 가 기체를 몬다 (0 이면 아두파일럿).

**`tdcn_live.py` — 실시간 그림**

```bash
cd ~/Desktop/KAT_v4/ardupilot/TDCN
python3 tdcn_live.py
```

- TDCN 에 들어가기 전에는 "TDCN 진입 대기 중" 이 뜨고, 들어가면 그림이 그려진다.
- 휠 = 확대 (그 창은 최신 구간 따라가기를 멈춘다), 드래그 = 이동,
  Space = 최신 구간으로 복귀.
- 받는 주기는 펌웨어 파라미터 `TDCN_LIVE_HZ` (기본 10 Hz).  SITL 에서는 50 까지 올려도 된다.

**`tdcn_log_analyze.py` — 로그 분석**

```bash
cd ~/Desktop/KAT_v4/ardupilot/TDCN
python3 tdcn_log_analyze.py                       # ardupilot/logs 의 가장 최근 .BIN
python3 tdcn_log_analyze.py 00000007.BIN          # 특정 로그
python3 tdcn_log_analyze.py --start 40 --end 70   # 구간 (s, TDCN 진입 기준)
```


## 6. 최종 SITL 실행 (총 명령어)

설치가 끝난 뒤 매번 이 순서로 띄운다.  터미널 하나에 하나씩.

```bash
# 터미널 1 — Gazebo
gz sim -v4 -r iris_runway.sdf

# 터미널 2 — SITL + MAVProxy  (14550 은 기본, 14551 / 14560 은 스크립트용)
cd ~/Desktop/KAT_v4/ardupilot
./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console \
    --out=udp:127.0.0.1:14551 --out=udp:127.0.0.1:14560
#   파라미터를 기본값으로 되돌릴 때만 맨 뒤에 -w

# 터미널 3 — Mission Planner  (UDP 14550 으로 CONNECT)
mono ~/MissionPlanner/MissionPlanner.exe

# 터미널 4 — TDCN 명령  (먼저 터미널 2 의 MAVProxy 에서 mode 29)
cd ~/Desktop/KAT_v4/ardupilot/TDCN
python3 tdcn_gcs_NEU.py

# 터미널 5 — 실시간 그림
cd ~/Desktop/KAT_v4/ardupilot/TDCN
python3 tdcn_live.py

# 비행 후 — 로그 분석
cd ~/Desktop/KAT_v4/ardupilot/TDCN
python3 tdcn_log_analyze.py
```

코드를 고쳤으면 터미널 2 를 띄우기 전에 `./waf copter` 로 다시 빌드한다.
실기체 (Windows GCS) 명령은 [TDCN/command_for_IBSC.txt](TDCN/command_for_IBSC.txt) 에 있다.


## 라이선스

ArduPilot ([github.com/ArduPilot/ardupilot](https://github.com/ArduPilot/ardupilot)) 을
기반으로 하며 GNU GPL v3 를 따른다.  전문은 [COPYING.txt](COPYING.txt).
