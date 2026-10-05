#!/usr/bin/env python3
"""
TDCN 실시간 그림 (v4)  /* Sejong */

비행 중에 tdcn_log_analyze.py 의 Figure 1~3 을 pyqtgraph 로 띄운다.  축 / 선 /
배경을 처음에 한 번 만들고 매 프레임 선 데이터만 바꿔 끼우므로 초당 20 번
정도 바뀐다.

    Figure 1   현재 상태 — 위치 / 속도 / 자세
    Figure 2   목표 — 위치 / 헤딩.  현재값을 겹쳐 그린다
    Figure 3   믹서 직전 제어값 — 아두파일럿 vs CLAW

색 / 범례 문구와 구간 계산 (TDCN 진입 기준 시간, 축 폭 통일, 헤딩 범위, 음영,
단계 전환) 은 tdcn_log_analyze.py 의 것을 그대로 쓴다.  보고서용 그림은 지금처럼
착륙 후 tdcn_log_analyze.py 로 뽑는다 (모양은 비슷하게 맞췄지만 같지는 않다).


데이터
------
펌웨어 (mode_tdcn_log.cpp, send_tdcn_live) 가 TDST / TDTG / TDMX 와 같은 값을
DEBUG_FLOAT_ARRAY 하나 (name "TDCN", array_id 1) 로 묶어 TDCN_LIVE_HZ (기본
10 Hz) 로 보낸다.  여기서 받아 read_log() 와 같은 모양으로 쌓아 그린다.

로그 분석과 다른 점
    - 샘플이 TDCN_LIVE_HZ (기본 10 Hz) 다.  로그 (400 Hz) 보다 성기므로 빠른
      진동 (예: yaw limit cycle) 은 뭉개지거나 다른 모양으로 보인다.  진동의
      모양은 착륙 후 SD 로그로 본다.
    - 구간은 TDCN 첫 진입 ~ 지금.  이탈한 뒤에도 계속 그린다 (회색 음영).
    - --window 를 주면 최근 N 초만 그린다.


연결
----
텔레메트리 포트는 한 프로그램만 열 수 있으므로 MAVProxy 가 UDP 로 나눠 준다.
이 스크립트는 그중 한 포트 (기본 14560) 를 듣는다.

    SITL     sim_vehicle.py ... --out=udp:127.0.0.1:14560
    실기체   mavproxy.py --master=/dev/ttyUSB0,57600 \\
                 --out=udp:127.0.0.1:14550 \\      # Mission Planner / QGC
                 --out=udp:127.0.0.1:14551 \\      # tdcn_gcs_NEU.py
                 --out=udp:127.0.0.1:14560         # tdcn_live.py

텔레메트리 대역폭: 한 통에 약 136 byte, 10 Hz 면 약 1.4 kB/s.  SiK 57600 에서는
기본 스트림 (SRx_*) 과 나눠 쓴다.


설치
----
    pip install pyqtgraph PyQt5


조작 (창마다 따로)
----
    휠             확대 / 축소.  그림 위에서는 x / y 둘 다, 축 위에서는 그 축만.
                   한 창 안의 x 축은 묶여 있다
    왼쪽 드래그    이동
    오른쪽 드래그  축별로 늘이고 줄이기
    Space          최신 구간 따라가기 멈춤 / 재개

    휠이나 드래그로 손대는 순간 그 창은 최신 구간 따라가기를 멈춘다.  데이터는
    계속 들어와 그려지고, 화면 범위만 그대로 둔다.  Space 를 누르면 다시 최신
    구간을 따라간다.
    오른쪽 클릭 (드래그 없이) → Export... 로 지금 화면을 그림 파일로 저장


사용 예
-------
    ./tdcn_live.py                          # udp:127.0.0.1:14560 을 듣는다
    ./tdcn_live.py --window 30              # 최근 30 초만
    ./tdcn_live.py -c udp:127.0.0.1:14570   # 다른 포트
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
import threading
import time
import warnings

# ArduPilot 은 MAVLink2 로 보낸다.  DEBUG_FLOAT_ARRAY (id 350) 는 MAVLink2 에만
# 있으므로 pymavlink 을 부르기 전에 정한다.
os.environ.setdefault("MAVLINK20", "1")

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# 색 / 범례 문구 / 구간 계산은 로그 분석 스크립트의 것을 그대로 쓴다
import tdcn_log_analyze as A  # noqa: E402

import numpy as np  # noqa: E402
from pymavlink import mavutil  # noqa: E402

try:
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtCore
except Exception as e:      # Qt 바인딩이 없으면 ImportError 가 아닌 Exception 이 난다
    sys.exit(f"pyqtgraph 와 Qt 가 필요합니다:  pip install pyqtgraph PyQt5\n({e})")

# 목표가 없는 구간 (전부 NaN) 을 그릴 때 numpy 가 내는 경고는 정상이라 끈다
warnings.filterwarnings("ignore", message="All-NaN")


# ---------------------------------------------------------------------------
# 스타일 — tdcn_log_analyze.py 의 색 / 선 / 범례 문구를 그대로 쓴다
# ---------------------------------------------------------------------------

SOLID = QtCore.Qt.PenStyle.SolidLine
DASH = QtCore.Qt.PenStyle.DashLine
DOT = QtCore.Qt.PenStyle.DotLine


def line_pen(key: str):
    st = A.LINE[key]
    dashed = st["ls"] == "--"
    return pg.mkPen(st["color"], width=2.0 if dashed else 1.5,
                    style=DASH if dashed else SOLID)


def shade(color: str, alpha: float):
    c = pg.mkColor(color)
    c.setAlphaF(alpha)
    return pg.mkBrush(c)


STATE_PEN = pg.mkPen(A.C_STATE, width=1, style=DOT)
ZERO_PEN = pg.mkPen("#888888", width=1, style=DOT)
BRUSH_OUT = shade(A.C_OUT, A.SHADE_OUT["alpha"])
BRUSH_ACT = shade(A.C_CLAW, A.SHADE_ACT["alpha"])

FPS = 20
SIZE_3x3 = (1500, 950)
SIZE_4x1 = (1100, 950)
AXIS_W = 60             # 왼쪽 축 폭 (px).  고정해야 위아래 축이 줄 맞는다
LABEL_GAP_PX = 24       # 단계 숫자끼리 이보다 가까우면 한 줄 내린다 (약 0.25 인치)


def legend_html(lines: list[str]) -> str:
    """범례 — 선 (그림마다 다름) 다음에 배경 (세 장 공통), 로그 분석과 같은 순서."""
    parts = []
    for k in lines:
        st = A.LINE[k]
        glyph = "- - -" if st["ls"] == "--" else "━━"
        parts.append(f"<span style='color:{st['color']}'><b>{glyph}</b></span> {st['label']}")
    parts.append(f"<span style='color:{A.C_STATE}'>┆</span> "
                 "단계 전환 (위 숫자 = 단계, 12·13 = 자동 진행 시작)")
    parts.append("<span style='color:#d9d9d9'>■</span> TDCN 밖 (Mode ≠ 29)")
    parts.append("<span style='color:#f4c7c7'>■</span> CLAW 구동 (Act = 1)")
    return "&nbsp;&nbsp;&nbsp;&nbsp;".join(parts)


# ---------------------------------------------------------------------------
# 수신
# ---------------------------------------------------------------------------

# data 순서.  mode_tdcn_log.cpp 의 send_tdcn_live() 와 같아야 한다.  펌웨어에서
# 순서를 바꾸면 array_id 를 올리고 여기 LIVE_ID 도 같이 올린다.
LIVE_NAME = "TDCN"
LIVE_ID = 1
LAYOUT = (
    ("TDST", ("Mode", "St", "Stp", "PN", "PE", "PD",
              "VN", "VE", "VD", "Roll", "Pitch", "Yaw")),
    ("TDTG", ("Val", "PN", "PE", "PD", "Hdg")),
    ("TDMX", ("Act", "AR", "AP", "AY", "AT", "CR", "CP", "CY", "CT")),
)
N_DATA = sum(len(fs) for _, fs in LAYOUT)      # 26

LOST_S = 2.0        # 이보다 오래 못 받으면 부제에 수신 끊김을 적는다


def msg_name(msg) -> str:
    n = msg.name
    if isinstance(n, bytes):
        n = n.decode("ascii", "ignore")
    return n.rstrip("\x00")


class Receiver(threading.Thread):
    """TDCN 메시지를 [t, data 0..25] 행으로 쌓는다.  t 는 부팅 후 s (로그의 TimeUS)."""

    def __init__(self, device: str):
        super().__init__(daemon=True)
        self.master = mavutil.mavlink_connection(device)
        self.lock = threading.Lock()
        self.rows: list[list[float]] = []
        self.last_rx: float | None = None      # 마지막 수신 (time.monotonic)
        self.epoch = 0                          # 재부팅으로 rows 를 비울 때마다 +1
        self._warned_id = False

    def run(self) -> None:
        while True:
            msg = self.master.recv_match(type="DEBUG_FLOAT_ARRAY",
                                         blocking=True, timeout=0.5)
            if msg is None or msg_name(msg) != LIVE_NAME:
                continue
            if msg.array_id != LIVE_ID:
                if not self._warned_id:
                    print(f"[live] array_id {msg.array_id} 를 받았습니다 (여기는 "
                          f"{LIVE_ID}).  펌웨어와 tdcn_live.py 의 data 순서가 다른 "
                          "버전입니다 — 무시합니다.")
                    self._warned_id = True
                continue

            t = msg.time_usec / 1e6
            row = [t] + list(msg.data[:N_DATA])
            with self.lock:
                if self.rows and t < self.rows[-1][0] - 1.0:
                    # 시간이 거꾸로 = 기체 재부팅.  처음부터 다시 쌓는다
                    print("[live] 기체 재부팅 — 처음부터 다시 쌓습니다")
                    self.rows.clear()
                    self.epoch += 1
                elif self.rows and t <= self.rows[-1][0]:
                    continue                    # 중복 / 순서 뒤바뀐 UDP 패킷
                if not self.rows:
                    print(f"[live] 수신 시작 (부팅 후 {t:.1f} s)")
                self.rows.append(row)
                self.last_rx = time.monotonic()

    def rows_since(self, epoch: int, n: int):
        """n 번째 이후의 새 행만.  epoch 이 다르면 (재부팅) 처음부터.
        돌려주는 값: (epoch, 새 행, 마지막 수신 시각)."""
        with self.lock:
            if epoch != self.epoch:
                n = 0
            return self.epoch, self.rows[n:], self.last_rx


# ---------------------------------------------------------------------------
# 데이터 — 수신 행을 numpy 로 이어 붙이고, 그릴 구간을 정한다
# ---------------------------------------------------------------------------

class Store:
    """Receiver 의 새 행만 가져와 배열에 붙인다.  용량을 두 배씩 늘려 매 프레임
    전체를 다시 만들지 않는다."""

    def __init__(self):
        self.epoch = -1
        self.buf = np.empty((1024, 1 + N_DATA))
        self.n = 0
        self.last_rx: float | None = None

    def pull(self, rx: Receiver) -> bool:
        """새 행이 있었거나 재부팅으로 비웠으면 True."""
        epoch, new, self.last_rx = rx.rows_since(self.epoch, self.n)
        changed = epoch != self.epoch
        if changed:
            self.epoch, self.n = epoch, 0
        if new:
            need = self.n + len(new)
            if need > len(self.buf):
                grown = np.empty((max(need, 2 * len(self.buf)), self.buf.shape[1]))
                grown[:self.n] = self.buf[:self.n]
                self.buf = grown
            self.buf[self.n:need] = new
            self.n = need
            changed = True
        return changed

    @property
    def rows(self) -> np.ndarray | None:
        return self.buf[:self.n] if self.n else None


def to_data(rows: np.ndarray) -> dict:
    """read_log() 와 같은 모양 — {메시지: {필드: 배열, "t": 배열}}."""
    data, col = {}, 1
    for name, fields in LAYOUT:
        d = {"t": rows[:, 0].copy()}
        for f in fields:
            d[f] = rows[:, col]
            col += 1
        data[name] = d
    return data


def prepare(rows: np.ndarray | None, window: float | None, connect: str):
    """수신 행 -> (data, ctx, None).  아직 그릴 게 없으면 (None, None, 안내 문구).

    구간은 TDCN 첫 진입 ~ 지금, 첫 진입을 0 초로 (로그 분석과 같다).
    """
    if rows is None or len(rows) == 0:
        return None, None, (f"수신 대기 중 ...  ({connect})\n\n"
                            "TDCN_LIVE_HZ 가 0 이 아닌지, MAVProxy --out 포트가 맞는지 확인")

    data = to_data(rows)
    s = data["TDST"]
    on = np.where(s["Mode"] == A.TDCN_MODE)[0]
    if on.size == 0 or s["t"][-1] <= s["t"][on[0]]:
        mode = int(s["Mode"][-1])
        return None, None, (f"TDCN 진입 대기 중 ...  지금 Mode {mode} "
                            f"{A.MODE_NAMES.get(mode, '?')}")

    t_entry = float(s["t"][on[0]])
    data = A.crop(data, t_entry, None)
    A.shift(data, t_entry)
    if window:
        data = A.crop(data, data["TDST"]["t"][-1] - window, None)
    ctx = A.context(data)
    ctx["t_entry"] = t_entry
    return data, ctx, None


def subtitle(rows: np.ndarray, last_rx: float, connect: str, ctx: dict) -> str:
    """부제 — 접속, 구간, 수신 Hz, 수신 끊김."""
    dt = np.diff(rows[-50:, 0])
    hz = 1.0 / float(np.median(dt)) if dt.size else 0.0
    age = time.monotonic() - last_rx
    lost = f"   [수신 끊김 {age:.0f} s]" if age > LOST_S else ""
    return (f"실시간 {connect}   {ctx['t0']:.1f} ~ {ctx['t1']:.1f} s "
            f"(TDCN 진입 = 부팅 후 {ctx['t_entry']:.1f} s)   수신 {hz:.0f} Hz{lost}")


# ---------------------------------------------------------------------------
# 한 장
# ---------------------------------------------------------------------------

class LeftAxis(pg.AxisItem):
    """y 축.  격자를 켜면 pyqtgraph 는 축 안에 다 들어오는 눈금 숫자만 그려서 끝
    눈금 (±1, Throttle 0 / 1, 헤딩 ±180 등) 의 숫자가 빠진다.  위아래로 15 px
    넘어가도 그리게 한다."""

    def boundingRect(self):
        return super().boundingRect().adjusted(0, -15, 0, 15)


class Window(pg.GraphicsLayoutWidget):
    """Space 를 받는 창."""

    def __init__(self, on_space):
        super().__init__()
        self._on_space = on_space

    def keyPressEvent(self, ev):
        if ev.text() == " ":
            self._on_space()
            ev.accept()
            return
        super().keyPressEvent(ev)


class Decor:
    """한 축의 배경 — TDCN 밖 / CLAW 구동 음영, 단계 전환선과 숫자."""

    def __init__(self, plot, label_states: bool):
        self.plot = plot
        self.label_states = label_states
        self.regions = {"out": [], "act": []}
        self.lines: list = []
        self.labels: tuple = ()
        self.levels: list = []

    def set_spans(self, kind: str, spans: list, brush) -> None:
        items = self.regions[kind]
        while len(items) < len(spans):
            r = pg.LinearRegionItem(movable=False, brush=brush, pen=pg.mkPen(None))
            r.setZValue(-10)
            self.plot.addItem(r, ignoreBounds=True)
            items.append(r)
        for r, (a, b) in zip(items, spans):
            r.setRegion((a, b))
            r.show()
        for r in items[len(spans):]:
            r.hide()

    def set_states(self, tr_t, tr_label, min_dt: float, t0: float = 0.0) -> None:
        labels = tuple(int(s) for s in tr_label)
        if labels != self.labels:
            # 단계가 바뀌었을 때만 선을 새로 만든다.  나머지는 위치만 옮긴다
            for ln in self.lines:
                self.plot.removeItem(ln)
            self.lines = []
            for tt, s in zip(tr_t, labels):
                # 그림 왼쪽 끝 (TDCN 진입 = 0 초) 의 숫자는 가운데 맞춤이면 반이
                # 잘리므로 선 오른쪽에 붙인다
                anchor = (0.0, 0) if tt <= t0 else (0.5, 0)
                ln = pg.InfiniteLine(
                    pos=float(tt), angle=90, pen=STATE_PEN, movable=False,
                    label=f"{s}" if self.label_states else None,
                    labelOpts=dict(color=A.C_STATE, position=0.95,
                                   anchors=[anchor, anchor]))
                ln.setZValue(-5)
                self.plot.addItem(ln, ignoreBounds=True)
                self.lines.append(ln)
            self.labels = labels
            self.levels = [0] * len(self.lines)
        else:
            for ln, tt in zip(self.lines, tr_t):
                ln.setValue(float(tt))

        if self.label_states:
            # 숫자끼리 가까우면 한 줄 내려 겹치지 않게 한다 (로그 분석과 같은 규칙)
            last_t, level = -np.inf, 0
            for i, (ln, tt) in enumerate(zip(self.lines, tr_t)):
                level = (level + 1) % 2 if tt - last_t < min_dt else 0
                last_t = tt
                if self.levels[i] != level:
                    ln.label.setPosition(0.95 - 0.08 * level)
                    self.levels[i] = level

    def clear(self) -> None:
        for items in self.regions.values():
            for r in items:
                r.hide()
        self.set_states(np.array([]), np.array([]), 1.0)


class Figure:
    """한 장.  축 / 선 / 배경은 처음에 한 번 만들고, 매 프레임 데이터만 바꾼다.

    grid[r][c] = (y 축 이름, [선 키 ...] (뒤가 위에 그려진다), 열 제목 또는 None)
    선 키는 tdcn_log_analyze.LINE 의 키 (current / target / ap / claw).
    """

    def __init__(self, app, k: int, title: str, lines: list[str], grid, size):
        self.app = app
        self.title = title
        self.subs: tuple = ()
        self.follow = True          # 최신 구간을 따라가며 범위를 맞춘다
        self.win = Window(self.toggle_follow)
        self.win.setWindowTitle(f"TDCN 실시간 — Figure {k}")
        self.win.resize(*size)

        ncols = len(grid[0])
        self.head = self.win.addLabel(row=0, col=0, colspan=ncols)
        self.win.addLabel(legend_html(lines), row=1, col=0, colspan=ncols, size="9pt")

        self.plots: list[list] = []
        self.curves: dict = {}
        self.decors: list[Decor] = []
        self.tick_lim: dict = {}
        first = None
        for r, row in enumerate(grid):
            prow = []
            for c, (ylabel, keys, col_title) in enumerate(row):
                p = self.win.addPlot(row=r + 2, col=c,
                                     axisItems={"left": LeftAxis(orientation="left")})
                p.showGrid(x=True, y=True, alpha=0.3)
                p.hideButtons()
                p.vb.sigRangeChangedManually.connect(self.on_manual)
                p.setLabel("left", ylabel)
                p.getAxis("left").setWidth(AXIS_W)
                if col_title:
                    p.setTitle(col_title, size="11pt")
                if r < len(grid) - 1:
                    p.getAxis("bottom").setStyle(showValues=False)
                else:
                    p.setLabel("bottom", A.XLABEL)
                if first is None:
                    first = p
                else:
                    p.setXLink(first)
                for key in keys:
                    cv = p.plot(pen=line_pen(key))
                    cv.setDownsampling(auto=True, method="peak")
                    cv.setClipToView(True)
                    self.curves[(r, c, key)] = cv
                self.decors.append(Decor(p, label_states=(r == 0)))
                prow.append(p)
            self.plots.append(prow)
        self.win.show()

    # --- 머리글 -------------------------------------------------------------

    def set_head(self, *subs: str) -> None:
        self.subs = subs
        self.render_head()

    def render_head(self) -> None:
        body = "<br>".join(s.replace("\n", "<br>") for s in self.subs)
        hold = ("" if self.follow else
                "<br><span style='color:#d62728'>[따라가기 멈춤 — Space 로 최신 구간 복귀]</span>")
        self.head.setText(f"<b>{self.title}</b><br>"
                          f"<span style='font-size:9pt'>{body}</span>{hold}",
                          size="11pt")

    # --- 따라가기 -----------------------------------------------------------

    def on_manual(self, *_) -> None:
        """휠 / 드래그로 범위를 손대면 이 창은 따라가기를 멈춘다."""
        if self.follow:
            self.follow = False
            self.release_ticks()
            self.render_head()

    def toggle_follow(self) -> None:
        self.follow = not self.follow
        if not self.follow:
            self.release_ticks()
        self.render_head()
        if self.follow:
            self.app.redraw_now()

    # --- 축 / 배경 ----------------------------------------------------------
    # 범위 / 고정 눈금은 따라가는 동안에만 맞춘다.  멈추면 사용자가 정한 범위를 둔다.

    def decorate(self, ctx: dict) -> None:
        first = self.plots[0][0]
        if self.follow:
            first.setXRange(ctx["t0"], ctx["t1"], padding=0)
        x0, x1 = first.viewRange()[0]
        min_dt = LABEL_GAP_PX / max(first.vb.width(), 1.0) * (x1 - x0)
        for d in self.decors:
            d.set_spans("out", ctx["out_spans"], BRUSH_OUT)
            d.set_spans("act", ctx["act_spans"], BRUSH_ACT)
            d.set_states(ctx["tr_t"], ctx["tr_label"], min_dt, ctx["t0"])

    def set_yrange(self, r: int, c: int, lo: float, hi: float) -> None:
        if self.follow:
            self.plots[r][c].setYRange(lo, hi, padding=0)

    def set_ticks(self, r: int, c: int, lim: tuple[float, float]) -> None:
        """y 축 눈금을 lim 사이 5 개로 고정한다 (헤딩, 제어값)."""
        if self.follow and self.tick_lim.get((r, c)) != lim:
            vals = np.linspace(lim[0], lim[1], 5)
            self.plots[r][c].getAxis("left").setTicks([[(v, f"{v:g}") for v in vals], []])
            self.tick_lim[(r, c)] = lim

    def release_ticks(self) -> None:
        """고정 눈금을 풀어 확대한 범위에 맞는 눈금이 나오게 한다."""
        for r, c in self.tick_lim:
            self.plots[r][c].getAxis("left").setTicks(None)
        self.tick_lim.clear()

    def heading_axis(self, r: int, c: int, lim: tuple[float, float]) -> None:
        self.set_yrange(r, c, *lim)
        self.set_ticks(r, c, lim)

    def clear(self, msg: str) -> None:
        self.set_head(msg)
        for cv in self.curves.values():
            cv.setData([], [])
        for d in self.decors:
            d.clear()


# ---------------------------------------------------------------------------
# Figure 1 — 현재 상태 (위치 / 속도 / 자세)
# ---------------------------------------------------------------------------

COLS1 = (
    ("위치", (("PN", "North (m)"), ("PE", "East (m)"), ("PD", "Down (m)"))),
    ("속도", (("VN", "VN (m/s)"), ("VE", "VE (m/s)"), ("VD", "VD (m/s)"))),
    ("자세", (("Roll", "Roll (deg)"), ("Pitch", "Pitch (deg)"), ("Yaw", "Yaw (deg)"))),
)


def make_fig1(app) -> Figure:
    grid = [[(COLS1[c][1][r][1], ["current"], COLS1[c][0] if r == 0 else None)
             for c in range(3)] for r in range(3)]
    return Figure(app, 1, "[Figure 1] 현재 상태 — 위치 · 속도 · 자세 (TDST)",
                  ["current"], grid, SIZE_3x3)


def update1(fig: Figure, data: dict, ctx: dict, sub: str) -> None:
    s = data["TDST"]
    t = s["t"]
    pos_lims, pos_span = A.equal_span_limits([s["PN"], s["PE"], s["PD"]])
    vel_lims, vel_span = A.equal_span_limits([s["VN"], s["VE"], s["VD"]])
    att_lims, att_span = A.equal_span_limits([s["Roll"], s["Pitch"]])
    lims = (pos_lims, vel_lims, att_lims + [None])

    fig.set_head(sub, f"N/E/D 폭 {pos_span:.1f} m, 속도 폭 {vel_span:.1f} m/s, "
                      f"Roll/Pitch 폭 {att_span:.1f} deg 로 통일, "
                      f"Yaw 는 {A.hdg_text(ctx['hdg_lim'])} 고정")
    fig.decorate(ctx)
    for c, (_, rows) in enumerate(COLS1):
        for r, (key, _) in enumerate(rows):
            y = s[key]
            if key == "Yaw":
                y = A.wrap_heading(y, ctx["hdg_lim"])
                fig.heading_axis(r, c, ctx["hdg_lim"])
            else:
                fig.set_yrange(r, c, *lims[c][r])
            fig.curves[(r, c, "current")].setData(t, y, connect="finite")


# ---------------------------------------------------------------------------
# Figure 2 — 목표 (위치 / 헤딩) + 현재
# ---------------------------------------------------------------------------

ROWS2 = (
    ("North (m)",     "PN",  "PN"),
    ("East (m)",      "PE",  "PE"),
    ("Down (m)",      "PD",  "PD"),
    ("Heading (deg)", "Hdg", "Yaw"),
)


def make_fig2(app) -> Figure:
    grid = [[(ylabel, ["current", "target"], None)] for ylabel, _, _ in ROWS2]
    return Figure(app, 2, "[Figure 2] 목표 — 위치 · 헤딩 (TDTG), 현재값 (TDST) 겹침",
                  ["target", "current"], grid, SIZE_4x1)


def update2(fig: Figure, data: dict, ctx: dict, sub: str) -> None:
    s, g = data["TDST"], data["TDTG"]

    # 목표 위치는 Val = 1 인 곳만, 목표 헤딩은 TDCN 안에서만 그린다
    valid = g["Val"] > 0.5
    in_tdcn = A.at_times(s["t"], s["Mode"], g["t"]) == A.TDCN_MODE
    tgt = {k: np.where(valid, g[k], np.nan) for k in ("PN", "PE", "PD")}
    tgt["Hdg"] = np.where(in_tdcn, g["Hdg"], np.nan)

    lims, span = A.equal_span_limits(
        [np.concatenate([tgt[k], s[k]]) for k in ("PN", "PE", "PD")])
    lim_h = ctx["hdg_lim"]

    fig.set_head(sub, f"목표 위치는 Val = 1 (목표 있음) 인 구간만, "
                      f"N/E/D 는 같은 폭 {span:.1f} m, Heading 은 {A.hdg_text(lim_h)} 고정")
    fig.decorate(ctx)
    for r, (_, tk, ck) in enumerate(ROWS2):
        yc, yt = s[ck], tgt[tk]
        if tk == "Hdg":
            yc, yt = A.wrap_heading(yc, lim_h), A.wrap_heading(yt, lim_h)
            fig.heading_axis(r, 0, lim_h)
        else:
            fig.set_yrange(r, 0, *lims[r])
        fig.curves[(r, 0, "current")].setData(s["t"], yc, connect="finite")
        fig.curves[(r, 0, "target")].setData(g["t"], yt, connect="finite")


# ---------------------------------------------------------------------------
# Figure 3 — 믹서 직전 제어값 (아두파일럿 vs CLAW)
# ---------------------------------------------------------------------------

ROWS3 = (
    ("Roll",     "AR", "CR", (-1.0, 1.0)),
    ("Pitch",    "AP", "CP", (-1.0, 1.0)),
    ("Yaw",      "AY", "CY", (-1.0, 1.0)),
    ("Throttle", "AT", "CT", (0.0, 1.0)),
)


def make_fig3(app) -> Figure:
    grid = [[(name, ["claw", "ap"], None)] for name, *_ in ROWS3]
    fig = Figure(app, 3, "[Figure 3] 믹서 직전 제어값 — 아두파일럿 vs CLAW (TDMX)",
                 ["ap", "claw"], grid, SIZE_4x1)
    for r in range(len(ROWS3)):
        fig.plots[r][0].addItem(pg.InfiniteLine(0.0, angle=0, pen=ZERO_PEN, movable=False),
                                ignoreBounds=True)
    return fig


def range_html(ya, yc) -> str:
    """실제 사용 범위를 숫자로 적는다 (축은 정의역 고정이라 좁아 보인다)."""
    def one(y, color: str, name: str) -> str:
        y = y[np.isfinite(y)]
        txt = f"{name}  {y.min():+.3f} ~ {y.max():+.3f}" if y.size else f"{name}  -"
        return f"<span style='color:{color}'>{txt}</span>"
    return (one(ya, A.C_AP, A.LINE["ap"]["label"]) + "&nbsp;&nbsp;&nbsp;&nbsp;"
            + one(yc, A.C_CLAW, A.LINE["claw"]["label"]))


def update3(fig: Figure, data: dict, ctx: dict, sub: str) -> None:
    x = data["TDMX"]
    t = x["t"]
    fig.set_head(sub, "y 축은 정의역으로 고정: Roll/Pitch/Yaw = -1~1, "
                      "Throttle = 0~1.  CLAW 는 state 6 밖에서 0")
    fig.decorate(ctx)
    for r, (_, ak, ck, ylim) in enumerate(ROWS3):
        pad = 0.05 * (ylim[1] - ylim[0]) / 2.0     # 정의역 끝 (±1 포화) 이 테두리에 가리지 않게
        fig.set_yrange(r, 0, ylim[0] - pad, ylim[1] + pad)
        fig.set_ticks(r, 0, ylim)
        fig.curves[(r, 0, "claw")].setData(t, x[ck], connect="finite")
        fig.curves[(r, 0, "ap")].setData(t, x[ak], connect="finite")
        fig.plots[r][0].setTitle(range_html(x[ak], x[ck]), size="8pt", justify="right")


# ---------------------------------------------------------------------------
# 앱
# ---------------------------------------------------------------------------

class App:
    def __init__(self, args):
        self.args = args
        self.rx = Receiver(args.connect)
        self.rx.start()
        self.store = Store()
        self.figs = [make_fig1(self), make_fig2(self), make_fig3(self)]
        self.updates = (update1, update2, update3)
        self.last_draw = 0.0

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.tick)
        self.timer.start(max(1, int(1000 / args.fps)))

    def redraw_now(self) -> None:
        """다음 tick 에 새 데이터가 없어도 다시 그린다 (따라가기 재개)."""
        self.last_draw = 0.0

    def tick(self) -> None:
        changed = self.store.pull(self.rx)
        now = time.monotonic()
        # 새 데이터가 없어도 1 초마다는 다시 그린다 (수신 끊김 표시)
        if not changed and now - self.last_draw < 1.0:
            return
        self.last_draw = now

        rows = self.store.rows
        data, ctx, msg = prepare(rows, self.args.window, self.args.connect)
        if data is None:
            for f in self.figs:
                f.clear(msg)
            return
        sub = subtitle(rows, self.store.last_rx, self.args.connect, ctx)
        for f, update in zip(self.figs, self.updates):
            if f.win.isVisible():               # 닫은 창은 건너뛴다
                update(f, data, ctx, sub)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="TDCN 실시간 그림 (v4, pyqtgraph) — tdcn_log_analyze.py 의 "
                    "Figure 1~3 을 비행 중에")
    ap.add_argument("--connect", "-c", default="udp:127.0.0.1:14560",
                    help="MAVLink 접속 문자열 (기본: udp:127.0.0.1:14560). "
                         "MAVProxy 를 --out=udp:127.0.0.1:14560 으로 띄워 둔다.")
    ap.add_argument("--window", type=float, metavar="S",
                    help="최근 S 초만 그린다 (생략 시 TDCN 첫 진입부터 전부)")
    ap.add_argument("--fps", type=float, default=FPS,
                    help=f"화면 갱신 (초당, 기본 {FPS}).  새 데이터가 없으면 다시 그리지 않는다")
    args = ap.parse_args(argv)

    signal.signal(signal.SIGINT, signal.SIG_DFL)    # 터미널에서 Ctrl+C 로 끝낼 수 있게
    pg.setConfigOptions(background="w", foreground="k", antialias=True)
    pg.mkQApp("TDCN 실시간")

    print(f"[live] 수신 대기: {args.connect}")
    app = App(args)  # noqa: F841  (타이머가 살아 있도록 참조를 잡아 둔다)
    pg.exec()        # 세 창을 모두 닫으면 끝난다
    return 0


if __name__ == "__main__":
    sys.exit(main())
