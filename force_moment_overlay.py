# -*- coding: utf-8 -*-
"""
DynPick 力覚センサ -> RoboDK 3Dビュー  力/モーメント矢印リアルタイム表示

構成 : 既存の DynPick 読み取りスクリプトに本コードを統合（robolink を追加）
前提 : RoboDK を「Run on Robot（ドライバ接続）」状態で使用
動作 : ロボットが動き出すと robot.Busy() を検知して自動で計測・表示を開始し、
       停止すると矢印を消す（= 自動スタート/ストップ）

使い方（3ステップ）:
  1) この内容を既存スクリプトに取り込む（または import して main() を呼ぶ）
  2) 下の read_wrench() を、既存の DynPick 読み取り呼び出しに差し替える
     （USE_DEMO_SIGNAL=True なら、センサ未接続でもダミー波形で描画確認できる）
  3) RoboDK を Run on Robot 接続した状態で実行し、RoboDK 側で Run すると矢印が出る
"""

import os
import sys
import time
import math
import csv
import struct
from collections import deque
from datetime import datetime

# RoboDK のボタンから起動しても隣の dynpick_sensor.py を読めるよう、
# このファイルのあるフォルダを常に import パスへ追加する。
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# robolink は RoboDK 連携時のみ必要。--no-robodk（記録のみ）では読み込まないので、
# robodk 未インストールのPCでもロガーは動く。実際の import は _import_robolink() で遅延実行。
Robolink = None
ITEM_TYPE_ROBOT = None
PROJECTION_NONE = None


def _import_robolink():
    """RoboDK 連携に必要な robolink シンボルを遅延 import してモジュール全体で使えるようにする。"""
    global Robolink, ITEM_TYPE_ROBOT, PROJECTION_NONE
    if Robolink is not None:
        return
    try:
        # RoboDK 同梱 Python（scripts フォルダが path に入っている場合）
        from robolink import Robolink as _R, ITEM_TYPE_ROBOT as _IT, PROJECTION_NONE as _PN
    except ImportError:
        # 外部 Python（pip install robodk）の場合
        from robodk.robolink import Robolink as _R, ITEM_TYPE_ROBOT as _IT, PROJECTION_NONE as _PN
    Robolink, ITEM_TYPE_ROBOT, PROJECTION_NONE = _R, _IT, _PN


from dynpick_sensor import DynPickSensor

# =================== 調整パラメータ ===================
ROBOT_NAME = ''          # '' で最初のロボット。名前指定も可 例: 'Fanuc LR Mate 200iD/7L'
USE_DEMO_SIGNAL = False  # True: ダミー正弦波でRoboDK描画テスト / False: read_wrench()の実センサ値
USE_ROBODK      = True    # True: RoboDK連携（矢印表示）/ False: センサ読み取り＋CSV記録のみ（--no-robodk）

# --- DynPick 実センサ接続（USE_DEMO_SIGNAL=False のとき使用）---
DYNPICK_PORT = 'COM3'    # 接続ポート。Windows: 'COM3' 等 / Linux: '/dev/ttyUSB0'
DYNPICK_BAUD = 921600    # DynPick 標準ボーレート（機種により 230400 等）
TARE_ON_START = True     # 開始時に無負荷状態で零点を実測（測定前はツールに何も触れないこと）
TARE_SAMPLES  = 100      # 零点実測に使うサンプル数
# 別センサを使う場合は dynpick_sensor.py の DEFAULT_SENS_* / DEFAULT_ZERO を書き換える

FORCE_SCALE   = 7.0      # [mm/N]      力1Nあたりの矢印長さ（実測: 最大力≈30Nで矢印≈210mm）
MOMENT_SCALE  = 200.0    # [mm/(N*m)]  モーメント矢印の長さ倍率（実測: 最大≈1N*mで矢印≈200mm）
FORCE_COLOR   = [1.0, 0.15, 0.15]   # 力 = 赤
MOMENT_COLOR  = [0.15, 0.5, 1.0]    # モーメント = 青
FORCE_DEADBAND  = 0.3    # [N]    これ未満は非表示（ノイズ抑制）
MOMENT_DEADBAND = 0.02   # [N*m]  これ未満は非表示
MAX_ARROW_LEN   = 250.0  # [mm]   矢印長さの上限（振り切れ防止）
UPDATE_RATE     = 10     # [Hz]   RoboDK連携時の更新レート（高すぎるとRoboDK APIが不安定に）
HEADLESS_RATE   = 50     # [Hz]   --no-robodk 記録のみモードのサンプリング（RoboDK制約が無いので高めに）
EMA_ALPHA       = 0.4    # 0<..<=1  表示のローパス（1で無効）
ACTIVE_ONLY_WHEN_MOVING = True   # True: ロボット動作中のみ表示（自動スタート）

# 動作検知の方法（ACTIVE_ONLY_WHEN_MOVING=True のとき有効）
#   'busy'   : robot.Busy() のみ（RoboDK が駆動しているときに有効。ドライバ依存で鈍いことあり）
#   'joints' : 関節角の変化のみ（実機に接続して関節を監視。ドライバ非依存で確実）
#   'both'   : 上記の OR で判定（推奨。どちらかが検知すれば動作中とみなし取りこぼさない）
MOTION_DETECT  = 'both'
JOINT_MOVE_DEG = 0.05    # [deg] 1ループでの関節角変化がこれ以上なら「動作中」とみなす
MOTION_HOLD_S  = 0.5     # [s]  最後に動きを検知してからこの時間は表示/記録を継続（ちらつき防止）

# センサ生値 -> ツール座標 の軸割当  (生値のindex, 符号)。取付向きに合わせて調整。
# 校正(センサ軸分離) + RoboDK上の向き確認(+X押し->+Y, +Z押し->-X)から、
# アクティブTCPと表示triadの90°ズレ(Q: X->Y,Z->-X)を打ち消して補正した割当。
AXIS_MAP_FORCE  = [(1, -1), (2, -1), (0, +1)]
AXIS_MAP_MOMENT = [(1, -1), (2, -1), (0, +1)]   # 取付回転は力と同じ

# 矢印の基点オフセット [mm]（TCP座標系）。TCPとセンサ計測原点/刃先がずれている場合に
# 根元位置を調整する。例: 刃先方向(+Z)に50mm出すなら [0,0,50]。既定は TCP そのもの。
BASE_OFFSET_TOOL = [0.0, 0.0, 0.0]

# --- CSV ログ記録（動作中の力/モーメントを記録） ---
LOG_CSV   = False   # True: 力/モーメントをCSVに記録（--log でも有効化）
LOG_PATH  = ''      # 保存先パス。'' なら force_log_日時.csv をスクリプトと同じフォルダに自動生成
LOG_EVERY = 1       # 何サンプルごとに1行記録するか（1=毎サンプル。長時間運用で間引くなら 2,5.. ）

# --- 過負荷監視（工具・ワーク保護） -------------------------------------
# |F|[N] がしきい値以上に一定サンプル連続で達したら「発報」する。発報時に端末アラーム＋
# アラームログ(sidecar)＋（RELAY_PORT 指定時）USBリレーの接点をON。
# ⚠ これはソフト監視で「安全定格」ではない。人身安全は必ずハード非常停止（安全回路）で担保し、
#    本機能は押し過ぎ・突入から刃/砥石/センサを守るプロセス保護として使うこと（--force-limit で有効化）。
FORCE_LIMIT      = None  # [N] しきい値。None で無効（--force-limit N で設定）
FORCE_LIMIT_HOLD = 3     # 連続何サンプル超過で発報するか（誤検知防止のデバウンス。3=約60ms@50Hz）
FORCE_LIMIT_MASK = 0.0   # [s] 記録開始からこの秒数は監視しない（突入過渡での誤検知を避ける）
FORCE_LIMIT_LATCH = False  # True: 一度発報したら解除（再起動）まで保持＝リレーON維持（E-stop/HOLD向け）
RELAY_PORT       = ''    # USBリレーのシリアルポート 例 'COM5'。'' で出力なし（アラームのみ）
RELAY_BAUD       = 9600  # USBリレーのボーレート（LCUS系は 9600）
# LCUS-1 等の一般的な5V USBリレー（CH340系）のON/OFFコマンド。別型番なら書き換える。
RELAY_ON_BYTES   = bytes((0xA0, 0x01, 0x01, 0xA2))
RELAY_OFF_BYTES  = bytes((0xA0, 0x01, 0x00, 0xA1))
# =====================================================


# ---------- 小さなベクトル演算（依存を増やさないため自前） ----------
def _norm(v):
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])

def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]]

def _remap(v, amap):
    return [sign * v[idx] for (idx, sign) in amap]

def _ema(prev, cur, a):
    return [prev[i] + a * (cur[i] - prev[i]) for i in range(3)]

def _perp(d):
    """単位ベクトル d に直交する単位ベクトルを1つ返す"""
    a = [1.0, 0.0, 0.0]
    if abs(d[0]) > 0.9:
        a = [0.0, 1.0, 0.0]
    u = _cross(d, a)
    n = _norm(u)
    return [u[0] / n, u[1] / n, u[2] / n]

def _rot_vec(pose, v):
    """RoboDK の Mat(4x4) の回転部分だけでベクトル v を回す（並進は無視）"""
    return [
        pose[0, 0] * v[0] + pose[0, 1] * v[1] + pose[0, 2] * v[2],
        pose[1, 0] * v[0] + pose[1, 1] * v[1] + pose[1, 2] * v[2],
        pose[2, 0] * v[0] + pose[2, 1] * v[1] + pose[2, 2] * v[2],
    ]


# ---------- 矢印（1本 = 1つの折れ線カーブで描画） ----------
class WrenchArrow:
    def __init__(self, RDK, color, name):
        self.RDK = RDK
        self.color = color
        self.name = name
        self.item = None

    def _points(self, p0, vec, length):
        """始点 p0[mm]、方向 vec、全長 length[mm] の矢印を折れ線点列で返す"""
        n = _norm(vec)
        if n < 1e-9 or length < 1e-6:
            return None
        d = [vec[0] / n, vec[1] / n, vec[2] / n]
        p1 = [p0[i] + d[i] * length for i in range(3)]
        # 矢じり：先端 p1 から後方へ head だけ戻り、直交2方向に開く
        head = max(4.0, length * 0.18)
        u = _perp(d)
        w = _cross(d, u)
        back = [p1[i] - d[i] * head for i in range(3)]
        h = head * 0.5
        pts = [p0, p1]
        for s1, s2 in [(+1, 0), (-1, 0), (0, +1), (0, -1)]:
            tip = [back[i] + u[i] * h * s1 + w[i] * h * s2 for i in range(3)]
            pts.append(tip)
            pts.append(p1)  # 先端に戻ってから次の矢じり線へ
        return pts

    def update(self, p0, vec, length):
        pts = self._points(p0, vec, length)
        new_item = None
        if pts is not None:
            # reference=None -> ステーション絶対座標。投影しない（空中に描く）。
            new_item = self.RDK.AddCurve(pts, None, False, PROJECTION_NONE)
            if new_item.Valid():
                new_item.setColor(self.color)
                new_item.setName(self.name)
        # 旧カーブは後から削除（先に新規を出すことでちらつきを軽減）
        if self.item is not None and self.item.Valid():
            self.item.Delete()
        self.item = new_item

    def clear(self):
        if self.item is not None and self.item.Valid():
            self.item.Delete()
        self.item = None


# ---------- CSV ログ記録 ----------
def _make_log_path():
    """既定のログ保存パス（スクリプトと同じフォルダに force_log_日時.csv）を返す。"""
    folder = os.path.dirname(os.path.abspath(__file__))
    fname = 'force_log_' + datetime.now().strftime('%Y%m%d_%H%M%S') + '.csv'
    return os.path.join(folder, fname)


class ForceLogger:
    """力/モーメントの計測値を CSV に追記記録する。

    記録するのは read_wrench() が返す生の物理量（N, N*m）。表示用の EMA/軸割当
    ではなく、後解析しやすい素の値を残す。TCP位置も併記して力と位置を対応付ける。
    """

    HEADER = [
        'time_iso', 't_s',
        'fx_N', 'fy_N', 'fz_N',
        'mx_Nm', 'my_Nm', 'mz_Nm',
        'Fmag_N', 'Mmag_Nm',
        'tcp_x_mm', 'tcp_y_mm', 'tcp_z_mm',
        'moving',
    ]

    def __init__(self, path):
        self.path = path
        self._f = open(path, 'w', newline='', encoding='utf-8')
        self._w = csv.writer(self._f)
        self._w.writerow(self.HEADER)
        self._f.flush()
        self.rows = 0

    def write(self, t_s, wrench, tcp_pos, moving):
        fx, fy, fz, mx, my, mz = wrench
        fmag = math.sqrt(fx * fx + fy * fy + fz * fz)
        mmag = math.sqrt(mx * mx + my * my + mz * mz)
        # tcp_pos=None（RoboDK未使用時）は位置列を空欄にする
        if tcp_pos is None:
            px = py = pz = ''
        else:
            px = '%.2f' % tcp_pos[0]
            py = '%.2f' % tcp_pos[1]
            pz = '%.2f' % tcp_pos[2]
        self._w.writerow([
            datetime.now().isoformat(timespec='milliseconds'),
            '%.3f' % t_s,
            '%.4f' % fx, '%.4f' % fy, '%.4f' % fz,
            '%.5f' % mx, '%.5f' % my, '%.5f' % mz,
            '%.4f' % fmag, '%.5f' % mmag,
            px, py, pz,
            1 if moving else 0,
        ])
        self._f.flush()   # 途中で強制終了(Ctrl+C)しても記録が残るよう毎回フラッシュ
        self.rows += 1

    def close(self):
        try:
            self._f.close()
        except Exception:
            pass


class SerialRelay:
    """USBシリアルリレー（LCUS系 CH340 等）の接点を ON/OFF する薄いラッパ。

    既定の RELAY_ON_BYTES / RELAY_OFF_BYTES は一般的な1ch 5Vリレーのコマンド。
    別型番なら冒頭のバイト列を書き換える。open に失敗しても記録は続行する（監視は
    アラームのみになる）。pyserial を使う（記録用に既に依存済み）。
    """

    def __init__(self, port, baud=RELAY_BAUD,
                 on_bytes=RELAY_ON_BYTES, off_bytes=RELAY_OFF_BYTES):
        import serial   # pyserial（DynPick で既に使用）
        self._ser = serial.Serial(port, baud, timeout=0.2)
        self._on = on_bytes
        self._off = off_bytes
        self.off()   # 起動時は必ず接点OFF（安全側）

    def on(self):
        try:
            self._ser.write(self._on)
            self._ser.flush()
        except Exception:
            pass

    def off(self):
        try:
            self._ser.write(self._off)
            self._ser.flush()
        except Exception:
            pass

    def close(self):
        try:
            self.off()
            self._ser.close()
        except Exception:
            pass


class ForceLimitMonitor:
    """|F| の過負荷監視（工具・ワーク保護）。1サンプルずつ update() で評価する。

    - しきい値以上が hold サンプル連続したら「発報」（立ち上がりエッジで1回）。
    - 発報で: 端末アラーム(ビープ付)＋アラームログ(sidecar CSV)＋（あれば）リレーON。
    - latch=False: |F| がしきい値未満に戻ると復帰し、リレーはOFF（次の超過で再発報）。
      latch=True : 一度発報したら保持（リレーON維持）。E-stop/HOLD 用の安全側。
    ⚠ ソフト監視なので安全定格ではない。人身安全はハード非常停止で担保すること。
    """

    def __init__(self, limit_N, hold=3, mask_s=0.0, latch=False,
                 alarm_path=None, relay=None):
        self.limit = limit_N
        self.hold = max(1, int(hold))
        self.mask_s = mask_s
        self.latch = latch
        self.alarm_path = alarm_path
        self.relay = relay
        self.count = 0        # 連続超過サンプル数
        self.active = False   # いま発報状態か（リレーON中か）
        self.events = 0       # 発報回数
        self._af = None       # アラームログのファイルハンドル（遅延生成）

    def enabled(self):
        return self.limit is not None

    def update(self, ts, fmag, iso=None):
        """1サンプル評価。新規に発報した瞬間だけ True を返す。"""
        if self.limit is None or ts < self.mask_s:
            return False
        over = fmag >= self.limit
        self.count = self.count + 1 if over else 0

        if over and self.count >= self.hold and not self.active:
            # 立ち上がり発報
            self.active = True
            self.events += 1
            if self.relay is not None:
                self.relay.on()
            self._log(ts, fmag, iso, 'TRIP')
            self._alarm(ts, fmag)
            return True

        if not over and self.active and not self.latch:
            # 平常復帰（latch時は保持）
            self.active = False
            if self.relay is not None:
                self.relay.off()
            self._log(ts, fmag, iso, 'clear')
        return False

    def _alarm(self, ts, fmag):
        # \n で改行してから出す（\r の1行更新に上書きされないように）。\a はビープ。
        state = ' [LATCH: 解除まで保持]' if self.latch else ''
        relay = ' → リレーON' if self.relay is not None else ''
        print('\a\n*** 過負荷検知 |F|=%.1fN ≥ しきい値%.1fN  (t=%.1fs)%s%s ***'
              % (fmag, self.limit, ts, relay, state))

    def _log(self, ts, fmag, iso, event):
        if not self.alarm_path:
            return
        try:
            if self._af is None:
                self._af = open(self.alarm_path, 'w', newline='', encoding='utf-8')
                w = csv.writer(self._af)
                w.writerow(['time_iso', 't_s', 'Fmag_N', 'limit_N', 'event'])
                self._aw = w
            self._aw.writerow([iso or datetime.now().isoformat(timespec='milliseconds'),
                               '%.3f' % ts, '%.4f' % fmag, '%.4f' % self.limit, event])
            self._af.flush()
        except Exception:
            pass

    def close(self):
        if self.relay is not None:
            self.relay.close()
        try:
            if self._af is not None:
                self._af.close()
        except Exception:
            pass


# 保存後に、CSV のあるフォルダをエクスプローラーで開く（ファイルを選択した状態）。
OPEN_FOLDER_ON_SAVE = True
# 保存後に plot_force_log.py で自動的にグラフを開く（--plot）。有効時はエクスプローラー表示より優先。
PLOT_ON_SAVE = False
# 記録中に力/モーメントをリアルタイム表示する（--live）。
LIVE_PLOT = False

def _reveal_in_explorer(path):
    """保存した CSV をエクスプローラーで選択表示する。失敗しても記録には影響しない。

    Windows: explorer /select でフォルダを開きファイルを選択。
    mac/Linux: フォルダを開くだけ（フォールバック）。
    """
    if not OPEN_FOLDER_ON_SAVE:
        return
    try:
        if not path or not os.path.isfile(path):
            return
        import subprocess
        full = os.path.normpath(os.path.abspath(path))
        if sys.platform.startswith('win'):
            subprocess.Popen(['explorer', '/select,' + full])
        elif sys.platform == 'darwin':
            subprocess.Popen(['open', '-R', full])
        else:
            subprocess.Popen(['xdg-open', os.path.dirname(full)])
    except Exception:
        pass


# ---------- センサ読み取り ----------
_SENSOR = None   # DynPickSensor インスタンス（実センサ使用時に main() で生成）

def read_wrench(t=0.0):
    """(fx, fy, fz, mx, my, mz) を返す。単位は [N, N*m]。

    USE_DEMO_SIGNAL=True  : ダミー正弦波（センサ未接続でも描画テスト可）。
    USE_DEMO_SIGNAL=False : DynPickSensor から実センサ値を取得（dynpick_sensor.py）。
    """
    if USE_DEMO_SIGNAL:
        return (8 * math.sin(t * 1.5), 4 * math.cos(t * 1.1), 6 * math.sin(t * 0.7),
                0.10 * math.sin(t * 1.3), 0.05 * math.cos(t * 0.9), 0.08 * math.sin(t))
    if _SENSOR is None:
        raise RuntimeError('DynPick センサが初期化されていません（main() で open 済みのはず）')
    return _SENSOR.read_wrench()


def _connect_robot():
    """RoboDK に接続してロボット Item を返す。"""
    RDK = Robolink()
    robot = RDK.Item(ROBOT_NAME, ITEM_TYPE_ROBOT)
    if not robot.Valid():
        raise Exception('ロボットが見つかりません。ROBOT_NAME を確認してください。')
    return RDK, robot


def _joints_list(robot):
    """ロボットの現在の関節角を list[float]（度）で返す。

    Run on Robot 接続時は実機の関節角、未接続時はシミュレーションの関節角。
    robodk の版差（Mat.list / tolist / 添字）を吸収する。
    """
    j = robot.Joints()
    try:
        return [float(v) for v in j.list()]          # 版によっては .list() を持つ
    except Exception:
        pass
    try:
        rows = j.tolist()                             # [[j1],[j2],...] 形式
        return [float(r[0] if isinstance(r, (list, tuple)) else r) for r in rows]
    except Exception:
        n = getattr(j, 'rows', 6)
        return [float(j[i, 0]) for i in range(n)]


# ---------- リアルタイム表示 ----------
LIVE_WINDOW_S = 30.0    # ライブグラフに表示する直近秒数
LIVE_FPS      = 12.0    # ライブグラフの再描画頻度[Hz]（サンプリングとは独立）

class LivePlot:
    """記録しながら 力/モーメントを直近 LIVE_WINDOW_S 秒だけ流し表示する簡易ライブグラフ。

    matplotlib の対話モード（ion）で、サンプリングループを止めずに一定間隔で再描画する。
    matplotlib が無ければ生成に失敗するので、呼び出し側で握りつぶして端末表示のみに退避する。
    """

    COL = {'x': '#D55E00', 'y': '#009E73', 'z': '#0072B2', 'mag': '#222222'}

    def __init__(self, window_s=LIVE_WINDOW_S):
        import matplotlib.pyplot as plt
        self.plt = plt
        self.window = window_s
        self._last_draw = 0.0
        self._interval = 1.0 / LIVE_FPS
        self.t = deque()
        self.buf = {k: deque() for k in
                    ('fx', 'fy', 'fz', 'fm', 'mx', 'my', 'mz', 'mm')}
        plt.ion()
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, sharex=True,
                                                       figsize=(10, 6))
        try:
            self.fig.canvas.manager.set_window_title('DynPick Live Force/Moment')
        except Exception:
            pass   # バックエンドにより manager が無い/仕様違いでも致命的でない
        self.lines = {}
        for key, ax, comps, ylab in (
                ('F', self.ax1, (('fx', 'Fx', 'x'), ('fy', 'Fy', 'y'),
                                 ('fz', 'Fz', 'z'), ('fm', '|F|', 'mag')), 'Force [N]'),
                ('M', self.ax2, (('mx', 'Mx', 'x'), ('my', 'My', 'y'),
                                 ('mz', 'Mz', 'z'), ('mm', '|M|', 'mag')), 'Moment [N*m]')):
            for buf_key, label, ckey in comps:
                lw = 2.0 if ckey == 'mag' else 1.2
                (self.lines[buf_key],) = ax.plot([], [], color=self.COL[ckey],
                                                 lw=lw, label=label)
            ax.set_ylabel(ylab)
            ax.grid(True, color='#CCCCCC', lw=0.6)
            ax.legend(loc='upper left', ncol=4, framealpha=0.9)
        self.ax2.set_xlabel('Time [s]')
        self.ax1.set_title('Live (last %.0f s) - click STOP (or close window / press q) to finish'
                           % window_s)
        # 下側にSTOPボタンの場所を空ける
        self.fig.subplots_adjust(left=0.09, right=0.98, top=0.93, bottom=0.15, hspace=0.15)

        self.closed = False
        self.stop_requested = False

        # STOP ボタン（クリックで記録終了）。Ctrl+C が効かない環境の主要な終了手段。
        self.btn = None
        try:
            from matplotlib.widgets import Button
            axstop = self.fig.add_axes([0.83, 0.02, 0.14, 0.07])
            self.btn = Button(axstop, 'STOP', color='#E8A0A0', hovercolor='#E05050')
            self.btn.on_clicked(self._on_stop_clicked)
        except Exception:
            pass   # ボタンが作れなくても窓閉じ/qキー/Ctrl+C で終了可能

        self.fig.canvas.mpl_connect('close_event', self._on_close)
        self.fig.canvas.mpl_connect('key_press_event', self._on_key)
        # ウィンドウを確実に表示（block=False + pause でGUIイベントを回して描画）
        try:
            plt.show(block=False)
        except Exception:
            self.fig.show()
        plt.pause(0.05)

    def _on_close(self, _evt):
        self.closed = True
        self.stop_requested = True   # 窓を閉じたら記録も終了

    def _on_stop_clicked(self, _evt):
        self.stop_requested = True

    def _on_key(self, evt):
        if getattr(evt, 'key', None) in ('q', 'escape'):
            self.stop_requested = True

    def push(self, t, f, m):
        fx, fy, fz = f
        mx, my, mz = m
        self.t.append(t)
        self.buf['fx'].append(fx); self.buf['fy'].append(fy); self.buf['fz'].append(fz)
        self.buf['fm'].append(math.sqrt(fx * fx + fy * fy + fz * fz))
        self.buf['mx'].append(mx); self.buf['my'].append(my); self.buf['mz'].append(mz)
        self.buf['mm'].append(math.sqrt(mx * mx + my * my + mz * mz))
        # 窓外の古いサンプルを捨てる
        while self.t and (t - self.t[0]) > self.window:
            self.t.popleft()
            for b in self.buf.values():
                b.popleft()

    def maybe_draw(self, now):
        if self.closed or (now - self._last_draw) < self._interval:
            return
        self._last_draw = now
        ts = list(self.t)
        if len(ts) < 2:
            return
        for key, line in self.lines.items():
            line.set_data(ts, list(self.buf[key]))
        self.ax1.set_xlim(ts[0], ts[-1])
        for ax in (self.ax1, self.ax2):
            ax.relim(); ax.autoscale_view(scalex=False, scaley=True)
        try:
            # plt.pause は描画＋GUIイベント処理をまとめて行い、バックエンド差に強い
            self.plt.pause(0.001)
        except Exception:
            self.closed = True

    def close(self):
        try:
            self.plt.ioff()
        except Exception:
            pass


def _launch_plot(csv_path):
    """記録終了後、保存した CSV を plot_force_log.py で開く（別プロセス・非ブロッキング）。"""
    try:
        here = os.path.dirname(os.path.abspath(__file__))
        script = os.path.join(here, 'plot_force_log.py')
        if not os.path.isfile(script) or not csv_path or not os.path.isfile(csv_path):
            return
        import subprocess
        subprocess.Popen([sys.executable, script, os.path.abspath(csv_path)],
                         cwd=here)
    except Exception:
        pass


def main_headless():
    """RoboDK を使わず、DynPick センサの値を CSV に記録するだけのモード。

    ロボットはティーチペンダント等で動かし、力データだけを時刻付きで残したいとき用
    （Stream Motion 等のドライバが無く RoboDK から実機を駆動できない構成向け）。
    接続・矢印描画は行わず、常時記録する。位置列は空欄になる。
    """
    global _SENSOR

    if not USE_DEMO_SIGNAL:
        _SENSOR = DynPickSensor(port=DYNPICK_PORT, baudrate=DYNPICK_BAUD)
        _SENSOR.open()
        if TARE_ON_START:
            print('DynPick 零点測定中… ツールに触れないでください')
            _SENSOR.tare(TARE_SAMPLES)
            print('零点測定 完了')

    log_path = LOG_PATH or _make_log_path()
    logger = ForceLogger(log_path)
    print('CSV記録先:', log_path)
    print('記録開始（ロボットを動かしてください。終了は Ctrl+C）  サンプリング %g Hz' % HEADLESS_RATE)

    # ライブグラフ（--live）。matplotlib が無い/失敗しても記録は続ける。
    live = None
    if LIVE_PLOT:
        try:
            live = LivePlot()
            print('リアルタイム表示: ON（別ウィンドウ）。終了はグラフの STOP ボタン / 窓を閉じる / q キー / 端末で Ctrl+C')
        except Exception as e:
            print('リアルタイム表示は使えませんでした（matplotlib未導入など）:', e)
            live = None

    # 過負荷監視（--force-limit）。リレーが開けなくても記録・監視は続ける（アラームのみ）。
    monitor = None
    if FORCE_LIMIT is not None:
        relay = None
        if RELAY_PORT:
            try:
                relay = SerialRelay(RELAY_PORT)
                print('過負荷時のリレー出力: ON（ポート %s）' % RELAY_PORT)
            except Exception as e:
                print('リレーを開けませんでした（アラームのみで続行）:', e)
                relay = None
        alarm_path = os.path.splitext(log_path)[0] + '_alarms.csv'
        monitor = ForceLimitMonitor(FORCE_LIMIT, hold=FORCE_LIMIT_HOLD,
                                    mask_s=FORCE_LIMIT_MASK, latch=FORCE_LIMIT_LATCH,
                                    alarm_path=alarm_path, relay=relay)
        print('過負荷監視: |F| ≥ %.1fN が %d サンプル連続で発報（%s%s%s）'
              % (FORCE_LIMIT, FORCE_LIMIT_HOLD,
                 'マスク%.0fs / ' % FORCE_LIMIT_MASK if FORCE_LIMIT_MASK else '',
                 'ラッチ保持' if FORCE_LIMIT_LATCH else '自動復帰',
                 ' / リレー出力' if relay is not None else ' / アラームのみ'))

    dt = 1.0 / HEADLESS_RATE
    t_start = time.time()
    log_count = 0
    try:
        while True:
            t0 = time.time()
            fx, fy, fz, mx, my, mz = read_wrench(t0)
            ts = t0 - t_start
            if log_count % LOG_EVERY == 0:
                logger.write(ts, (fx, fy, fz, mx, my, mz), None, True)
            log_count += 1
            # 端末に簡易ライブ表示（1行を上書き更新）
            fmag = math.sqrt(fx * fx + fy * fy + fz * fz)
            # 過負荷監視（生の |F| で評価＝スパイクを取りこぼさない）
            if monitor is not None:
                monitor.update(ts, fmag)
            print('\r t=%6.1fs  F=(%6.2f,%6.2f,%6.2f)N |F|=%5.2f  '
                  'M=(%6.3f,%6.3f,%6.3f)Nm   ' %
                  (ts, fx, fy, fz, fmag, mx, my, mz), end='')
            if live is not None:
                live.push(ts, (fx, fy, fz), (mx, my, mz))
                live.maybe_draw(t0)
                if live.stop_requested:
                    print('\n（STOP操作を検知しました）')
                    break
            elapsed = time.time() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)
    except KeyboardInterrupt:
        pass
    finally:
        logger.close()
        if live is not None:
            live.close()
        if monitor is not None:
            if monitor.events:
                print('過負荷 発報 %d 回（記録: %s）' % (monitor.events, monitor.alarm_path))
            monitor.close()
        print('\nCSV記録を保存しました（%d 行）: %s' % (logger.rows, logger.path))
        if PLOT_ON_SAVE:
            print('グラフを表示します（plot_force_log.py）…')
            _launch_plot(logger.path)
        else:
            _reveal_in_explorer(logger.path)
        if _SENSOR is not None:
            _SENSOR.close()


def main():
    global _SENSOR

    _import_robolink()   # RoboDK 連携モードのみ robolink を読み込む
    RDK, robot = _connect_robot()

    # 実センサ使用時はここで接続。無負荷状態で零点を実測してから開始する。
    if not USE_DEMO_SIGNAL:
        _SENSOR = DynPickSensor(port=DYNPICK_PORT, baudrate=DYNPICK_BAUD)
        _SENSOR.open()
        if TARE_ON_START:
            RDK.ShowMessage('DynPick 零点測定中… ツールに触れないでください', False)
            _SENSOR.tare(TARE_SAMPLES)

    RDK.ShowMessage('力/モーメント表示を開始（Ctrl+Cで終了）', False)
    f_arrow = WrenchArrow(RDK, FORCE_COLOR, 'Force_vector')
    m_arrow = WrenchArrow(RDK, MOMENT_COLOR, 'Moment_vector')

    # CSV ログ（動作中の力/モーメントを記録）。有効時のみファイルを開く。
    logger = None
    if LOG_CSV:
        log_path = LOG_PATH or _make_log_path()
        logger = ForceLogger(log_path)
        RDK.ShowMessage('CSV記録中: ' + log_path, False)
        print('CSV記録先:', log_path)
    log_count = 0

    f_ema = [0.0, 0.0, 0.0]
    m_ema = [0.0, 0.0, 0.0]
    dt = 1.0 / UPDATE_RATE
    t_start = time.time()

    prev_joints = None       # 関節角変化での動作検知に使う直前の関節角
    last_move_t = 0.0        # 最後に動作を検知した時刻（MOTION_HOLD_S のホールドに使用）

    # RoboDK API 通信断のとき再接続に使う例外群
    comm_errors = (ConnectionError, OSError, struct.error)

    try:
        while True:
            t0 = time.time()

            try:
                if USE_DEMO_SIGNAL or not ACTIVE_ONLY_WHEN_MOVING:
                    moving = True   # デモ／常時表示は動作検知に依存しない
                else:
                    # --- 動作検知: robot.Busy() と 関節角変化 の併用 ---
                    raw_moving = False
                    if MOTION_DETECT in ('busy', 'both'):
                        raw_moving = (robot.Busy() == 1)
                    if MOTION_DETECT in ('joints', 'both'):
                        cur_j = _joints_list(robot)
                        if prev_joints is not None and len(cur_j) == len(prev_joints):
                            dmax = max(abs(cur_j[i] - prev_joints[i])
                                       for i in range(len(cur_j)))
                            if dmax >= JOINT_MOVE_DEG:
                                raw_moving = True
                        prev_joints = cur_j
                    # 検知後ホールド（低速時や一定姿勢での瞬間的な途切れを防ぐ）
                    if raw_moving:
                        last_move_t = t0
                    moving = raw_moving or ((t0 - last_move_t) <= MOTION_HOLD_S)

                if moving:
                    fx, fy, fz, mx, my, mz = read_wrench(t0)
                    f_tool = _remap([fx, fy, fz], AXIS_MAP_FORCE)
                    m_tool = _remap([mx, my, mz], AXIS_MAP_MOMENT)
                    f_ema = _ema(f_ema, f_tool, EMA_ALPHA)
                    m_ema = _ema(m_ema, m_tool, EMA_ALPHA)

                    # TCP の絶対姿勢（ステーション基準）を基点にする。
                    # robot.Pose() は「アクティブな参照フレーム基準」のため、別フレームが
                    # 有効だと矢印が空中に飛ぶ。参照フレームに依存しない FK から絶対姿勢を求める。
                    tcp = robot.PoseAbs() * robot.SolveFK(robot.Joints(), robot.PoseTool())
                    # 基点（必要なら BASE_OFFSET_TOOL でセンサ原点/刃先へずらす）
                    off_world = _rot_vec(tcp, BASE_OFFSET_TOOL)
                    tcp_pos = tcp.Pos()
                    p0 = [tcp_pos[i] + off_world[i] for i in range(3)]
                    f_world = _rot_vec(tcp, f_ema)   # ツール座標 -> ワールド方向
                    m_world = _rot_vec(tcp, m_ema)

                    # CSV記録（動作中のみ。LOG_EVERY で間引き可）
                    if logger is not None:
                        if log_count % LOG_EVERY == 0:
                            logger.write(t0 - t_start,
                                         (fx, fy, fz, mx, my, mz),
                                         tcp_pos, True)
                        log_count += 1

                    fmag = _norm(f_ema)
                    if fmag >= FORCE_DEADBAND:
                        L = min(fmag * FORCE_SCALE, MAX_ARROW_LEN)
                        f_arrow.update(p0, f_world, L)
                    else:
                        f_arrow.clear()

                    mmag = _norm(m_ema)
                    if mmag >= MOMENT_DEADBAND:
                        L = min(mmag * MOMENT_SCALE, MAX_ARROW_LEN)
                        m_arrow.update(p0, m_world, L)
                    else:
                        m_arrow.clear()
                else:
                    f_arrow.clear()
                    m_arrow.clear()

            except comm_errors as e:
                # RoboDK が高頻度の描画更新でAPIソケットを切ることがある。
                # 落とさずに再接続してループを続ける。
                print('RoboDK 接続が切れました。再接続します…', e)
                time.sleep(0.5)
                try:
                    RDK, robot = _connect_robot()
                    f_arrow.RDK = RDK
                    m_arrow.RDK = RDK
                    f_arrow.item = None   # 旧ハンドルは無効なので捨てる
                    m_arrow.item = None
                    prev_joints = None    # 関節角の連続性が切れたので基準をリセット
                except comm_errors as e2:
                    print('再接続に失敗。1秒後に再試行:', e2)
                    time.sleep(1.0)
                continue

            elapsed = time.time() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)

    except KeyboardInterrupt:
        pass
    finally:
        try:
            f_arrow.clear()
            m_arrow.clear()
            RDK.ShowMessage('力/モーメント表示を終了しました', False)
        except comm_errors:
            pass
        if logger is not None:
            logger.close()
            print('CSV記録を保存しました（%d 行）: %s' % (logger.rows, logger.path))
            _reveal_in_explorer(logger.path)
        if _SENSOR is not None:
            _SENSOR.close()



if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser(
        description='DynPick 力/モーメントを RoboDK 3D ビューに矢印表示')
    ap.add_argument('--demo', action='store_true',
                    help='センサを使わずダミー波形で描画テスト（USE_DEMO_SIGNAL=True 相当）')
    ap.add_argument('--port', help='DynPick のシリアルポート 例 COM3')
    ap.add_argument('--baud', type=int, help='ボーレート 例 921600')
    ap.add_argument('--robot', help='ロボット名（既定: 先頭のロボット）')
    ap.add_argument('--always-on', action='store_true',
                    help='ロボット停止中でも常時表示（手押しでの軸校正に便利）')
    ap.add_argument('--detect', choices=['busy', 'joints', 'both'],
                    help='動作検知の方法（既定: both = Busy と関節角変化の併用）')
    ap.add_argument('--log', action='store_true',
                    help='動作中の力/モーメントをCSVに記録する')
    ap.add_argument('--log-path',
                    help='CSVの保存先パス（既定: force_log_日時.csv をスクリプトと同じフォルダに生成）')
    ap.add_argument('--no-robodk', action='store_true',
                    help='RoboDKを使わずセンサ読み取り＋CSV記録のみ（ペンダントでロボットを動かす構成向け。常時記録）')
    ap.add_argument('--no-open', action='store_true',
                    help='保存後にエクスプローラーでCSVフォルダを開かない')
    ap.add_argument('--rate', type=float,
                    help='サンプリング周波数[Hz]（既定: 記録のみ %g / RoboDK連携 %g）'
                         % (HEADLESS_RATE, UPDATE_RATE))
    ap.add_argument('--live', action='store_true',
                    help='記録中に力/モーメントをリアルタイム表示（別ウィンドウ・要matplotlib）')
    ap.add_argument('--plot', action='store_true',
                    help='記録終了後に自動でグラフ(plot_force_log.py)を開く')
    ap.add_argument('--force-limit', type=float, metavar='N',
                    help='過負荷監視（工具保護）: |F|[N] がこの値以上で発報。'
                         '端末アラーム＋アラームログ＋（--relay-port指定時）リレーON')
    ap.add_argument('--force-limit-hold', type=int, metavar='K',
                    help='発報までの連続超過サンプル数（デバウンス。既定 %d）' % FORCE_LIMIT_HOLD)
    ap.add_argument('--force-limit-mask', type=float, metavar='S',
                    help='記録開始からS秒は監視しない（突入過渡での誤検知回避。既定 %g）' % FORCE_LIMIT_MASK)
    ap.add_argument('--force-limit-latch', action='store_true',
                    help='一度発報したら解除（再起動）まで保持＝リレーON維持（E-stop/HOLD向け）')
    ap.add_argument('--relay-port', metavar='COMx',
                    help='USBリレーのシリアルポート（例 COM5）。指定時、発報でリレー接点をON')
    args = ap.parse_args()

    # コマンドライン引数で冒頭パラメータを上書き（ファイルを編集せずに切替できる）
    if args.demo:
        USE_DEMO_SIGNAL = True
    if args.port:
        DYNPICK_PORT = args.port
    if args.baud:
        DYNPICK_BAUD = args.baud
    if args.robot is not None:
        ROBOT_NAME = args.robot
    if args.always_on:
        ACTIVE_ONLY_WHEN_MOVING = False
    if args.detect:
        MOTION_DETECT = args.detect
    if args.log:
        LOG_CSV = True
    if args.log_path:
        LOG_CSV = True
        LOG_PATH = args.log_path
    if args.no_robodk:
        USE_ROBODK = False
    if args.no_open:
        OPEN_FOLDER_ON_SAVE = False
    if args.rate:
        UPDATE_RATE = args.rate
        HEADLESS_RATE = args.rate
    if args.live:
        LIVE_PLOT = True
    if args.plot:
        PLOT_ON_SAVE = True
    if args.force_limit is not None:
        FORCE_LIMIT = args.force_limit
    if args.force_limit_hold is not None:
        FORCE_LIMIT_HOLD = args.force_limit_hold
    if args.force_limit_mask is not None:
        FORCE_LIMIT_MASK = args.force_limit_mask
    if args.force_limit_latch:
        FORCE_LIMIT_LATCH = True
    if args.relay_port:
        RELAY_PORT = args.relay_port

    if USE_ROBODK:
        main()
    else:
        # RoboDK 未使用モードは常に記録する（記録が目的のため）
        main_headless()
