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

import time
import math
from robolink import Robolink, ITEM_TYPE_ROBOT, PROJECTION_NONE
from dynpick_sensor import DynPickSensor

# =================== 調整パラメータ ===================
ROBOT_NAME = ''          # '' で最初のロボット。名前指定も可 例: 'Fanuc LR Mate 200iD/7L'
USE_DEMO_SIGNAL = False  # True: ダミー正弦波でRoboDK描画テスト / False: read_wrench()の実センサ値

# --- DynPick 実センサ接続（USE_DEMO_SIGNAL=False のとき使用）---
DYNPICK_PORT = 'COM3'    # 接続ポート。Windows: 'COM3' 等 / Linux: '/dev/ttyUSB0'
DYNPICK_BAUD = 921600    # DynPick 標準ボーレート（機種により 230400 等）
TARE_ON_START = True     # 開始時に無負荷状態で零点を実測（測定前はツールに何も触れないこと）
TARE_SAMPLES  = 100      # 零点実測に使うサンプル数
# 別センサを使う場合は dynpick_sensor.py の DEFAULT_SENS_* / DEFAULT_ZERO を書き換える

FORCE_SCALE   = 5.0      # [mm/N]      力1Nあたりの矢印長さ
MOMENT_SCALE  = 300.0    # [mm/(N*m)]  モーメント矢印の長さ倍率
FORCE_COLOR   = [1.0, 0.15, 0.15]   # 力 = 赤
MOMENT_COLOR  = [0.15, 0.5, 1.0]    # モーメント = 青
FORCE_DEADBAND  = 0.3    # [N]    これ未満は非表示（ノイズ抑制）
MOMENT_DEADBAND = 0.02   # [N*m]  これ未満は非表示
MAX_ARROW_LEN   = 250.0  # [mm]   矢印長さの上限（振り切れ防止）
UPDATE_RATE     = 20     # [Hz]   更新レート（重ければ下げる）
EMA_ALPHA       = 0.4    # 0<..<=1  表示のローパス（1で無効）
ACTIVE_ONLY_WHEN_MOVING = True   # True: ロボット動作中のみ表示（自動スタート）

# センサ生値 -> ツール座標 の軸割当  (生値のindex, 符号)。取付向きに合わせて調整。
# 例: tool X <- sensor[0], tool Y <- sensor[1], tool Z <- sensor[2]
AXIS_MAP_FORCE  = [(0, +1), (1, +1), (2, +1)]
AXIS_MAP_MOMENT = [(0, +1), (1, +1), (2, +1)]
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


def main():
    global _SENSOR

    RDK = Robolink()
    robot = RDK.Item(ROBOT_NAME, ITEM_TYPE_ROBOT)
    if not robot.Valid():
        raise Exception('ロボットが見つかりません。ROBOT_NAME を確認してください。')

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

    f_ema = [0.0, 0.0, 0.0]
    m_ema = [0.0, 0.0, 0.0]
    dt = 1.0 / UPDATE_RATE

    try:
        while True:
            t0 = time.time()

            if USE_DEMO_SIGNAL:
                moving = True   # デモは常時表示（動作検知に依存しない）
            else:
                moving = (robot.Busy() == 1) if ACTIVE_ONLY_WHEN_MOVING else True

            if moving:
                fx, fy, fz, mx, my, mz = read_wrench(t0)
                f_tool = _remap([fx, fy, fz], AXIS_MAP_FORCE)
                m_tool = _remap([mx, my, mz], AXIS_MAP_MOMENT)
                f_ema = _ema(f_ema, f_tool, EMA_ALPHA)
                m_ema = _ema(m_ema, m_tool, EMA_ALPHA)

                # TCP の絶対姿勢（ステーション基準）を基点にする
                tcp = robot.PoseAbs() * robot.Pose()
                p0 = tcp.Pos()
                f_world = _rot_vec(tcp, f_ema)   # ツール座標 -> ワールド方向
                m_world = _rot_vec(tcp, m_ema)

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

            elapsed = time.time() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)

    except KeyboardInterrupt:
        pass
    finally:
        f_arrow.clear()
        m_arrow.clear()
        if _SENSOR is not None:
            _SENSOR.close()
        RDK.ShowMessage('力/モーメント表示を終了しました', False)


if __name__ == '__main__':
    main()
