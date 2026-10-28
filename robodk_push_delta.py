# -*- coding: utf-8 -*-
"""
押し込み深さ δ を RoboDK 上で数値入力するマクロ（CSV を作り直さずに δ を変える）

仕組み :
  HaR / HaL がそれぞれ専用の包丁ツール（例 blade-HaR / blade-HaL）を使っている前提で、
  そのツールの「立体を移動」（ジオメトリ位置）の Y と Rz を δ から計算して書き込む。
    Y  … その面の押し込みを刃長方向に一様に変える（＝アゴ δ）
    Rz … 刃元を支点に、刃先ほど押し込みを変える（＝切先 δ − アゴ δ）
  δ は生成器と同じく「＋＝深く」[mm]。δ=0 の基準は BASE_Y / BASE_RZ（両面共通の現行値）。

使い方 :
  1) RoboDK で包丁ツールを2つに複製し、HaR / HaL のプロジェクトに別々に割り当てる（下の TOOL_* と名前を合わせる）
  2) RoboDK のメニュー［プログラム］→［Python プログラムを追加］でこのファイルを読み込み、実行する
     （または RoboDK 起動中に  python robodk_push_delta.py ）
  3) ダイアログで各面のアゴ/切先 δ を入力 → OK。プロジェクトの更新（パス再計算）まで自動で行う
  4) LS を出力し、転送前に LS 逆算で包丁位置を確認する（handoff §0.3）

注意 :
  ・ツールを複製せず両面が同じツールのままだと、片面の δ がもう片面に逆向きに効く（プロジェクトのツールが違えば止める）
  ・プロジェクトのオフセット transl は触らない（transl とこの δ は足し算で効く）
  ・BLADE_LEN_MM は研磨ルートCSVの刃長（Y列の最大−最小）。包丁を変えたら見直す
"""

import math
import sys

# =================== 設定（RoboDK 上の名前・基準値に合わせる）===================
TOOL_R = 'blade-HaR'          # HaR（右面）専用の包丁ツール名
TOOL_L = 'blade-HaL'          # HaL（左面）専用の包丁ツール名
PROJ_R = 'HaR'                # 曲線を辿るプロジェクト名（更新に使う）
PROJ_L = 'HaL'
BASE_Y  = -1.600              # δ=0 のときの「立体を移動」Y [mm]（2026-10-28 確定値）
BASE_RZ = -90.494             # δ=0 のときの「立体を移動」Rz [deg]
BLADE_LEN_MM = 179.8          # 刃長 [mm]（研磨ルートCSVの Y 範囲。アゴ=0 → 切先）
DELTA_WARN_MM = 3.0           # これを超える δ は確認を出す
# ============================================================================

# 面ごとの「深く」の向き（ツールX方向の符号）。
# 右面の外向き法線＝ツール−X なので、包丁モデルを +X に寄せると深くなる。左面はその逆。
# 2026-10-28 実測：Y を負へ動かすと右面の刃元が浅くなった（handoff §0.3）。
DEEPER_SIGN = {'R': +1, 'L': -1}


# ---------- 数式（RoboDK 不要。単体で検算できる）----------

def edge_x(y, rz_deg, w_deg, s):
    """刃長位置 s [mm] の包丁中心面のツールX位置 [mm]（ジオメトリ Y, Rz, W から）。"""
    return y + math.cos(math.radians(rz_deg)) * math.cos(math.radians(w_deg)) * s


def delta_to_geom(side, heel, tip, w_deg, base_y=BASE_Y, base_rz=BASE_RZ, length=BLADE_LEN_MM):
    """面 side('R'/'L') のアゴ/切先 δ [mm] から「立体を移動」の (Y, Rz) を返す。"""
    sg = DEEPER_SIGN[side]
    x_heel = edge_x(base_y, base_rz, w_deg, 0.0) + sg * heel
    x_tip = edge_x(base_y, base_rz, w_deg, length) + sg * tip
    c = (x_tip - x_heel) / (length * math.cos(math.radians(w_deg)))
    if abs(c) > 0.2:
        raise ValueError('δ の差が大きすぎます（刃長に対して傾きが大きすぎる）')
    # Rz は −90° 付近の解を選ぶ（cos(Rz)=c, sin(Rz)<0）
    rz = -math.degrees(math.acos(c))
    return x_heel, rz


def geom_to_delta(side, y, rz_deg, w_deg, base_y=BASE_Y, base_rz=BASE_RZ, length=BLADE_LEN_MM):
    """現在の (Y, Rz) から面 side のアゴ/切先 δ [mm] を逆算する（ダイアログの初期値用）。"""
    sg = DEEPER_SIGN[side]
    heel = sg * (edge_x(y, rz_deg, w_deg, 0.0) - edge_x(base_y, base_rz, w_deg, 0.0))
    tip = sg * (edge_x(y, rz_deg, w_deg, length) - edge_x(base_y, base_rz, w_deg, length))
    return heel, tip


# ---------- Fanuc XYZWPR ⇔ 4x4（RoboDK の Fanuc 表記: Rz(R)·Ry(P)·Rx(W)）----------

def fanuc_to_rows(x, y, z, w, p, r):
    a, b, c = math.radians(w), math.radians(p), math.radians(r)
    ca, sa, cb, sb, cc, sc = math.cos(a), math.sin(a), math.cos(b), math.sin(b), math.cos(c), math.sin(c)
    return [[cc * cb, cc * sb * sa - sc * ca, cc * sb * ca + sc * sa, x],
            [sc * cb, sc * sb * sa + cc * ca, sc * sb * ca - cc * sa, y],
            [-sb, cb * sa, cb * ca, z],
            [0.0, 0.0, 0.0, 1.0]]


def rows_to_fanuc(m):
    p = math.degrees(math.atan2(-m[2][0], math.hypot(m[0][0], m[1][0])))
    w = math.degrees(math.atan2(m[2][1], m[2][2]))
    r = math.degrees(math.atan2(m[1][0], m[0][0]))
    return [m[0][3], m[1][3], m[2][3], w, p, r]


# ---------- RoboDK 連携 ----------

def _import_robodk():
    try:
        from robolink import Robolink, ITEM_TYPE_TOOL, ITEM_TYPE_MACHINING   # RoboDK 同梱 Python
        from robodk import Mat
    except ImportError:
        from robodk.robolink import Robolink, ITEM_TYPE_TOOL, ITEM_TYPE_MACHINING   # pip install robodk
        from robodk.robomath import Mat
    return Robolink, ITEM_TYPE_TOOL, ITEM_TYPE_MACHINING, Mat


def _pose_rows(pose):
    return [[pose[i, j] for j in range(4)] for i in range(4)]


def ask_deltas(init):
    """各面のアゴ/切先 δ を入力するダイアログ。init={'R':(h,t),'L':(h,t)}。キャンセルで None。"""
    try:
        import tkinter as tk
    except ImportError:
        return _ask_console(init)
    res = {}
    root = tk.Tk()
    root.title('押し込み δ（＋＝深く, mm）')
    tk.Label(root, text='δ は「立体を移動」の基準（Y %.3f / Rz %.3f）からの量' % (BASE_Y, BASE_RZ)).grid(
        row=0, column=0, columnspan=3, padx=8, pady=6)
    tk.Label(root, text='アゴ').grid(row=1, column=1)
    tk.Label(root, text='切先').grid(row=1, column=2)
    ents = {}
    for i, (side, label) in enumerate((('R', '右面 HaR'), ('L', '左面 HaL'))):
        tk.Label(root, text=label).grid(row=2 + i, column=0, padx=8, sticky='w')
        for j in range(2):
            e = tk.Entry(root, width=8, justify='right')
            e.insert(0, '%.2f' % init[side][j])
            e.grid(row=2 + i, column=1 + j, padx=4, pady=2)
            ents[side, j] = e
    upd = tk.BooleanVar(value=True)
    tk.Checkbutton(root, text='プロジェクトを更新（パス再計算）', variable=upd).grid(
        row=4, column=0, columnspan=3, pady=4)

    def ok():
        try:
            for side in ('R', 'L'):
                res[side] = (float(ents[side, 0].get()), float(ents[side, 1].get()))
        except ValueError:
            tk.Label(root, text='数値を入力してください', fg='red').grid(row=6, column=0, columnspan=3)
            res.clear()
            return
        res['update'] = upd.get()
        root.destroy()

    tk.Button(root, text='OK', width=10, command=ok).grid(row=5, column=1, pady=8)
    tk.Button(root, text='キャンセル', width=10, command=root.destroy).grid(row=5, column=2, pady=8)
    root.mainloop()
    return res or None


def _ask_console(init):
    res = {}
    for side, label in (('R', '右面 HaR'), ('L', '左面 HaL')):
        vals = []
        for j, name in enumerate(('アゴ', '切先')):
            s = input('%s %s δ [mm] (現在 %.2f): ' % (label, name, init[side][j])).strip()
            vals.append(float(s) if s else init[side][j])
        res[side] = tuple(vals)
    res['update'] = True
    return res


def _confirm(msg):
    try:
        import tkinter as tk
        from tkinter import messagebox
        root = tk.Tk(); root.withdraw()
        ok = messagebox.askyesno('確認', msg)
        root.destroy()
        return ok
    except ImportError:
        return input(msg + ' [y/N]: ').strip().lower() == 'y'


def main():
    Robolink, ITEM_TYPE_TOOL, ITEM_TYPE_MACHINING, Mat = _import_robodk()
    RDK = Robolink()
    tools = {'R': RDK.Item(TOOL_R, ITEM_TYPE_TOOL), 'L': RDK.Item(TOOL_L, ITEM_TYPE_TOOL)}
    for side, name in (('R', TOOL_R), ('L', TOOL_L)):
        if not tools[side].Valid():
            RDK.ShowMessage('ツール「%s」が見つかりません。包丁ツールを複製して名前を合わせてください。' % name)
            return 1
    projs = {'R': RDK.Item(PROJ_R, ITEM_TYPE_MACHINING), 'L': RDK.Item(PROJ_L, ITEM_TYPE_MACHINING)}
    # 各プロジェクトが専用ツールを使っているか確認（同じツールを共有したままだと δ が反対面にも逆向きに効く）
    for side, tname in (('R', TOOL_R), ('L', TOOL_L)):
        try:
            used = projs[side].getLink(ITEM_TYPE_TOOL)
            if projs[side].Valid() and used.Valid() and used.Name() != tname:
                RDK.ShowMessage('プロジェクト「%s」のツールが「%s」です。「%s」に切り替えてから実行してください。'
                                % (projs[side].Name(), used.Name(), tname))
                return 1
        except Exception:
            pass   # 古い API で取得できない場合は確認を省略

    cur = {s: rows_to_fanuc(_pose_rows(tools[s].GeometryPose())) for s in ('R', 'L')}
    init = {s: tuple(round(v, 3) + 0.0 for v in geom_to_delta(s, cur[s][1], cur[s][5], cur[s][3]))
            for s in ('R', 'L')}
    ans = ask_deltas(init)
    if not ans:
        return 0
    big = [v for s in ('R', 'L') for v in ans[s] if abs(v) > DELTA_WARN_MM]
    if big and not _confirm('δ が %.1f mm を超えています。続けますか？' % DELTA_WARN_MM):
        return 0

    lines = []
    for side in ('R', 'L'):
        x, y0, z, w, p, r0 = cur[side]
        heel, tip = ans[side]
        y, rz = delta_to_geom(side, heel, tip, w)
        tools[side].setGeometryPose(Mat(fanuc_to_rows(x, y, z, w, p, rz)))
        lines.append('%s: アゴ %+.2f / 切先 %+.2f mm → Y %.3f / Rz %.3f' % (
            TOOL_R if side == 'R' else TOOL_L, heel, tip, y, rz))
        if ans['update'] and projs[side].Valid():
            projs[side].Update()
    msg = '\n'.join(lines) + '\n\nLS を出力したら、転送前に包丁位置を確認してください。'
    print(msg)
    RDK.ShowMessage(msg.replace('\n', '<br>'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
