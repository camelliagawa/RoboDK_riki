# -*- coding: utf-8 -*-
"""
力ログを「砥石基準」または「ワールド（ロボット台座）」の座標で表示するツール

センサ座標の Fx/Fy/Fz は J6 と一緒に回るので、右面(HaR)と左面(HaL)で同じ押し付けが別の成分に
出て読みにくい。研磨に使った LS（HaR.LS / HaL.LS / kenma.LS）から各時刻の手首の姿勢を復元し、
力とモーメントを固定した座標に変換して描く。既存のグラフ（plot_force_log.py）はそのまま。

使い方:
  python world_frame.py                       # 最新の force_log_*.csv を砥石基準で表示
  python world_frame.py force_log_xxx.csv     # 指定ファイル
  python world_frame.py --frame world         # ワールド（ロボット台座 X/Y/Z）で表示
  python world_frame.py --no-show             # 画面に出さず PNG/CSV 保存のみ

必要なもの（ログと同じフォルダ）:
  ・air.csv（包丁付きの空運転。重力の差し引きに使う）
  ・その研磨で使った HaR.LS / HaL.LS / kenma.LS
    （ログ名と同じ名前のフォルダ force_log_xxx/ に入れてあれば、そちらを優先する）
  LS が無ければ、その旨を表示して通常のグラフ（plot_force_log.py）を開く。

座標:
  wheel … Kenma point（UF9）基準。Fn=押し付け(+Z) / Ft=引きずり(−Y、刃の動きと逆向きが＋) / Fa=刃長方向(X)
  world … ロボット台座基準の X/Y/Z
  ※ センサ取付向きの推定に ±10° 程度の不確かさがある（2026-10-28 校正、handoff §0.3）。
    Fn と |F| は信頼できるが、Ft（引きずり）は参考値として見ること。

出力: wheel_<ログ名>.png / wheel_<ログ名>.csv（world なら world_<ログ名>）
依存: numpy, matplotlib
"""

import argparse
import math
import os
import re
import sys

import numpy as np

import plot_force_log as pfl

# =================== 校正値（2026-10-28 の air / LS から求めた値）===================
# センサ取付向き：列＝センサの x,y,z 軸をフランジ座標で表したもの。
# 空運転の重力（包丁＋グリッパ ≈4.8N）から推定。残差 0.17N。不確かさ ±10° 程度。
MOUNT_R = np.array([[-0.998, 0.003, 0.068],
                    [0.007, -0.989, 0.150],
                    [0.068, 0.151, 0.986]])
# LS の動作時間モデル：低速区間＝距離/指令速度 × K_SLOW、早送り1区間＝A_FAST 秒
K_SLOW = 1.96
A_FAST = 0.13
# J6 反転（左面側の重力段差）と面の端の時間差 [s]（kenma の関節移動ぶん）
HAR_END_TO_FLIP = 7.3
FLIP_TO_HAL_START = 7.4
# ストロークの山で時刻を合わせるときの探索幅 [s] と、使う山の最小の高さ [N]
PEAK_SEARCH_S = 0.6
PEAK_PROM_N = 0.8
# 山で合わせられたストロークがこれ未満なら、反転の時刻だけで合わせる
MIN_PEAKS = 8
# =====================================================================================


# ---------- 回転 ----------

def _rx(a):
    a = math.radians(a)
    return np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])


def _ry(a):
    a = math.radians(a)
    return np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])


def _rz(a):
    a = math.radians(a)
    return np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])


def wpr(w, p, r):
    """Fanuc の W,P,R → 回転行列（Rz(R)·Ry(P)·Rx(W)）"""
    return _rz(r) @ _ry(p) @ _rx(w)


def _log(R):
    c = max(-1.0, min(1.0, (np.trace(R) - 1) / 2))
    th = math.acos(c)
    if th < 1e-9:
        return np.zeros(3)
    return th / (2 * math.sin(th)) * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


def _exp(w):
    th = np.linalg.norm(w)
    if th < 1e-12:
        return np.eye(3)
    k = w / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


def wrist_R(j):
    """LR Mate 200iD の関節角 → 台座から見たフランジの向き（LS の関節/直交の組で誤差0.001°を確認）"""
    R0 = np.array([[0, 0, 1], [0, -1, 0], [1, 0, 0]], dtype=float)
    return _rz(j[0]) @ _ry(-j[2]) @ _rx(-j[3]) @ _ry(-j[4]) @ _rx(-j[5]) @ R0


# ---------- LS ----------

def parse_ls(path):
    """LS から直交点（速度・位置・TCP の向き）と UFRAME/UTOOL（PR[9]）、P[1] の関節角を読む。"""
    with open(path, encoding='latin1') as f:
        s = f.read()
    speed = dict(re.findall(r'L P\[(\d+)\] (\d+)mm/sec', s))
    pts = []
    for m in re.finditer(r'P\[(\d+)\]\{([^}]*)\}', s):
        x = re.search(r'X =\s*([-\d.]+).*?Y =\s*([-\d.]+).*?Z =\s*([-\d.]+).*?'
                      r'W =\s*([-\d.]+).*?P =\s*([-\d.]+).*?R =\s*([-\d.]+)', m.group(2), re.S)
        if x and m.group(1) in speed:
            v = [float(a) for a in x.groups()]
            pts.append({'v': float(speed[m.group(1)]), 'p': np.array(v[:3]), 'R': wpr(*v[3:])})
    pr = [float(a) for a in re.findall(r'PR\[9,\d\]=([-\d.]+)', s)]
    uframe = pr[0:6] if len(pr) >= 6 else None
    utool = pr[6:12] if len(pr) >= 12 else None
    j1 = re.search(r'P\[1\]\{.*?J1=\s*([-\d.]+).*?J2=\s*([-\d.]+).*?J3=\s*([-\d.]+).*?'
                   r'J4=\s*([-\d.]+).*?J5=\s*([-\d.]+).*?J6=\s*([-\d.]+)', s, re.S)
    joints = [float(a) for a in j1.groups()] if j1 else None
    return {'pts': pts, 'uframe': uframe, 'utool': utool, 'p1_joints': joints}


def find_ls_dir(csv_path):
    """LS の置き場所：ログ名と同じフォルダ → ログと同じフォルダ。見つからなければ None。"""
    base = os.path.splitext(csv_path)[0]
    for d in (base, os.path.dirname(os.path.abspath(csv_path))):
        if all(os.path.isfile(os.path.join(d, n)) for n in ('HaR.LS', 'HaL.LS')):
            return d
    return None


# ---------- 時刻合わせ ----------

def _smooth(x, n):
    k = np.ones(n) / n
    pad = np.pad(x, (n // 2, n - 1 - n // 2), mode='edge')
    return np.convolve(pad, k, mode='valid')


def nominal_times(pts):
    """LS の各点に着く時刻（先頭点=0）のモデル値と、各区間の指令速度。"""
    d = np.array([np.linalg.norm(pts[i]['p'] - pts[i - 1]['p']) for i in range(1, len(pts))])
    v = np.array([pts[i]['v'] for i in range(1, len(pts))])
    dt = np.where(v < 200, K_SLOW * d / v, A_FAST)
    return np.concatenate([[0.0], np.cumsum(dt)]), v


def stroke_centers(tl, v):
    """連続する低速区間（＝1ストローク）の中央の時刻。"""
    out, i = [], 0
    while i < len(v):
        if v[i] < 200:
            j = i
            while j + 1 < len(v) and v[j + 1] < 200:
                j += 1
            out.append((tl[i] + tl[j + 1]) / 2)
            i = j + 1
        else:
            i += 1
    return np.array(out)


def find_peaks(t, f, w0, w1, half=1.2, prom=PEAK_PROM_N):
    m = (t > w0) & (t < w1)
    tq, fq = t[m], _smooth(f[m], 25)
    if len(tq) < 10:
        return np.array([])
    h = max(1, int(half / np.median(np.diff(tq))))
    out = []
    for i in range(h, len(fq) - h):
        if fq[i] == fq[i - h:i + h + 1].max():
            lo = min(fq[max(0, i - 2 * h):i].min(), fq[i:i + 2 * h].min())
            if fq[i] - lo > prom:
                out.append(tq[i])
    return np.array(out)


def face_times(pts, t, f, t_anchor, anchor_is_end, w0, w1):
    """面の LS 各点の実時刻。反転の時刻で大まかに合わせ、ストロークの山で1本ずつ補正する。"""
    tl, v = nominal_times(pts)
    t0 = (t_anchor - tl[-1]) if anchor_is_end else t_anchor
    cen = stroke_centers(tl, v)
    pk = find_peaks(t, f, w0, w1)
    # 山の列全体のずれ（反転からの推定の誤差）を先に合わせる
    if len(pk):
        best = (0, 0.0)
        for sh in np.arange(-3, 3.01, 0.05):
            n = sum(np.min(np.abs(pk - c)) < 0.3 for c in cen + t0 + sh)
            if n > best[0]:
                best = (n, sh)
        t0 += best[1]
    tn = cen + t0
    ta = tn.copy()
    used = 0
    for k, c in enumerate(tn):
        if len(pk):
            j = np.argmin(np.abs(pk - c))
            if abs(pk[j] - c) < PEAK_SEARCH_S:
                ta[k] = pk[j]
                used += 1
    if used < MIN_PEAKS:
        ta = tn          # 山が少ない（ほぼ非接触）→ 反転の時刻だけで合わせる
    x = tl + t0
    out = np.interp(x, tn, ta)
    out[x < tn[0]] = x[x < tn[0]] + (ta[0] - tn[0])
    out[x > tn[-1]] = x[x > tn[-1]] + (ta[-1] - tn[-1])
    return out, used, len(cen)


def flip_edges(t, F, u, dur=None):
    """J6 反転（左面側）の区間。重力段差の向き u への射影に矩形を当てはめる。"""
    p = _smooth(F @ u, 25)
    cs = np.concatenate([[0.0], np.cumsum(p)])
    n = len(p)
    durs = np.arange(max(5.0, dur - 3), dur + 3, 0.2) if dur else np.arange(20.0, t[-1], 0.2)
    best = (-1e18, None, None)
    for t1 in np.arange(t[0], t[-1], 0.2):
        i1 = np.searchsorted(t, t1)
        for D in durs:
            i2 = np.searchsorted(t, t1 + D)
            if i2 >= n:
                break
            sc = (cs[i2] - cs[i1]) / (i2 - i1) - (cs[-1] - cs[i2] + cs[i1]) / max(1, n - (i2 - i1))
            if sc > best[0]:
                best = (sc, t[i1], t[i2])
    return best[1], best[2]


# ---------- 変換 ----------

def orientations(pts, times, tq):
    """各時刻の TCP の向き（UF 基準）。面の区間外は NaN。"""
    out = np.full((len(tq), 3, 3), np.nan)
    idx = np.searchsorted(times, tq, side='right') - 1
    for n, (i, tt) in enumerate(zip(idx, tq)):
        if 0 <= i < len(pts) - 1:
            u = (tt - times[i]) / max(times[i + 1] - times[i], 1e-6)
            Ri = pts[i]['R']
            out[n] = Ri @ _exp(u * _log(Ri.T @ pts[i + 1]['R']))
    return out


def _cols(d, keys):
    return np.array([d[k] for k in keys], dtype=float).T


def transform(csv_path, frame='wheel', air_path=None, ls_dir=None, log=print):
    """力ログを変換して dict を返す。LS か air が無ければ None。"""
    ls_dir = ls_dir or find_ls_dir(csv_path)
    if ls_dir is None:
        log('LS（HaR.LS / HaL.LS）がログと同じフォルダにありません。')
        return None
    air_path = air_path or pfl._find_air_csv([os.path.dirname(os.path.abspath(csv_path))])
    if air_path is None:
        log('air.csv（包丁付きの空運転）が見つかりません。')
        return None
    d = pfl.load_and_validate(csv_path)
    base = pfl.load_and_validate(air_path)
    raw = _cols(d, ('fx_N', 'fy_N', 'fz_N'))
    for line in pfl.apply_baseline_persides(d, base):
        log(line)
    t = np.array(d['t_s'])
    F = _cols(d, ('fx_N', 'fy_N', 'fz_N'))
    M = _cols(d, ('mx_Nm', 'my_Nm', 'mz_Nm'))
    fmag = np.linalg.norm(F, axis=1)

    LS = {s: parse_ls(os.path.join(ls_dir, 'Ha%s.LS' % s)) for s in 'RL'}
    kenma = os.path.join(ls_dir, 'kenma.LS')
    uf = LS['R']['uframe'] or [0, 0, 0, 0, 0, 0]
    ut = LS['R']['utool'] or [0, 0, 0, -90, 0, 90]
    R_base_uf = wpr(*uf[3:6])
    R_tcp_in_fl = wpr(*ut[3:6])

    # 反転の区間：air の重力段差の向きと長さを手本に研磨ログで探す
    ta = np.array(base['t_s'])
    Fa = _cols(base, ('fx_N', 'fy_N', 'fz_N'))
    half = len(ta) // 2
    u = np.median(Fa[half:half + len(ta) // 4], axis=0) - np.median(Fa[:len(ta) // 4], axis=0)
    u = u / np.linalg.norm(u)
    a0, a1 = flip_edges(ta, Fa, u)
    g0, g1 = flip_edges(t, raw, u, dur=a1 - a0)
    if g0 is None:
        log('J6 反転（左面の重力段差）が見つかりません。両面を研磨し終えたログか確認してください。')
        return None
    pin = raw[(t > g0) & (t < g1)] @ u
    pout = raw[(t < g0) | (t > g1)] @ u
    if len(pin) == 0 or len(pout) == 0 or np.median(pin) - np.median(pout) < 3.0:
        log('J6 反転（左面の重力段差）が見つかりません。両面を研磨し終えたログか確認してください。')
        return None
    log('J6 反転の区間: 研磨 %.1f〜%.1f s（空運転 %.1f〜%.1f s）' % (g0, g1, a0, a1))

    Fw = np.full_like(F, np.nan)
    Mw = np.full_like(M, np.nan)
    faces = {}
    for side, (anchor, is_end, w0, w1) in (('R', (g0 - HAR_END_TO_FLIP, True, 0.0, g0 - 1.0)),
                                           ('L', (g0 + FLIP_TO_HAL_START, False, g0 + 3.0, g1))):
        pts = LS[side]['pts']
        times, used, n = face_times(pts, t, fmag, anchor, is_end, w0, w1)
        idx = np.where((t > times[0]) & (t < times[-1]))[0]
        Rtcp = orientations(pts, times, t[idx])
        ok = ~np.isnan(Rtcp[:, 0, 0])
        idx = idx[ok]
        R_uf_s = np.einsum('nij,kj->nik', Rtcp[ok], R_tcp_in_fl) @ MOUNT_R   # UF ← センサ
        if frame == 'world':
            R_uf_s = np.einsum('ij,njk->nik', R_base_uf, R_uf_s)
        Fw[idx] = np.einsum('nij,nj->ni', R_uf_s, F[idx])
        Mw[idx] = np.einsum('nij,nj->ni', R_uf_s, M[idx])
        faces[side] = (times[0], times[-1], used, n)
        log('%s面: %.1f〜%.1f s、ストロークの山で時刻合わせ %d/%d 本%s'
            % ('右' if side == 'R' else '左', times[0], times[-1], used, n,
               '' if used >= MIN_PEAKS else '（山が少ないので反転の時刻だけで合わせた）'))
    if frame == 'wheel':
        Fw[:, 1] *= -1        # Ft：刃の動き(+Y)と逆向きを＋にする
    return {'t': t, 'F': Fw, 'M': Mw, 'fmag': fmag, 'mmag': np.linalg.norm(M, axis=1),
            'faces': faces, 'frame': frame, 'air': air_path, 'ls_dir': ls_dir,
            'kenma': kenma if os.path.isfile(kenma) else None}


# グラフの文字は既存の plot_force_log.py に合わせて英語（日本語フォントが無いPCでも文字化けしない）
LABELS = {
    'wheel': (('Fa along blade (X)', 'Ft drag (-Y) *ref.', 'Fn push (+Z)'),
              ('Mx (Kenma)', 'My (Kenma)', 'Mz (Kenma)'), 'wheel frame (Kenma point)'),
    'world': (('Fx (world)', 'Fy (world)', 'Fz (world)'), ('Mx (world)', 'My (world)', 'Mz (world)'),
              'world frame (robot base)'),
}
SHORT = {'wheel': ('Fa', 'Ft', 'Fn'), 'world': ('Fx', 'Fy', 'Fz')}


def summary(res, log=print):
    t, F = res['t'], res['F']
    for side, (a, b, used, n) in res['faces'].items():
        m = (t > a) & (t < b) & ~np.isnan(F[:, 0])
        if not m.any():
            continue
        fm = res['fmag'][m]
        hi = fm > np.percentile(fm, 70)
        log('%s面（接触の強い上位30%%の平均）: ' % ('右 HaR' if side == 'R' else '左 HaL') +
            ' / '.join('%s %.2f N' % (nm, v) for nm, v in zip(SHORT[res['frame']], F[m][hi].mean(0))))


def save_csv(res, path):
    names = ('fa_N', 'ft_N', 'fn_N') if res['frame'] == 'wheel' else ('fx_N', 'fy_N', 'fz_N')
    with open(path, 'w', encoding='utf-8', newline='') as f:
        f.write('t_s,%s,%s,%s,mx_Nm,my_Nm,mz_Nm,Fmag_N\n' % names)
        for i in range(len(res['t'])):
            vals = [res['t'][i], *res['F'][i], *res['M'][i], res['fmag'][i]]
            f.write(','.join('' if v != v else '%.4f' % v for v in vals) + '\n')


def plot(res, title, png, show=True):
    import warnings
    warnings.filterwarnings('ignore', message='Glyph')
    import matplotlib
    if not show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fl, ml, fname = LABELS[res['frame']]
    t = res['t']
    colors = ('#16A085', '#8E44AD', '#C0392B') if res['frame'] == 'wheel' else ('#D55E00', '#009E73', '#0072B2')
    fig, ax = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    for i in range(3):
        ax[0].plot(t, res['F'][:, i], color=colors[i], lw=1.0, label=fl[i])
        ax[1].plot(t, res['M'][:, i], color=colors[i], lw=1.0, label=ml[i])
    ax[0].plot(t, res['fmag'], color='#222222', lw=0.8, alpha=0.5, label='|F|')
    ax[1].plot(t, res['mmag'], color='#222222', lw=0.8, alpha=0.5, label='|M|')
    for side, (a, b, used, n) in res['faces'].items():
        for x in ax:
            x.axvspan(a, b, color='#F0C000' if side == 'R' else '#3498DB', alpha=0.06)
        ax[0].text((a + b) / 2, 1.0, 'right HaR' if side == 'R' else 'left HaL', transform=ax[0].get_xaxis_transform(),
                   ha='center', va='bottom', fontsize=9)
    ax[0].set_ylabel('Force [N]')
    ax[1].set_ylabel('Moment [N*m]')
    ax[1].set_xlabel('Time [s]')
    for x in ax:
        x.grid(alpha=0.4)
        x.legend(loc='upper right', fontsize=8, ncol=4)
    fig.suptitle('%s  -  %s' % (title, fname), fontsize=11)
    if res['frame'] == 'wheel':
        fig.text(0.01, 0.005, 'air subtracted. Sensor mounting has ~±10 deg uncertainty: Fn and |F| are reliable, '
                 'Ft is for reference only. Components are shown only inside each face.', fontsize=8, color='#666666')
    fig.tight_layout(rect=(0, 0.02, 1, 0.97))
    fig.savefig(png, dpi=120)
    if show:
        plt.show()


def main(argv=None):
    ap = argparse.ArgumentParser(description='力ログを砥石基準／ワールド座標で表示')
    ap.add_argument('csv', nargs='?', help='force_log_*.csv（省略時は最新）')
    ap.add_argument('--frame', choices=('wheel', 'world'), default='wheel',
                    help='wheel=砥石基準(Fn/Ft/Fa, 既定) / world=ロボット台座 X/Y/Z')
    ap.add_argument('--air', help='空運転CSV（省略時は同じフォルダの air.csv）')
    ap.add_argument('--ls-dir', help='HaR.LS / HaL.LS / kenma.LS のあるフォルダ（省略時は自動）')
    ap.add_argument('--no-show', action='store_true', help='画面に出さず保存のみ')
    a = ap.parse_args(argv)
    here = os.path.dirname(os.path.abspath(__file__))
    path = a.csv or pfl.find_latest_csv(os.getcwd()) or pfl.find_latest_csv(here)
    if not path:
        print('force_log_*.csv が見つかりません。')
        return 1
    print('ファイル :', path)
    res = transform(path, a.frame, a.air, a.ls_dir)
    if res is None:
        print('→ 座標変換はできないので、通常のグラフを表示します。')
        sys.argv = [sys.argv[0], path, '--sides', '--auto-baseline'] + (['--no-show'] if a.no_show else [])
        return pfl.main()
    summary(res)
    # 出力名は force_log_*.csv に一致させない（plot_sides 等が「最新ログ」と取り違えないように）
    folder, name = os.path.split(os.path.splitext(path)[0])
    stem = os.path.join(folder, '%s_%s' % (a.frame, name))
    save_csv(res, stem + '.csv')
    plot(res, os.path.basename(path), stem + '.png', show=not a.no_show)
    print('保存 :', stem + '.png', '/', stem + '.csv')
    return 0


if __name__ == '__main__':
    sys.exit(main())
