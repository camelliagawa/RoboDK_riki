#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ストローク単位の研削力解析 / 剛性同定 / 送り速度カーブ逆算

force_moment_overlay.py が記録した force_log_*.csv を後解析する。

やること:
  1. air.csv（空運転ログ）を時間同期して減算し、姿勢依存の重力オフセットを除去
  2. 研削面（HaR / HaL）を自動分割
  3. 各面のストローク（力の山）を自動検出
  4. 山の面積 ∫|F|dt を算出 —— Preston則より、これが局所除去量に比例する
  5. 左右の面積比から取り付け角度誤差を推定
  6. （押し込みスイープ時）ΔF/Δδ から等価剛性 k(x) を同定
  7. 面積を揃える送り速度カーブを逆算し、speed_profile 形式で出力

使い方:
  # 単発解析（1本のログを解析）
  python stroke_analyze.py force_log_20260717_151800.csv --air air.csv

  # 速度カーブの逆算まで行う
  python stroke_analyze.py force_log_*.csv --air air.csv --suggest-speed

  # 押し込みスイープから剛性 k(x) を同定
  python stroke_analyze.py --stiffness push_config.json

  push_config.json の書式:
    {
      "air": "air.csv",
      "tests": [
        {"push_mm": 2.0, "csv": "force_log_push20.csv"},
        {"push_mm": 2.5, "csv": "force_log_push25.csv"},
        {"push_mm": 3.0, "csv": "force_log_push30.csv"},
        {"push_mm": 3.5, "csv": "force_log_push35.csv"},
        {"push_mm": 4.0, "csv": "force_log_push40.csv"}
      ]
    }
"""

import argparse
import json
import os
import sys

import numpy as np
import pandas as pd

# =================== 調整パラメータ ===================
# --- 研削パスの諸元（LSファイル / RoboDKの曲線追従プロジェクトと合わせること）---
N_STROKES     = 19       # 片面あたりのストローク数
STROKE_DIST_MM = 50.0    # [mm] 1ストロークで砥石上を移動する距離（速度→時間の換算に使う）
# 刃長方向の各ストローク中心座標 [mm]。LSファイルのP[]から算出した実測値。
X_CENTERS = [57.9, 66.4, 76.3, 85.8, 95.5, 105.2, 115.0, 124.6, 134.1, 143.9,
             153.2, 163.3, 172.7, 182.3, 191.7, 201.1, 210.9, 220.3, 229.8]
# 現行の指令送り速度 [mm/sec]（speed_profile.json 適用後の実際の値）
SPEED_CMD = [50, 44, 39, 35, 32, 29, 27, 25, 24, 22, 21, 21, 20, 20, 19, 19, 19, 18, 18]

# --- 山（ストローク）検出 ---
PEAK_THRESHOLD  = None   # [N] 山の検出しきい値。None で自動（面ごとに最大値から決定）
THRESH_RATIO    = 0.045  # 自動しきい値 = 面内最大値 × この比率（下限 MIN_THRESHOLD）
MIN_THRESHOLD   = 0.7    # [N] 自動しきい値の下限（センサノイズより十分上に）
MIN_STROKE_S    = 0.4    # [s] これより短い山はノイズとして除外
# 面(HaR/HaL)の切り替わり判定。None で自動（山と山のギャップの中央値 × FACE_GAP_RATIO を境界とする）。
# 固定値では破綻しやすい: 面内のギャップは速度変調で 1.7〜4s と幅があり、面間は10s超。
FACE_GAP_S      = None
FACE_GAP_RATIO  = 3.0    # 自動判定の倍率（ギャップ中央値の何倍を面境界とみなすか）

# --- air減算（姿勢依存の重力オフセット除去）---
AIR_LAG_SEARCH  = 3.0    # [s] air と本番の時刻ずれの探索範囲（±）
AIR_LAG_STEP    = 0.05   # [s] 探索の刻み
AIR_SYNC_WINDOW = (0.5, 11.0)  # [s] 同期の評価に使う「接触前」区間

# --- 速度カーブ逆算 ---
CONTACT_OFFSET_S = None  # [s] 接触時間の下駄（早送り中も接触している分）。None で実測から自動推定
SPEED_MIN       = 3.0    # [mm/sec] 逆算した速度の下限（これ未満はクリップ）
SPEED_MAX       = 60.0   # [mm/sec] 同上限
# =====================================================


# ---------- 読み込み ----------
def load_log(path):
    """force_log_*.csv を読み、必要列だけの DataFrame を返す。"""
    d = pd.read_csv(path)
    need = ['t_s', 'fx_N', 'fy_N', 'fz_N']
    for c in need:
        if c not in d.columns:
            raise ValueError('%s に列 %s がありません' % (path, c))
    return d


def subtract_air(d, air_path, verbose=True):
    """air.csv を時間同期して減算し、真の接触力 Fc を付加した DataFrame を返す。

    force_moment_overlay.py は TARE_ON_START で「待機姿勢」の零点を取るため、
    研削中の姿勢では重力成分が固定オフセットとして乗る。J6で180°反転する
    HaR/HaL では、このオフセットが面ごとに大きく変わる（実測で約8.6N差）。
    同じプログラムを非接触で走らせた air.csv を引けば、これが丸ごと消える。
    """
    a = load_log(air_path)
    cols = ['fx_N', 'fy_N', 'fz_N']
    t = d.t_s.values

    # 接触前区間の残差が最小になる時刻ずれを探す
    lo, hi = AIR_SYNC_WINDOW
    m = (t > lo) & (t < hi)
    best_lag, best_sc = 0.0, None
    for lag in np.arange(-AIR_LAG_SEARCH, AIR_LAG_SEARCH + 1e-9, AIR_LAG_STEP):
        r = np.sqrt(sum(
            (d[c].values[m] - np.interp(t[m] + lag, a.t_s.values, a[c].values)) ** 2
            for c in cols))
        sc = r.mean()
        if best_sc is None or sc < best_sc:
            best_lag, best_sc = lag, sc

    for c in cols:
        d['c_' + c] = d[c].values - np.interp(t + best_lag, a.t_s.values, a[c].values)
    d['Fc'] = np.sqrt(d.c_fx_N ** 2 + d.c_fy_N ** 2 + d.c_fz_N ** 2)

    if verbose:
        print('  air同期: ラグ %+.2f s / 接触前の残差 %.3f N' % (best_lag, best_sc))
        if best_sc > 1.5:
            print('  ⚠ 残差が大きい。air.csv が同じプログラム・同じ取付で記録されたか確認を')
    return d


# ---------- 山（ストローク）検出 ----------
def _segments(tt, FF, thr):
    """しきい値を超える連続区間を [(i, j), ...] で返す。"""
    above = FF > thr
    segs, i = [], 0
    while i < len(above):
        if above[i]:
            j = i
            while j < len(above) and above[j]:
                j += 1
            if tt[j - 1] - tt[i] >= MIN_STROKE_S:
                segs.append((i, j))
            i = j
        else:
            i += 1
    return segs


def detect_strokes(d, verbose=True):
    """力の山を検出し、面(HaR/HaL)ごとにストローク情報のリストを返す。

    戻り値: [face1, face2, ...]  各 face は dict のリスト
            dict: ts, te, dur, peak, area, favg
    """
    t, F = d.t_s.values, d.Fc.values
    thr = PEAK_THRESHOLD if PEAK_THRESHOLD is not None \
        else max(MIN_THRESHOLD, F.max() * THRESH_RATIO)

    segs = _segments(t, F, thr)
    if not segs:
        return []

    # 山と山のギャップを求め、突出して大きいところを面の境界とする
    gaps = np.array([t[nxt[0]] - t[prev[1] - 1]
                     for prev, nxt in zip(segs[:-1], segs[1:])])
    if FACE_GAP_S is not None:
        gap_thr = FACE_GAP_S
    elif len(gaps):
        # 面内のギャップ（早送りの戻り）は速度変調で幅があるため、中央値基準で判定する
        gap_thr = float(np.median(gaps)) * FACE_GAP_RATIO
    else:
        gap_thr = float('inf')

    faces, cur = [], [segs[0]]
    for (prev, nxt), gap in zip(zip(segs[:-1], segs[1:]), gaps):
        if gap > gap_thr:
            faces.append(cur)
            cur = [nxt]
        else:
            cur.append(nxt)
    faces.append(cur)

    out = []
    for f in faces:
        strokes = []
        for i, j in f:
            area = np.trapezoid(F[i:j], t[i:j])
            dur = t[j - 1] - t[i]
            strokes.append(dict(ts=t[i], te=t[j - 1], dur=dur,
                                peak=F[i:j].max(), area=area,
                                favg=area / dur if dur > 0 else 0.0))
        out.append(strokes)

    if verbose:
        print('  力しきい値 %.2f N / 面境界 %.1f s / 検出: %s 山'
              % (thr, gap_thr, ' + '.join(str(len(f)) for f in out)))
        for k, f in enumerate(out):
            if len(f) != N_STROKES:
                print('  ⚠ 面%d の山が %d 個（想定 %d）。しきい値か FACE_GAP_S の調整を'
                      % (k + 1, len(f), N_STROKES))
    return out


# ---------- 表示 ----------
def report_face(name, strokes):
    """1つの面のストローク一覧を表示し、面積配列を返す。"""
    print('\n===== %s : %d ストローク =====' % (name, len(strokes)))
    print('  # |    X[mm] | 開始[s] | 幅[s] | ピーク[N] | 平均[N] | 面積[N・s]')
    for i, s in enumerate(strokes):
        x = X_CENTERS[i] if i < len(X_CENTERS) else float('nan')
        print(' %2d | %8.1f | %7.2f | %5.2f | %9.2f | %7.2f | %9.2f'
              % (i + 1, x, s['ts'], s['dur'], s['peak'], s['favg'], s['area']))
    a = np.array([s['area'] for s in strokes])
    print('  → 面積比 刃元/刃先 = %.2f 倍   （1.0 が理想＝除去量が均一）' % (a[0] / a[-1]))
    return a


def report_asymmetry(aR, aL):
    """左右の面積比から取り付け角度誤差を推定して表示する。"""
    n = min(len(aR), len(aL))
    if n < 3:
        return
    aR, aL = aR[:n], aL[:n]
    x = np.array(X_CENTERS[:n])
    ratio = aR / aL

    print('\n===== 左右差（HaR面積 / HaL面積）=====')
    print('  X[mm]     比')
    for xi, ri in zip(x, ratio):
        bar = '#' * int(min(ri, 10) * 4)
        print('  %6.1f  %5.2f  %s' % (xi, ri, bar))
    print('  刃元 %.2f → 刃先 %.2f' % (ratio[0], ratio[-1]))

    if ratio[-1] / ratio[0] > 1.5:
        # 刃先ほど比が開く＝角度誤差の特徴。押し込み量の差から傾きを概算する。
        # 「真の剛性は左右対称」を仮定し、力の比＝押し込み量の比とみなす。
        fR = np.array([a / w for a, w in zip(aR, [1] * n)])  # 相対値のみ使うのでスケール不問
        k_ref = aR[0] / ratio[0]  # 刃元は左右ほぼ一致 → ここを基準に
        d_tip = (1.0 - 1.0 / ratio[-1])  # 刃先での相対的な押し込み不足
        print('\n  → 刃先ほど差が開く＝平行ずれではなく【取り付け角度誤差】の特徴')
        print('  → HaL 側の刃先が砥石から離れている可能性が高い')
    else:
        print('\n  → 左右差は一様。角度誤差より平行ずれ／剛性差の可能性')


# ---------- 剛性同定 ----------
def analyze_stiffness(cfg_path):
    """押し込みスイープから等価剛性 k(x) を同定する。

    砥石を止めた状態で押し込み量 δ を段階的に変え、各ストロークの平均力 F を測る。
    k(x) = ΔF/Δδ （傾き）なので、δ の絶対値や接触開始点の誤差に影響されない。
    ここが「E や L を仮定せずに剛性が測れる」理由。
    """
    with open(cfg_path, encoding='utf-8') as f:
        cfg = json.load(f)
    base = os.path.dirname(os.path.abspath(cfg_path))

    def _p(p):
        return p if os.path.isabs(p) else os.path.join(base, p)

    air = _p(cfg['air'])
    pushes, faces_all = [], []
    for tst in cfg['tests']:
        print('\n[%.2f mm] %s' % (tst['push_mm'], os.path.basename(tst['csv'])))
        d = subtract_air(load_log(_p(tst['csv'])), air)
        faces = detect_strokes(d)
        if not faces:
            print('  ⚠ 山が検出できず。スキップ')
            continue
        pushes.append(tst['push_mm'])
        faces_all.append(faces)

    if len(pushes) < 3:
        print('\n⚠ 有効なデータが %d 点しかありません。最低3点（推奨5点）必要です' % len(pushes))
        return None

    pushes = np.array(pushes, float)
    n_faces = min(len(f) for f in faces_all)
    results = {}

    for fi in range(n_faces):
        name = ['HaR', 'HaL'][fi] if fi < 2 else 'face%d' % (fi + 1)
        n_st = min(len(fa[fi]) for fa in faces_all)
        k_list, r2_list = [], []
        for si in range(n_st):
            F = np.array([fa[fi][si]['favg'] for fa in faces_all])
            # 最小二乗で傾き（＝剛性）を求める
            A = np.vstack([pushes, np.ones_like(pushes)]).T
            (k, b), res, *_ = np.linalg.lstsq(A, F, rcond=None)
            ss_tot = ((F - F.mean()) ** 2).sum()
            r2 = 1 - (res[0] / ss_tot) if len(res) and ss_tot > 0 else float('nan')
            k_list.append(k)
            r2_list.append(r2)
        results[name] = dict(k=np.array(k_list), r2=np.array(r2_list))

        print('\n===== %s の等価剛性 k(x) =====' % name)
        print('  # |    X[mm] |  k [N/mm] |   R^2')
        for si, (k, r2) in enumerate(zip(k_list, r2_list)):
            x = X_CENTERS[si] if si < len(X_CENTERS) else float('nan')
            flag = '' if (r2 == r2 and r2 > 0.9) else '  ← 直線性が悪い'
            print(' %2d | %8.1f | %9.3f | %6.3f%s' % (si + 1, x, k, r2, flag))

    if 'HaR' in results and 'HaL' in results:
        kR, kL = results['HaR']['k'], results['HaL']['k']
        n = min(len(kR), len(kL))
        ratio = kR[:n] / kL[:n]
        print('\n===== 剛性の左右比 kR/kL =====')
        for si in range(n):
            print('  X=%6.1f  %5.2f' % (X_CENTERS[si], ratio[si]))
        spread = ratio.max() / ratio.min()
        print('\n  ばらつき（最大/最小）= %.2f' % spread)
        if spread < 1.3:
            print('  → 剛性は左右で一致。【真の剛性は対称】が確認された。')
            print('     よって面積の左右差は剛性差ではなく《位置決め誤差》。角度調整で解決可能。')
        else:
            print('  → 剛性そのものが左右で非対称。クランプの効き方が左右で違う。')
            print('     位置調整では解決しない。グリッパ／治具側の見直しが必要。')
    return results


# ---------- 速度カーブ逆算 ----------
def suggest_speed(strokes, name='HaR'):
    """山の面積を揃える送り速度カーブを逆算する。

    モデル:
        接触時間 w(x) = STROKE_DIST_MM / v(x) + T0
        面積     A(x) = Favg(x) * w(x)

    T0 は早送り（接近/後退）中も接触している分の「下駄」。速度変調が効かない
    原因なので、明示的にモデルへ入れる。Favg は押し込み量で決まり送り速度に
    依存しない、と仮定して逆算する（厳密でないため反復が必要）。
    """
    A = np.array([s['area'] for s in strokes])
    W = np.array([s['dur'] for s in strokes])
    Favg = np.array([s['favg'] for s in strokes])
    n = len(A)
    v_old = np.array(SPEED_CMD[:n], float)

    # 下駄 T0 の推定: w - D/v の中央値（全ストロークでほぼ一定になるはず）
    if CONTACT_OFFSET_S is not None:
        T0 = CONTACT_OFFSET_S
    else:
        t_theory = STROKE_DIST_MM / v_old
        T0 = float(np.median(W - t_theory))
    print('\n===== %s 速度カーブ逆算 =====' % name)
    print('  接触時間の下駄 T0 = %.2f s（実測 w − 理論 D/v の中央値）' % T0)
    print('  ばらつき: %.2f 〜 %.2f s' % ((W - STROKE_DIST_MM / v_old).min(),
                                          (W - STROKE_DIST_MM / v_old).max()))
    if T0 <= 0:
        print('  → 下駄なし。単純な v ∝ 面積 で逆算できます')
        T0 = 0.0

    # T0 があるため、各ストロークが取りうる面積には下限がある（速度∞でも T0 分は削る）
    A_floor = Favg * T0
    print('\n  各ストロークの「最速でもこれ以上削れてしまう」面積:')
    print('     刃元 %.1f N・s  /  刃先 %.1f N・s' % (A_floor[0], A_floor[-1]))
    A_target = A_floor.max() * 1.02   # 全ストロークが到達可能な最小の目標値
    print('  → 到達可能な目標面積 A_target = %.1f N・s' % A_target)
    print('     （刃元は T0 の下駄があるためこれ以上は下げられない）')

    v_new = np.zeros(n)
    for i in range(n):
        need = A_target / Favg[i] - T0     # 必要な定速区間の時間
        if need <= 0:
            v_new[i] = SPEED_MAX
        else:
            v_new[i] = np.clip(STROKE_DIST_MM / need, SPEED_MIN, SPEED_MAX)

    print('\n  # |    X[mm] | 現行v | 新v[mm/s] | 現面積 | 予測面積')
    for i in range(n):
        w_new = STROKE_DIST_MM / v_new[i] + T0
        print(' %2d | %8.1f | %5.0f | %9.1f | %6.1f | %8.1f'
              % (i + 1, X_CENTERS[i], v_old[i], v_new[i], A[i], Favg[i] * w_new))

    cycle_old = float((STROKE_DIST_MM / v_old).sum())
    cycle_new = float((STROKE_DIST_MM / v_new).sum())
    print('\n  研削部の所要時間: %.1f s → %.1f s （%.2f 倍）'
          % (cycle_old, cycle_new, cycle_new / cycle_old))
    print('  予測される面積比（刃元/刃先）: %.2f → 1.00' % (A[0] / A[-1]))

    if T0 > 1.0:
        print('\n  ⚠ T0 = %.2f s が大きく、これが変調の効きを殺しています。' % T0)
        print('     接近/後退の距離を増やす・退避を速くして T0 を削ると、')
        print('     刃先を極端に遅くせずに済み、サイクルタイムを大きく改善できます。')

    ratio = v_new / v_new.max()
    return dict(T0_s=T0, A_target=A_target,
                speed_mm_s=[round(float(x), 1) for x in v_new],
                speed_ratio=[round(float(x), 4) for x in ratio])


# ---------- グラフ ----------
def plot(d, faces, out_png):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    t, F = d.t_s.values, d.Fc.values
    fig = plt.figure(figsize=(13, 8))

    ax = plt.subplot(2, 1, 1)
    ax.plot(t, F, lw=0.6, color='#333')
    cols = ['tab:blue', 'tab:red', 'tab:green']
    for k, f in enumerate(faces):
        for s in f:
            ax.axvspan(s['ts'], s['te'], color=cols[k % 3], alpha=0.18)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('|F| [N]  (gravity compensated)')
    ax.set_title('Detected strokes: ' + ' + '.join(str(len(f)) for f in faces))
    ax.grid(alpha=.3)

    ax = plt.subplot(2, 2, 3)
    for k, f in enumerate(faces):
        a = [s['area'] for s in f]
        x = X_CENTERS[:len(a)]
        ax.plot(x, a, 'o-', color=cols[k % 3], label=['HaR', 'HaL'][k] if k < 2 else 'face%d' % k)
    ax.set_xlabel('Blade position X [mm]  (heel -> tip)')
    ax.set_ylabel('Stroke area [N*s]')
    ax.set_title('Removal proxy per stroke')
    ax.legend()
    ax.grid(alpha=.3)

    ax = plt.subplot(2, 2, 4)
    for k, f in enumerate(faces):
        w = [s['dur'] for s in f]
        x = X_CENTERS[:len(w)]
        ax.plot(x, w, 'o-', color=cols[k % 3], label=['HaR', 'HaL'][k] if k < 2 else 'face%d' % k)
    v = np.array(SPEED_CMD[:len(X_CENTERS)], float)
    ax.plot(X_CENTERS, STROKE_DIST_MM / v, 'k^--', label='theoretical D/v')
    ax.set_xlabel('Blade position X [mm]')
    ax.set_ylabel('Contact time [s]')
    ax.set_title('Contact time vs theory (gap = T0 offset)')
    ax.legend(fontsize=8)
    ax.grid(alpha=.3)

    plt.tight_layout()
    plt.savefig(out_png, dpi=110)
    print('\nグラフを保存: %s' % out_png)


# ---------- main ----------
def main():
    ap = argparse.ArgumentParser(description='ストローク単位の研削力解析')
    ap.add_argument('csv', nargs='?', help='force_log_*.csv')
    ap.add_argument('--air', help='air.csv（空運転ログ）。姿勢オフセット除去に使う')
    ap.add_argument('--stiffness', metavar='CONFIG.json',
                    help='押し込みスイープから剛性 k(x) を同定')
    ap.add_argument('--suggest-speed', action='store_true',
                    help='面積を揃える送り速度カーブを逆算して出力')
    ap.add_argument('--out', default='', help='出力の接頭辞（既定: 入力CSVと同じ場所）')
    args = ap.parse_args()

    if args.stiffness:
        analyze_stiffness(args.stiffness)
        return

    if not args.csv:
        ap.error('CSV を指定するか --stiffness を使ってください')

    print('読み込み: %s' % args.csv)
    d = load_log(args.csv)

    if args.air:
        d = subtract_air(d, args.air)
    else:
        print('  ⚠ --air 未指定。姿勢依存の重力オフセットが残ります')
        print('     （HaR/HaL は J6 で180°反転するため、面ごとに約8.6Nの差が乗ります）')
        d['Fc'] = np.sqrt(d.fx_N ** 2 + d.fy_N ** 2 + d.fz_N ** 2)

    faces = detect_strokes(d)
    if not faces:
        print('山が検出できませんでした。PEAK_THRESHOLD を調整してください')
        return

    names = ['HaR', 'HaL']
    areas = []
    for k, f in enumerate(faces):
        areas.append(report_face(names[k] if k < 2 else 'face%d' % (k + 1), f))

    if len(areas) >= 2:
        report_asymmetry(areas[0], areas[1])

    base = args.out or os.path.splitext(args.csv)[0]
    plot(d, faces, base + '_strokes.png')

    # ストローク一覧を CSV 出力
    rows = []
    for k, f in enumerate(faces):
        for i, s in enumerate(f):
            rows.append(dict(face=names[k] if k < 2 else 'face%d' % (k + 1),
                             stroke=i + 1,
                             x_mm=X_CENTERS[i] if i < len(X_CENTERS) else None,
                             speed_cmd=SPEED_CMD[i] if i < len(SPEED_CMD) else None,
                             t_start=round(s['ts'], 3), dur_s=round(s['dur'], 3),
                             peak_N=round(s['peak'], 3), favg_N=round(s['favg'], 3),
                             area_Ns=round(s['area'], 3)))
    pd.DataFrame(rows).to_csv(base + '_strokes.csv', index=False)
    print('ストローク一覧: %s' % (base + '_strokes.csv'))

    if args.suggest_speed:
        prof = {}
        for k, f in enumerate(faces[:2]):
            prof[names[k]] = suggest_speed(f, names[k])
        with open(base + '_speed_suggest.json', 'w', encoding='utf-8') as fp:
            json.dump(prof, fp, indent=2, ensure_ascii=False)
        print('\n速度カーブ案: %s' % (base + '_speed_suggest.json'))
        print('⚠ Favg が送り速度に依存しない仮定に基づく1次近似です。')
        print('   適用 → 再測定 → 再逆算 の反復で収束させてください。')


if __name__ == '__main__':
    main()
