# -*- coding: utf-8 -*-
"""
force_log_*.csv から 力/モーメントの時系列グラフを生成するツール。

使い方:
  python plot_force_log.py                     # 最新の force_log_*.csv を自動で開く
  python plot_force_log.py force_log_xxx.csv   # 指定ファイル
  python plot_force_log.py --no-show           # 画面表示せず PNG 保存のみ
  python plot_force_log.py --contact 3.0       # |F|>=3N の区間を加工区間として薄く塗る

出力: 同じ場所に <csvと同名>.png を保存し、画面にも表示する。
依存: matplotlib（pip install matplotlib）。numpy 等は不要。
"""

import os
import sys
import csv
import glob
import argparse

# 力[N] と モーメント[N*m] はスケールが違うので、デュアル軸にせず 2段に分ける。
# X/Y/Z の3系列は色覚異常でも区別できる Okabe-Ito 配色を固定順で割り当て、
# 合力 |F|・合モーメント |M| は太い濃色（ink）で強調する。
COLOR_X = '#D55E00'   # vermillion
COLOR_Y = '#009E73'   # green
COLOR_Z = '#0072B2'   # blue
COLOR_MAG = '#222222'  # 濃いグレー（合成値の強調線）
GRID_COLOR = '#CCCCCC'


def find_latest_csv(folder):
    """folder 内で最も新しい force_log_*.csv を返す。無ければ None。"""
    files = glob.glob(os.path.join(folder, 'force_log_*.csv'))
    if not files:
        return None
    return max(files, key=os.path.getmtime)


def load_log(path):
    """CSV を読み、列名→float リストの dict を返す。"""
    cols = {k: [] for k in
            ('t_s', 'fx_N', 'fy_N', 'fz_N', 'mx_Nm', 'my_Nm', 'mz_Nm',
             'Fmag_N', 'Mmag_Nm')}
    with open(path, 'r', newline='', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                vals = {k: float(row[k]) for k in cols}
            except (KeyError, ValueError):
                continue   # 壊れた行/空行は飛ばす
            for k in cols:
                cols[k].append(vals[k])
    return cols


def _interp(x, xs, ys):
    """xs（昇順）に対する ys を x で線形補間。範囲外は端値で外挿。"""
    n = len(xs)
    if n == 0:
        return 0.0
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    lo, hi = 0, n - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if xs[mid] <= x:
            lo = mid
        else:
            hi = mid
    span = xs[hi] - xs[lo]
    if span <= 0:
        return ys[lo]
    r = (x - xs[lo]) / span
    return ys[lo] + r * (ys[hi] - ys[lo])


def _pearson(a, b):
    """2系列の相関係数（-1..1）。分散0なら0。"""
    n = len(a)
    if n < 2:
        return 0.0
    ma = sum(a) / n
    mb = sum(b) / n
    va = sum((x - ma) ** 2 for x in a)
    vb = sum((y - mb) ** 2 for y in b)
    if va <= 0 or vb <= 0:
        return 0.0
    cov = sum((a[i] - ma) * (b[i] - mb) for i in range(n))
    return cov / (va ** 0.5 * vb ** 0.5)


def estimate_baseline_shift(d, base, max_lag=20.0, step=0.1):
    """研磨(d)と空運転(base)の |F| 波形が最もそろう時間シフト s を推定して返す。

    LSが同一なら動作は同尺で、記録開始ボタンのタイミングだけずれる。そのズレを、
    姿勢プラトー＋パスの山（歯）の共通パターンで相互相関（相関係数最大）して求める。
    返り値 s は apply_baseline(d, base, shift=s) にそのまま渡せる（base側を s 進めて d に合わせる）。
    戻り値: (s, corr)
    """
    td, yd = d['t_s'], d['Fmag_N']
    tb, yb = base['t_s'], base['Fmag_N']
    if len(td) < 2 or len(tb) < 2:
        return 0.0, 0.0
    n = int((td[-1] - td[0]) / step)
    grid = [td[0] + i * step for i in range(n + 1)]
    fd = [_interp(g, td, yd) for g in grid]
    best_s, best_c = 0.0, -2.0
    s = -max_lag
    while s <= max_lag + 1e-9:
        idx = [i for i, g in enumerate(grid) if tb[0] <= g + s <= tb[-1]]
        if len(idx) > 50:
            a = [fd[i] for i in idx]
            b = [_interp(grid[i] + s, tb, yb) for i in idx]
            c = _pearson(a, b)
            if c > best_c:
                best_c, best_s = c, s
        s += step
    return best_s, best_c


def apply_baseline(d, base, shift=0.0):
    """空運転(base)の各成分を時刻で補間して d から差し引き、重力/姿勢オフセットを除去する。

    両方とも「プログラム開始」を起点に記録している前提（同じkenmaなので時間軸が揃う）。
    ズレがあれば shift[s] で base 側の時刻をずらして合わせる。
    差し引き後に Fmag/Mmag を再計算する。
    """
    bt = base['t_s']
    comps = ('fx_N', 'fy_N', 'fz_N', 'mx_Nm', 'my_Nm', 'mz_Nm')
    for i, t in enumerate(d['t_s']):
        for k in comps:
            d[k][i] -= _interp(t + shift, bt, base[k])
        fx, fy, fz = d['fx_N'][i], d['fy_N'][i], d['fz_N'][i]
        mx, my, mz = d['mx_Nm'][i], d['my_Nm'][i], d['mz_Nm'][i]
        d['Fmag_N'][i] = (fx * fx + fy * fy + fz * fz) ** 0.5
        d['Mmag_Nm'][i] = (mx * mx + my * my + mz * mz) ** 0.5
    return d


def _argmax(values):
    """最大値のインデックスと値を返す。空なら (None, None)。"""
    if not values:
        return None, None
    idx = max(range(len(values)), key=lambda i: values[i])
    return idx, values[idx]


def summarize(d):
    """統計を dict で返す（表示・タイトル用）。"""
    t = d['t_s']
    s = {}
    s['n'] = len(t)
    s['dur'] = (t[-1] - t[0]) if len(t) >= 2 else 0.0
    s['rate'] = (s['n'] / s['dur']) if s['dur'] > 0 else 0.0
    fi, fmax = _argmax(d['Fmag_N'])
    mi, mmax = _argmax(d['Mmag_Nm'])
    s['fmax'] = fmax
    s['fmax_t'] = t[fi] if fi is not None else None
    s['mmax'] = mmax
    s['mmax_t'] = t[mi] if mi is not None else None
    s['fmean'] = (sum(d['Fmag_N']) / len(d['Fmag_N'])) if d['Fmag_N'] else 0.0
    return s


def make_figure(d, s, title, contact=None):
    import matplotlib.pyplot as plt

    t = d['t_s']
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(11, 7))

    # --- 上段: 力 [N] ---
    ax1.plot(t, d['fx_N'], color=COLOR_X, lw=1.2, label='Fx')
    ax1.plot(t, d['fy_N'], color=COLOR_Y, lw=1.2, label='Fy')
    ax1.plot(t, d['fz_N'], color=COLOR_Z, lw=1.2, label='Fz')
    ax1.plot(t, d['Fmag_N'], color=COLOR_MAG, lw=2.0, label='|F|')
    if s['fmax_t'] is not None:
        ax1.annotate('max |F| = %.1f N' % s['fmax'],
                     xy=(s['fmax_t'], s['fmax']),
                     xytext=(8, -14), textcoords='offset points',
                     fontsize=9, color=COLOR_MAG)
        ax1.plot([s['fmax_t']], [s['fmax']], 'o', color=COLOR_MAG, ms=5)
    ax1.set_ylabel('Force [N]')
    ax1.grid(True, color=GRID_COLOR, lw=0.6)
    ax1.legend(loc='upper right', ncol=4, framealpha=0.9)
    ax1.set_title(title, fontsize=11)

    # --- 下段: モーメント [N*m] ---
    ax2.plot(t, d['mx_Nm'], color=COLOR_X, lw=1.2, label='Mx')
    ax2.plot(t, d['my_Nm'], color=COLOR_Y, lw=1.2, label='My')
    ax2.plot(t, d['mz_Nm'], color=COLOR_Z, lw=1.2, label='Mz')
    ax2.plot(t, d['Mmag_Nm'], color=COLOR_MAG, lw=2.0, label='|M|')
    ax2.set_ylabel('Moment [N*m]')
    ax2.set_xlabel('Time [s]')
    ax2.grid(True, color=GRID_COLOR, lw=0.6)
    ax2.legend(loc='upper right', ncol=4, framealpha=0.9)

    # --- 加工区間（|F|>=contact）を薄く塗る（任意）---
    if contact is not None:
        for ax in (ax1, ax2):
            _shade_contact(ax, t, d['Fmag_N'], contact)

    fig.tight_layout()
    return fig


def _shade_contact(ax, t, fmag, thr):
    """|F|>=thr の連続区間を軽く塗る。"""
    start = None
    for i in range(len(t)):
        on = fmag[i] >= thr
        if on and start is None:
            start = t[i]
        elif not on and start is not None:
            ax.axvspan(start, t[i], color='#F0C000', alpha=0.12)
            start = None
    if start is not None:
        ax.axvspan(start, t[-1], color='#F0C000', alpha=0.12)


def _median(values):
    v = sorted(values)
    n = len(v)
    if n == 0:
        return 0.0
    return v[n // 2] if n % 2 else 0.5 * (v[n // 2 - 1] + v[n // 2])


def detect_segments(t, fmag, thr, min_dur=0.4, merge_gap=0.3):
    """|F|>=thr の接触区間を (start_i, end_i) のリストで返す。

    merge_gap 秒以下の短い途切れは1区間に連結し、min_dur 秒未満の区間は捨てる
    （砥石の噛み込みで一瞬だけ閾値を割る、といったノイズをまとめる）。
    """
    raw = []
    s = None
    for i in range(len(t)):
        on = fmag[i] >= thr
        if on and s is None:
            s = i
        elif not on and s is not None:
            raw.append([s, i - 1])
            s = None
    if s is not None:
        raw.append([s, len(t) - 1])
    # 近接区間を連結
    merged = []
    for seg in raw:
        if merged and t[seg[0]] - t[merged[-1][1]] <= merge_gap:
            merged[-1][1] = seg[1]
        else:
            merged.append(seg)
    return [(a, b) for a, b in merged if t[b] - t[a] >= min_dur]


def analyze_segments(d, thr):
    """接触区間の統計・空中ベースラインを算出して文字列で返す。"""
    t, F = d['t_s'], d['Fmag_N']
    segs = detect_segments(t, F, thr)
    # 空中（非接触）ベースライン: 閾値未満サンプルの |F| 中央値
    air = [F[i] for i in range(len(t)) if F[i] < thr]
    air_med = _median(air)
    lines = []
    lines.append('--- 接触区間の解析 (閾値 |F| >= %.1f N) ---' % thr)
    lines.append('空中ベースライン |F| 中央値 : %.2f N '
                 '（これが大きいと姿勢による重力オフセットの疑い）' % air_med)
    if not segs:
        lines.append('接触区間は検出されませんでした。')
        return lines, segs
    lines.append('%3s  %8s %8s %6s  %7s %7s  %7s  %8s' %
                 ('#', 't開始', 't終了', '継続s', '平均|F|', '最大|F|', '平均Fy', '平均Mx'))
    tot_contact = 0.0
    for k, (a, b) in enumerate(segs, 1):
        dur = t[b] - t[a]
        tot_contact += dur
        seg = range(a, b + 1)
        fm = [F[i] for i in seg]
        fy = [d['fy_N'][i] for i in seg]
        mx = [d['mx_Nm'][i] for i in seg]
        lines.append('%3d  %8.1f %8.1f %6.1f  %7.2f %7.2f  %7.2f  %8.3f' %
                     (k, t[a], t[b], dur, sum(fm) / len(fm), max(fm),
                      sum(fy) / len(fy), sum(mx) / len(mx)))
    span = (t[-1] - t[0]) or 1.0
    lines.append('接触区間 %d 個 / 合計接触 %.1f s (%.0f%%) / 空中 %.1f s' %
                 (len(segs), tot_contact, 100.0 * tot_contact / span, span - tot_contact))
    return lines, segs


def detect_phase_split(t, F, lo_frac=0.35, hi_frac=0.65, win_s=1.0):
    """右(HaR)と左(HaL)の境目の時刻を推定して返す。

    kenma は HaR→HaL の順。境目は工程反転で力が谷になるので、中央 lo..hi の範囲で
    平滑化した |F| が最小になる時刻を境界とみなす。データが短ければ None。
    """
    n = len(t)
    if n < 10:
        return None
    dur = (t[-1] - t[0]) or 1.0
    rate = n / dur
    w = max(1, int(rate * win_s))
    ps = [0.0] * (n + 1)
    for i in range(n):
        ps[i + 1] = ps[i] + F[i]
    lo, hi = int(n * lo_frac), int(n * hi_frac)
    best_i, best_v = lo, float('inf')
    for i in range(lo, hi):
        a, b = max(0, i - w), min(n, i + w)
        v = (ps[b] - ps[a]) / (b - a)
        if v < best_v:
            best_v, best_i = v, i
    return t[best_i]


def side_summary(d, split):
    """右(HaR: 開始～split)と左(HaL: split～終了)それぞれの |F| 統計を返す。

    差引後(=各サイドが自分のゼロ基準)の d に対して使うと、左右の接触力を直接比較できる。
    戻り値: 文字列リスト
    """
    t, F = d['t_s'], d['Fmag_N']

    def stat(a, b):
        # F<=0 は自動ゼロで非活性(=非接触/反転区間)として0化した点なので集計から除く
        v = sorted(F[i] for i in range(len(t)) if a <= t[i] < b and F[i] > 0.0)
        if not v:
            return None
        n = len(v)
        return (n, sum(v) / n, v[n // 2], v[int(n * 0.9)], v[-1])

    right = stat(t[0], split)
    left = stat(split, t[-1])
    out = ['--- 左右サマリ（境界 t=%.1fs で分割）---' % split,
           '%-10s %6s %7s %7s %7s %7s' % ('サイド', 'n', '平均', '中央', 'p90', '最大')]
    for name, s in (('右 HaR', right), ('左 HaL', left)):
        if s:
            out.append('%-10s %6d %7.2f %7.2f %7.2f %7.2f' %
                       (name, s[0], s[1], s[2], s[3], s[4]))
        else:
            out.append('%-10s (データ無し)' % name)
    if right and left and right[2] > 0:
        out.append('左/右 の中央値比 = %.2f 倍（1.0で左右均等）' % (left[2] / right[2]))
    return out


def auto_zero(d, active_thr=0.5, split=None, ref_frac=0.15):
    """空運転CSVなしで、各サイド(HaR/HaL)の重力オフセットを自分自身から推定して差し引く。

    Webツールの「工程ごと自動ゼロ」に相当。手順:
      1) |F|>=active_thr の活性区間（＝研磨で当たっている区間）を検出。
      2) 左右(HaR/HaL)に振り分け、各サイドの活性サンプルのうち |F| が小さい側 ref_frac を
         「そのサイドの非接触＝重力レベル」とみなし、各成分の平均をオフセットとする。
      3) 活性サンプルはそのオフセットを引く。非活性(反転/退避/開始前)は0にする。
    注意: 姿勢変化のリップルは残るので、精密には空運転差引(--baseline)が上。
          ただし砥石を動かさずボタンだけで左右ゼロ比較したいときに有効。
    戻り値: (d, split)
    """
    t, F = d['t_s'], d['Fmag_N']
    n = len(t)
    comps = ('fx_N', 'fy_N', 'fz_N', 'mx_Nm', 'my_Nm', 'mz_Nm')
    if split is None:
        split = detect_phase_split(t, list(F))
    segs = detect_segments(t, F, active_thr)
    if not segs:
        return d, split
    active = [False] * n
    sideR, sideL = [], []
    for a, b in segs:
        mid = 0.5 * (t[a] + t[b])
        is_r = (split is None) or (mid < split)
        for i in range(a, b + 1):
            active[i] = True
            (sideR if is_r else sideL).append(i)

    def gravity(idxs):
        if not idxs:
            return None
        order = sorted(idxs, key=lambda i: F[i])
        ref = order[:max(1, int(len(order) * ref_frac))]
        return {k: sum(d[k][i] for i in ref) / len(ref) for k in comps}

    gR, gL = gravity(sideR), gravity(sideL)
    for i in range(n):
        if not active[i]:
            for k in comps:
                d[k][i] = 0.0
            d['Fmag_N'][i] = 0.0
            d['Mmag_Nm'][i] = 0.0
            continue
        g = gR if ((split is None) or t[i] < split) else gL
        if g:
            for k in comps:
                d[k][i] -= g[k]
        fx, fy, fz = d['fx_N'][i], d['fy_N'][i], d['fz_N'][i]
        mx, my, mz = d['mx_Nm'][i], d['my_Nm'][i], d['mz_Nm'][i]
        d['Fmag_N'][i] = (fx * fx + fy * fy + fz * fz) ** 0.5
        d['Mmag_Nm'][i] = (mx * mx + my * my + mz * mz) ** 0.5
    return d, split


def main():
    ap = argparse.ArgumentParser(
        description='force_log_*.csv から力/モーメントの時系列グラフを作る')
    ap.add_argument('csv', nargs='?',
                    help='CSVファイル（省略時は最新の force_log_*.csv を自動選択）')
    ap.add_argument('--no-show', action='store_true',
                    help='画面表示せず PNG 保存のみ')
    ap.add_argument('--contact', type=float, default=None,
                    help='この[N]以上を加工区間として薄く塗る 例 3.0')
    ap.add_argument('--seg-thr', type=float, default=1.0,
                    help='接触区間の判定しきい値[N]（既定 1.0）。区間ごとの統計を端末に表示')
    ap.add_argument('--no-seg', action='store_true',
                    help='接触区間の解析（区間ごと統計）を表示しない')
    ap.add_argument('--baseline', metavar='AIR_CSV',
                    help='空運転CSVを時刻で差し引き、重力/姿勢オフセットを除去して表示')
    ap.add_argument('--baseline-shift', type=float, default=0.0,
                    help='空運転の時刻ズレ補正[s]（手動。--baseline-align 併用時は初期値として無視）')
    ap.add_argument('--baseline-align', action='store_true',
                    help='空運転との時刻ズレを波形(歯)の相互相関で自動整列してから差し引く（推奨）')
    ap.add_argument('--sides', action='store_true',
                    help='右(HaR)/左(HaL)を自動で分けて、各サイドの|F|統計を表示（左右の接触力比較）')
    ap.add_argument('--split', type=float, default=None,
                    help='左右の境界時刻[s]を手動指定（省略時は自動検出）')
    ap.add_argument('--auto-zero', action='store_true',
                    help='空運転CSVなしで各サイドの重力を自分自身から推定して差し引く（Webツールの工程ごと自動ゼロ相当）')
    args = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    path = args.csv or find_latest_csv(here) or find_latest_csv(os.getcwd())
    if not path or not os.path.isfile(path):
        print('CSVが見つかりません。force_log_*.csv を作ってから実行してください。')
        print('（記録は  python force_moment_overlay.py --no-robodk --log ）')
        return 2

    d = load_log(path)
    if not d['t_s']:
        print('データ行がありません:', path)
        return 2

    # 左右の境界は「差引前の生波形」で検出（HaR≈高 / HaL≈低 のコントラストが強く確実）
    split_t = None
    if args.sides:
        split_t = args.split if args.split is not None else \
            detect_phase_split(d['t_s'], list(d['Fmag_N']))

    baseline_note = ''
    if args.baseline:
        if not os.path.isfile(args.baseline):
            print('空運転CSVが見つかりません:', args.baseline)
            return 2
        base = load_log(args.baseline)
        if not base['t_s']:
            print('空運転CSVにデータがありません:', args.baseline)
            return 2
        shift = args.baseline_shift
        if args.baseline_align:
            shift, corr = estimate_baseline_shift(d, base)
            print('自動整列: 時刻シフト %.2f s（波形相関 %.3f）' % (shift, corr))
            if corr < 0.5:
                print('  ※ 相関が低いです。開始タイミングが大きく違う/別プログラムの可能性。'
                      '--baseline-shift で手動調整も可。')
        apply_baseline(d, base, shift=shift)
        baseline_note = ' [空運転差引済み]'
        print('空運転を差し引きました:', os.path.basename(args.baseline),
              '(shift=%.2fs)' % shift)
    elif args.auto_zero:
        _, split_t = auto_zero(d, split=split_t)
        baseline_note = ' [自動ゼロ済み(空運転なし)]'
        print('自動ゼロ: 各サイドの重力を自分自身から推定して差し引きました。')
        print('  ⚠ 砥石から離れず押しっぱなしの側（例: HaR）は、姿勢リップルを接触力と')
        print('     誤認して過大に出ます。正確な左右比較には --baseline（空運転）を使ってください。')

    s = summarize(d)

    # 統計を端末に表示
    print('ファイル :', path)
    print('サンプル : %d 点 / %.1f 秒 (約 %.1f Hz)' % (s['n'], s['dur'], s['rate']))
    print('最大 |F| : %.2f N  (t=%.1fs)' % (s['fmax'], s['fmax_t']))
    print('最大 |M| : %.3f N*m (t=%.1fs)' % (s['mmax'], s['mmax_t']))
    print('平均 |F| : %.2f N' % s['fmean'])

    # 接触区間ごとの統計（研磨で「どこがどれだけ噛んだか」を数値で把握）
    if not args.no_seg:
        seg_lines, _ = analyze_segments(d, args.seg_thr)
        print()
        for ln in seg_lines:
            print(ln)

    # 左右サマリ（--sides）。差引後の d を使うと左右の真の接触力を直接比較できる。
    if args.sides:
        print()
        if split_t is None:
            print('左右の境界を自動検出できませんでした（--split 秒 で手動指定してください）。')
        else:
            for ln in side_summary(d, split_t):
                print(ln)
            if not args.baseline and not args.auto_zero:
                print('※ 重力未除去です。--baseline 空運転.csv（正確）か --auto-zero（簡易）を併用してください。')

    try:
        import matplotlib
        if args.no_show:
            matplotlib.use('Agg')   # 画面なしでも保存できるように
        import matplotlib.pyplot as plt
    except ImportError:
        print('matplotlib がありません。 pip install matplotlib を実行してください。')
        return 3

    # --contact 指定があればその値で、無ければ接触判定しきい値で加工区間を塗る
    shade_thr = args.contact if args.contact is not None else (
        None if args.no_seg else args.seg_thr)
    title = '%s%s   |   max|F|=%.1fN  max|M|=%.2fN*m  (%.0fs)' % (
        os.path.basename(path), baseline_note, s['fmax'], s['mmax'], s['dur'])
    fig = make_figure(d, s, title, contact=shade_thr)

    suffix = '_baselined' if args.baseline else ''
    png = os.path.splitext(path)[0] + suffix + '.png'
    fig.savefig(png, dpi=120)
    print('グラフを保存 :', png)

    if not args.no_show:
        plt.show()
    return 0


if __name__ == '__main__':
    sys.exit(main())
