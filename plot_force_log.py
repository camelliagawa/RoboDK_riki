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
import json
import argparse

# =====================================================================
#  グラフのデザイン設定（ここを編集すれば見た目を自由に変更できます）
#  ・コードを触りたくない場合は、同じフォルダに plot_config.json を置くと
#    同じキーで上書きできます（plot_config.example.json を参照）。
#  ・一部は実行時オプションでも変更できます（--title --xlim --ylim-force など）。
# =====================================================================
STYLE = {
    # --- 色（'#RRGGBB' か 色名）---
    'color_x': '#D55E00',      # X 成分（Fx / Mx）
    'color_y': '#009E73',      # Y 成分（Fy / My）
    'color_z': '#0072B2',      # Z 成分（Fz / Mz）
    'color_mag': '#222222',    # 合成値（|F| / |M|）の強調線
    'grid_color': '#CCCCCC',   # グリッド線
    'contact_color': '#F0C000',  # 加工区間の塗り
    'contact_alpha': 0.12,     # 加工区間の塗りの濃さ 0-1

    # --- 線の太さ ---
    'lw_component': 1.2,       # 成分線
    'lw_mag': 2.0,             # 合成値線
    'grid_lw': 0.6,            # グリッド線

    # --- 図のサイズ・解像度 ---
    'figsize_w': 11.0,
    'figsize_h': 7.0,
    'dpi': 120,                # PNG保存の解像度
    'font_family': None,       # 日本語ラベルにするなら 'Meiryo' 等（Windows）。Noneで既定

    # --- 文字（ラベル・凡例・タイトル）---
    'force_ylabel': 'Force [N]',
    'moment_ylabel': 'Moment [N*m]',
    'xlabel': 'Time [s]',
    'label_fx': 'Fx', 'label_fy': 'Fy', 'label_fz': 'Fz', 'label_fmag': '|F|',
    'label_mx': 'Mx', 'label_my': 'My', 'label_mz': 'Mz', 'label_mmag': '|M|',
    'title': None,             # None=自動生成。文字列を入れると固定タイトル
    'title_fontsize': 11,
    'legend_loc': 'upper right',
    'legend_fontsize': None,   # None=既定

    # --- 表示範囲（None=自動）---
    'xlim_min': None, 'xlim_max': None,
    'force_ylim_min': None, 'force_ylim_max': None,
    'moment_ylim_min': None, 'moment_ylim_max': None,

    # --- 最大点の注釈 ---
    'annotate_max': True,
    'annotate_fontsize': 9,

    # --- 系列の表示ON/OFF（不要な線を消せる）---
    'show_fx': True, 'show_fy': True, 'show_fz': True, 'show_fmag': True,
    'show_mx': True, 'show_my': True, 'show_mz': True, 'show_mmag': True,

    # --- 各要素の表示ON/OFF ---
    'show_title': True,     # タイトル（キャプション）
    'show_legend': True,    # 凡例
    'show_xaxis': True,     # 横軸（目盛り＋ラベル）
    'show_yaxis': True,     # 縦軸（目盛り＋ラベル）
    'show_grid': True,      # グリッド線
    'show_shade': True,     # 加工区間の黄色塗り
    'force_xaxis': True,    # 力(上段)グラフにも Time[s] 目盛りを表示

    # --- 線種（'-'=実線, '--'=破線, ':'=点線, '-.'=一点鎖線）系列ごとに指定可 ---
    'ls_fx': '-', 'ls_fy': '-', 'ls_fz': '-', 'ls_fmag': '-',
    'ls_mx': '-', 'ls_my': '-', 'ls_mz': '-', 'ls_mmag': '-',
}


def load_style(here):
    """STYLE を複製し、plot_config.json があればそのキーで上書きして返す。"""
    style = dict(STYLE)
    for base_dir in (here, os.getcwd()):
        p = os.path.join(base_dir, 'plot_config.json')
        if os.path.isfile(p):
            try:
                with open(p, 'r', encoding='utf-8') as f:
                    over = json.load(f)
                for k, v in over.items():
                    if k.startswith('_'):
                        continue   # "_"始まりはコメント扱い
                    if k in style:
                        style[k] = v
                    else:
                        print('（plot_config.json: 未知のキー「%s」は無視）' % k)
                print('デザイン設定を読み込みました:', p)
            except Exception as e:
                print('plot_config.json の読み込みに失敗（無視して既定を使用）:', e)
            break
    return style


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


def apply_baseline_persides(d, base):
    """研磨(d)と空運転(base)を「右姿勢/左姿勢のブロック」に分け、サイドごとに整列して差し引く。

    重力は姿勢だけで決まり、空運転には右姿勢・左姿勢の両ブロックが入っている。各ブロックを
    姿勢（=重力|F|レベル: 高い方=HaR右, 低い方=HaL左）で対応づけ、ブロック内の歯パターンで
    個別整列して引く。**研磨の順番(右先/左先)を変えても、同じ空運転1本で差し引ける。**
    境界検出に失敗したら全体整列にフォールバック。戻り値: 診断文字列リスト。
    """
    comps = ('fx_N', 'fy_N', 'fz_N', 'mx_Nm', 'my_Nm', 'mz_Nm', 'Fmag_N', 'Mmag_Nm')
    tg, ta = d['t_s'], base['t_s']
    sg = detect_phase_split(tg, list(d['Fmag_N']))
    sa = detect_phase_split(ta, list(base['Fmag_N']))
    if sg is None or sa is None:
        s, c = estimate_baseline_shift(d, base)
        apply_baseline(d, base, shift=s)
        return ['サイド別整列に失敗→全体整列で差引 (shift=%.2fs, corr=%.3f)' % (s, c)]

    def idx_at(t, val):
        for i in range(len(t)):
            if t[i] >= val:
                return i
        return len(t)

    gi, ai = idx_at(tg, sg), idx_at(ta, sa)
    gblocks = {'1': (0, gi), '2': (gi, len(tg))}
    ablocks = {'1': (0, ai), '2': (ai, len(ta))}

    def med(d_, a, b):
        v = sorted(d_['Fmag_N'][a:b])
        return v[len(v) // 2] if v else 0.0

    # 重力|F|レベルが高いブロック=右(HaR), 低い方=左(HaL) と姿勢で判定
    def label(d_, blocks):
        m1, m2 = med(d_, *blocks['1']), med(d_, *blocks['2'])
        return {'R': blocks['1'], 'L': blocks['2']} if m1 >= m2 \
            else {'R': blocks['2'], 'L': blocks['1']}

    gl, al = label(d, gblocks), label(base, ablocks)
    out = ['サイド別に整列して差引（順番非依存）:']
    for side, jp in (('R', '右 HaR'), ('L', '左 HaL')):
        ga, gb = gl[side]
        aa, ab = al[side]
        # ブロックを時刻0起点に揃えてから整列（絶対時刻が離れていてもマッチ可能に）
        g0, a0 = tg[ga], ta[aa]
        gsub = {k: (d[k][ga:gb] if k != 't_s' else [x - g0 for x in tg[ga:gb]])
                for k in d}
        asub = {k: (base[k][aa:ab] if k != 't_s' else [x - a0 for x in ta[aa:ab]])
                for k in base}
        sh, c = estimate_baseline_shift(gsub, asub)
        apply_baseline(gsub, asub, shift=sh)
        for k in comps:
            d[k][ga:gb] = gsub[k]
        out.append('  %s: shift %.2fs  corr %.3f' % (jp, sh, c))
    return out


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


def make_figure(d, s, title, contact=None, style=None):
    import matplotlib.pyplot as plt
    S = style or STYLE

    t = d['t_s']
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True,
                                   figsize=(S['figsize_w'], S['figsize_h']))

    lwc, lwm = S['lw_component'], S['lw_mag']
    leg = dict(loc=S['legend_loc'], ncol=4, framealpha=0.9)
    if S['legend_fontsize']:
        leg['fontsize'] = S['legend_fontsize']

    lines = {}   # 'fx','fy',... -> Line2D（操作パネルから触れるよう常に生成し可視だけ切替）

    # --- 上段: 力 [N] ---
    fseries = (('fx', 'fx_N', 'color_x', 'label_fx', lwc),
               ('fy', 'fy_N', 'color_y', 'label_fy', lwc),
               ('fz', 'fz_N', 'color_z', 'label_fz', lwc),
               ('fmag', 'Fmag_N', 'color_mag', 'label_fmag', lwm))
    for key, col, ckey, lkey, lw in fseries:
        (ln,) = ax1.plot(t, d[col], color=S[ckey], lw=lw,
                         ls=S.get('ls_' + key, '-'), label=S[lkey])
        ln.set_visible(S['show_' + key])
        lines[key] = ln
    # 最大点の注釈・マーカー（後からON/OFFできるよう常に生成し可視だけ制御）
    ax1._maxann = ax1._maxdot = None
    if s['fmax_t'] is not None:
        ax1._maxann = ax1.annotate('max |F| = %.1f N' % s['fmax'],
                                   xy=(s['fmax_t'], s['fmax']),
                                   xytext=(8, -14), textcoords='offset points',
                                   fontsize=S['annotate_fontsize'], color=S['color_mag'])
        (ax1._maxdot,) = ax1.plot([s['fmax_t']], [s['fmax']], 'o',
                                  color=S['color_mag'], ms=5)
        vis = S['annotate_max'] and S['show_fmag']
        ax1._maxann.set_visible(vis)
        ax1._maxdot.set_visible(vis)
    ax1.set_ylabel(S['force_ylabel'])
    ax1.set_title(title, fontsize=S['title_fontsize'])
    if S['force_ylim_min'] is not None or S['force_ylim_max'] is not None:
        ax1.set_ylim(S['force_ylim_min'], S['force_ylim_max'])

    # --- 下段: モーメント [N*m] ---
    mseries = (('mx', 'mx_Nm', 'color_x', 'label_mx', lwc),
               ('my', 'my_Nm', 'color_y', 'label_my', lwc),
               ('mz', 'mz_Nm', 'color_z', 'label_mz', lwc),
               ('mmag', 'Mmag_Nm', 'color_mag', 'label_mmag', lwm))
    for key, col, ckey, lkey, lw in mseries:
        (ln,) = ax2.plot(t, d[col], color=S[ckey], lw=lw,
                         ls=S.get('ls_' + key, '-'), label=S[lkey])
        ln.set_visible(S['show_' + key])
        lines[key] = ln
    ax2.set_ylabel(S['moment_ylabel'])
    ax2.set_xlabel(S['xlabel'])
    if S['moment_ylim_min'] is not None or S['moment_ylim_max'] is not None:
        ax2.set_ylim(S['moment_ylim_min'], S['moment_ylim_max'])

    # 力(上段)にも Time[s] 軸を出す（sharexで隠れる下端目盛りを復活）
    if S['force_xaxis']:
        ax1.tick_params(labelbottom=True)
        ax1.set_xlabel(S['xlabel'])

    _refresh_legend(ax1, lines, ('fx', 'fy', 'fz', 'fmag'), leg)
    _refresh_legend(ax2, lines, ('mx', 'my', 'mz', 'mmag'), leg)

    # --- 表示する時間範囲（xlim）---
    if S['xlim_min'] is not None or S['xlim_max'] is not None:
        ax1.set_xlim(S['xlim_min'], S['xlim_max'])

    # --- 加工区間（|F|>=contact）を薄く塗る（任意）---
    for ax in (ax1, ax2):
        ax._shade_patches = []
        if contact is not None:
            ax._shade_patches = _shade_contact(
                ax, t, d['Fmag_N'], contact,
                color=S['contact_color'], alpha=S['contact_alpha'])
            for p in ax._shade_patches:
                p.set_visible(S['show_shade'])

    # --- 各要素の表示ON/OFF（グリッド/タイトル/凡例/軸）---
    for ax in (ax1, ax2):
        if S['show_grid']:
            ax.grid(True, color=S['grid_color'], lw=S['grid_lw'])
        else:
            ax.grid(False)
        ax.get_yaxis().set_visible(S['show_yaxis'])
    ax1.title.set_visible(S['show_title'])
    # 横軸: 下段は show_xaxis、上段は show_xaxis かつ force_xaxis
    ax2.get_xaxis().set_visible(S['show_xaxis'])
    ax1.get_xaxis().set_visible(S['show_xaxis'] and S['force_xaxis'])
    if not S['show_legend']:
        for ax in (ax1, ax2):
            if ax.get_legend():
                ax.get_legend().set_visible(False)

    fig.tight_layout()
    # パネルから触れるようハンドルを図に保持
    fig._fml = {'ax1': ax1, 'ax2': ax2, 'lines': lines, 'leg': leg,
                'style': S, 'title_text': title}
    return fig, ax1, ax2, lines, leg


def save_axes_region(fig, axes_list, out_path, dpi):
    """図の中の指定axes（と付随ラベル/凡例）だけを切り出してPNG保存する。
    現在の見た目（色・表示ON/OFF・範囲など操作パネルでの変更）を反映する。"""
    from matplotlib.transforms import Bbox
    fig.canvas.draw()
    r = fig.canvas.get_renderer()
    bb = Bbox.union([ax.get_tightbbox(r) for ax in axes_list])
    ext = bb.transformed(fig.dpi_scale_trans.inverted())
    ext = ext.padded(0.12)   # 各辺に約0.12インチだけ余白（隣のグラフを巻き込まない）
    fig.savefig(out_path, bbox_inches=ext, dpi=dpi)
    return out_path


def _refresh_legend(ax, lines, keys, leg_kwargs):
    """可視な線だけで凡例を作り直す（系列ON/OFFに追従させるため）。"""
    handles = [lines[k] for k in keys if lines[k].get_visible()]
    if handles:
        ax.legend(handles, [h.get_label() for h in handles], **leg_kwargs)
    elif ax.get_legend():
        ax.get_legend().remove()


# X/Y/Z/合成 の色を系列に割り当てる配色テーマ（操作パネルの色ボタン用）
COLOR_THEMES = {
    'Default': ('#D55E00', '#009E73', '#0072B2', '#222222'),
    'Vivid':   ('#E6194B', '#3CB44B', '#4363D8', '#000000'),
    'Warm':    ('#D7263D', '#F46036', '#2E294E', '#1B1B1E'),
    'Mono':    ('#333333', '#777777', '#AAAAAA', '#000000'),
}


def add_control_panel(fig, ax1, ax2, lines, leg, save_base=None, save_dpi=120):
    """グラフ右側に操作パネルを常時表示する。
    系列ON/OFF・要素ON/OFF(タイトル/凡例/軸/グリッド/塗り/最大点)・範囲入力・配色・線種・
    力/モーメントの個別保存・タイトル変更。
    """
    from matplotlib.widgets import CheckButtons, TextBox, Button

    S = fig._fml['style'] if hasattr(fig, '_fml') else STYLE
    fig.set_size_inches(max(fig.get_figwidth(), 14), max(fig.get_figheight(), 8))
    fig.subplots_adjust(left=0.07, right=0.63, top=0.93, bottom=0.09, hspace=0.20)
    keep = []
    fkeys = ('fx', 'fy', 'fz', 'fmag')
    mkeys = ('mx', 'my', 'mz', 'mmag')

    def redraw_legends():
        _refresh_legend(ax1, lines, fkeys, leg)
        _refresh_legend(ax2, lines, mkeys, leg)

    def txt(rect, s, size=9):
        a = fig.add_axes(rect); a.axis('off'); a.text(0, 0.2, s, fontsize=size)

    # --- 系列ON/OFF（左カラム）---
    order = [('fx', 'Fx'), ('fy', 'Fy'), ('fz', 'Fz'), ('fmag', '|F|'),
             ('mx', 'Mx'), ('my', 'My'), ('mz', 'Mz'), ('mmag', '|M|')]
    slabels = [lb for _, lb in order]
    ax_s = fig.add_axes([0.655, 0.58, 0.15, 0.37])
    ax_s.set_title('Series', fontsize=9)
    scolors = [lines[k].get_color() for k, _ in order]
    try:
        chk_s = CheckButtons(ax_s, slabels, [lines[k].get_visible() for k, _ in order],
                             frame_props={'s': 90, 'facecolor': 'white',
                                          'edgecolor': '#999999', 'linewidth': 1.1},
                             check_props={'s': 90, 'facecolor': scolors},
                             label_props={'color': scolors})
    except TypeError:
        chk_s = CheckButtons(ax_s, slabels, [lines[k].get_visible() for k, _ in order])

    def on_series(label):
        k = order[slabels.index(label)][0]
        lines[k].set_visible(not lines[k].get_visible())
        redraw_legends(); fig.canvas.draw_idle()
    chk_s.on_clicked(on_series)
    keep.append(chk_s)

    # --- 要素ON/OFF（右カラム）---
    grid_on = [S['show_grid']]
    has_max = getattr(ax1, '_maxann', None) is not None
    elabels = ['Title', 'Legend', 'X-axis', 'Y-axis', 'Grid', 'Shade', 'Max pt']
    estate = [ax1.title.get_visible(),
              bool(ax1.get_legend() and ax1.get_legend().get_visible()),
              ax2.get_xaxis().get_visible(), ax1.get_yaxis().get_visible(),
              grid_on[0], any(p.get_visible() for p in ax1._shade_patches),
              bool(has_max and ax1._maxann.get_visible())]
    ax_e = fig.add_axes([0.82, 0.58, 0.17, 0.37])
    ax_e.set_title('Elements', fontsize=9)
    try:
        chk_e = CheckButtons(ax_e, elabels, estate,
                             frame_props={'s': 90, 'facecolor': 'white',
                                          'edgecolor': '#999999', 'linewidth': 1.1},
                             check_props={'s': 90, 'facecolor': '#444444'})
    except TypeError:
        chk_e = CheckButtons(ax_e, elabels, estate)

    def on_element(label):
        if label == 'Title':
            ax1.title.set_visible(not ax1.title.get_visible())
        elif label == 'Legend':
            for ax in (ax1, ax2):
                lg = ax.get_legend()
                if lg:
                    lg.set_visible(not lg.get_visible())
        elif label == 'X-axis':
            v = not ax2.get_xaxis().get_visible()
            ax2.get_xaxis().set_visible(v)
            ax1.get_xaxis().set_visible(v and S['force_xaxis'])
        elif label == 'Y-axis':
            v = not ax1.get_yaxis().get_visible()
            for ax in (ax1, ax2):
                ax.get_yaxis().set_visible(v)
        elif label == 'Grid':
            grid_on[0] = not grid_on[0]
            for ax in (ax1, ax2):
                if grid_on[0]:
                    ax.grid(True, color=S['grid_color'], lw=S['grid_lw'])
                else:
                    ax.grid(False)
        elif label == 'Shade':
            for ax in (ax1, ax2):
                for p in ax._shade_patches:
                    p.set_visible(not p.get_visible())
        elif label == 'Max pt':
            if getattr(ax1, '_maxann', None) is not None:
                v = not ax1._maxann.get_visible()
                ax1._maxann.set_visible(v)
                ax1._maxdot.set_visible(v)
        fig.canvas.draw_idle()
    chk_e.on_clicked(on_element)
    keep.append(chk_e)

    # --- 範囲入力（"min max"、空でオート）---
    txt([0.655, 0.545, 0.34, 0.03], 'Range  "min max"  (Enter; empty=Auto)', 8)

    def make_range_box(y, label, ax_target, axis):
        tb = TextBox(fig.add_axes([0.735, y, 0.20, 0.04]), label, initial='')

        def submit(text):
            text = text.strip()
            try:
                if text == '':
                    ax_target.autoscale(axis=axis)
                else:
                    a, b = text.replace(',', ' ').split()
                    (ax_target.set_xlim if axis == 'x' else ax_target.set_ylim)(float(a), float(b))
                fig.canvas.draw_idle()
            except Exception:
                pass
        tb.on_submit(submit); keep.append(tb)
    make_range_box(0.495, 'X [s]', ax1, 'x')
    make_range_box(0.445, 'F [N]', ax1, 'y')
    make_range_box(0.395, 'M[Nm]', ax2, 'y')
    btn_auto = Button(fig.add_axes([0.735, 0.340, 0.20, 0.045]), 'Auto range')

    def on_auto(_):
        for ax in (ax1, ax2):
            ax.relim(); ax.autoscale()
        fig.canvas.draw_idle()
    btn_auto.on_clicked(on_auto); keep.append(btn_auto)

    # 1行N個のボタンを x=0.655..0.99 に等間隔で並べる小ヘルパ
    def button_row(y, items, make_cb, hover=None):
        n = len(items); gap = 0.008; w = (0.335 - (n - 1) * gap) / n
        for i, (label, val) in enumerate(items):
            b = Button(fig.add_axes([0.655 + i * (w + gap), y, w, 0.042]),
                       label, hovercolor=hover or '0.85')
            b.on_clicked(make_cb(val)); keep.append(b)

    # --- 配色テーマ（1行4）---
    txt([0.655, 0.305, 0.2, 0.025], 'Colors', 9)

    def theme_cb(nm):
        def f(_e):
            cx, cy, cz, cm = COLOR_THEMES[nm]
            for k, c in (('fx', cx), ('fy', cy), ('fz', cz), ('fmag', cm),
                         ('mx', cx), ('my', cy), ('mz', cz), ('mmag', cm)):
                lines[k].set_color(c)
            redraw_legends(); fig.canvas.draw_idle()
        return f
    button_row(0.258, [(n, n) for n in COLOR_THEMES], theme_cb)

    # --- 力/モーメントの個別保存（1行3）---
    txt([0.655, 0.212, 0.3, 0.025], 'Save image', 9)

    def save_cb(which):
        def f(_e):
            if not save_base:
                return
            axes = {'f': [ax1], 'm': [ax2], 'b': [ax1, ax2]}[which]
            suffix = {'f': 'force', 'm': 'moment', 'b': 'both'}[which]
            out = save_axes_region(fig, axes, save_base + '_' + suffix + '.png', save_dpi)
            print('保存:', out)
        return f
    button_row(0.165, [('Force', 'f'), ('Moment', 'm'), ('Both', 'b')], save_cb,
               hover='#bfe3bf')

    # --- 線種（全線・1行4）---
    txt([0.655, 0.119, 0.2, 0.025], 'Line style (all lines)', 9)

    def ls_cb(ch):
        def f(_e):
            for k in lines:
                lines[k].set_linestyle(ch)
            redraw_legends(); fig.canvas.draw_idle()
        return f
    button_row(0.072, [('Solid', '-'), ('Dashed', '--'),
                       ('Dotted', ':'), ('DashDot', '-.')], ls_cb, hover='#c8e0ff')

    # --- タイトル文字の変更 ---
    tb_title = TextBox(fig.add_axes([0.720, 0.020, 0.270, 0.038]), 'Title',
                       initial=fig._fml.get('title_text', '') if hasattr(fig, '_fml') else '')

    def on_title(text):
        ax1.set_title(text, fontsize=S['title_fontsize'])
        ax1.title.set_visible(True)
        fig.canvas.draw_idle()
    tb_title.on_submit(on_title); keep.append(tb_title)

    fig._panel_widgets = keep
    return keep


def _shade_contact(ax, t, fmag, thr, color='#F0C000', alpha=0.12):
    """|F|>=thr の連続区間を軽く塗る。塗ったパッチのリストを返す（表示ON/OFF用）。"""
    patches = []
    start = None
    for i in range(len(t)):
        on = fmag[i] >= thr
        if on and start is None:
            start = t[i]
        elif not on and start is not None:
            patches.append(ax.axvspan(start, t[i], color=color, alpha=alpha))
            start = None
    if start is not None:
        patches.append(ax.axvspan(start, t[-1], color=color, alpha=alpha))
    return patches


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


def side_summary(d, split, right_first=True):
    """境界 split で2分割し、右(HaR)/左(HaL)それぞれの |F| 統計を返す。

    差引後(=各サイドが自分のゼロ基準)の d に対して使うと、左右の接触力を直接比較できる。
    right_first=False のとき（研磨の順番を左先に変えた場合）は前後のラベルを入れ替える。
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

    first = stat(t[0], split)
    second = stat(split, t[-1])
    right, left = (first, second) if right_first else (second, first)
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
    ap.add_argument('--baseline-persides', action='store_true',
                    help='空運転を右/左の姿勢ブロックに分けてサイドごとに整列して差引（研磨の順番を変えても同じ空運転1本でOK）')
    ap.add_argument('--sides', action='store_true',
                    help='右(HaR)/左(HaL)を自動で分けて、各サイドの|F|統計を表示（左右の接触力比較）')
    ap.add_argument('--split', type=float, default=None,
                    help='左右の境界時刻[s]を手動指定（省略時は自動検出）')
    ap.add_argument('--auto-zero', action='store_true',
                    help='空運転CSVなしで各サイドの重力を自分自身から推定して差し引く（Webツールの工程ごと自動ゼロ相当）')
    ap.add_argument('--auto-baseline', action='store_true',
                    help='フォルダの air.csv(またはair.csv.csv)を自動で見つけてサイド別差引。無ければauto-zero。plot_sides.bat用')
    # --- 見た目（デザイン）の実行時オプション。恒久的に変えるなら STYLE か plot_config.json ---
    ap.add_argument('--title', help='グラフのタイトル文字列（既定は自動生成）')
    ap.add_argument('--xlim', nargs=2, type=float, metavar=('MIN', 'MAX'),
                    help='時間軸の表示範囲[s] 例: --xlim 0 150')
    ap.add_argument('--ylim-force', nargs=2, type=float, metavar=('MIN', 'MAX'),
                    help='力[N]の縦軸範囲 例: --ylim-force -2 10')
    ap.add_argument('--ylim-moment', nargs=2, type=float, metavar=('MIN', 'MAX'),
                    help='モーメント[N*m]の縦軸範囲 例: --ylim-moment -1 1')
    ap.add_argument('--figsize', nargs=2, type=float, metavar=('W', 'H'),
                    help='図のサイズ(インチ) 例: --figsize 12 7')
    ap.add_argument('--dpi', type=float, help='PNG保存の解像度 例: --dpi 150')
    ap.add_argument('--panel', action='store_true',
                    help='グラフ画面に操作パネル(系列ON/OFF・範囲入力・配色ボタン)を常時表示')
    ap.add_argument('--save-split', action='store_true',
                    help='力とモーメントを別々のPNG(<名前>_force.png / _moment.png)にも保存')
    args = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    path = args.csv or find_latest_csv(here) or find_latest_csv(os.getcwd())
    if not path or not os.path.isfile(path):
        print('CSVが見つかりません。force_log_*.csv を作ってから実行してください。')
        print('（記録は  python force_moment_overlay.py --no-robodk --log ）')
        return 2

    # --auto-baseline: air.csv / air.csv.csv を探して自動設定（拡張子二重も許容）
    if args.auto_baseline and not args.baseline:
        found = None
        for base_dir in (here, os.getcwd()):
            for cand in ('air.csv', 'air.csv.csv'):
                p2 = os.path.join(base_dir, cand)
                if os.path.isfile(p2):
                    found = p2
                    break
            if found:
                break
        if found:
            args.baseline = found
            args.baseline_persides = True
            print('空運転を自動検出:', found)
        else:
            args.auto_zero = True
            print('air.csv が見つからないので --auto-zero にフォールバックします（精度は落ちます）。')
            print('  正確に見たい場合: 空運転CSVをこのフォルダに air.csv という名前で置いてください。')

    d = load_log(path)
    if not d['t_s']:
        print('データ行がありません:', path)
        return 2

    # 左右の境界は「差引前の生波形」で検出（HaR≈高 / HaL≈低 のコントラストが強く確実）
    split_t = None
    right_first = True
    if args.sides:
        split_t = args.split if args.split is not None else \
            detect_phase_split(d['t_s'], list(d['Fmag_N']))
        if split_t is not None:
            # 生の重力レベルが高いブロック=右(HaR)。順番を左先に変えても正しくラベルするため。
            fr = [d['Fmag_N'][i] for i in range(len(d['t_s'])) if d['t_s'][i] < split_t]
            fl = [d['Fmag_N'][i] for i in range(len(d['t_s'])) if d['t_s'][i] >= split_t]
            right_first = _median(fr) >= _median(fl)

    baseline_note = ''
    if args.baseline:
        if not os.path.isfile(args.baseline):
            print('空運転CSVが見つかりません:', args.baseline)
            return 2
        base = load_log(args.baseline)
        if not base['t_s']:
            print('空運転CSVにデータがありません:', args.baseline)
            return 2
        if args.baseline_persides:
            for ln in apply_baseline_persides(d, base):
                print(ln)
        else:
            shift = args.baseline_shift
            if args.baseline_align:
                shift, corr = estimate_baseline_shift(d, base)
                print('自動整列: 時刻シフト %.2f s（波形相関 %.3f）' % (shift, corr))
                if corr < 0.5:
                    print('  ※ 相関が低いです。開始タイミングが大きく違う/別プログラムの可能性。'
                          '--baseline-shift で手動調整も可。')
            apply_baseline(d, base, shift=shift)
            print('空運転を差し引きました:', os.path.basename(args.baseline),
                  '(shift=%.2fs)' % shift)
        baseline_note = ' [空運転差引済み]'
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
            if not right_first:
                print('（研磨順が左先と判定：ラベルを左右入れ替えて表示）')
            for ln in side_summary(d, split_t, right_first=right_first):
                print(ln)
            if not args.baseline and not args.auto_zero:
                print('※ 重力未除去です。--baseline 空運転.csv（正確）か --auto-zero（簡易）を併用してください。')

    # デザイン設定を読み込み（plot_config.json → 実行時オプションの順で上書き）
    style = load_style(here)
    if args.xlim:
        style['xlim_min'], style['xlim_max'] = args.xlim
    if args.ylim_force:
        style['force_ylim_min'], style['force_ylim_max'] = args.ylim_force
    if args.ylim_moment:
        style['moment_ylim_min'], style['moment_ylim_max'] = args.ylim_moment
    if args.figsize:
        style['figsize_w'], style['figsize_h'] = args.figsize
    if args.dpi:
        style['dpi'] = args.dpi
    if args.title is not None:
        style['title'] = args.title

    try:
        import matplotlib
        if args.no_show:
            matplotlib.use('Agg')   # 画面なしでも保存できるように
        if style.get('font_family'):
            matplotlib.rcParams['font.family'] = style['font_family']
        import matplotlib.pyplot as plt
    except ImportError:
        print('matplotlib がありません。 pip install matplotlib を実行してください。')
        return 3

    # --contact 指定があればその値で、無ければ接触判定しきい値で加工区間を塗る
    shade_thr = args.contact if args.contact is not None else (
        None if args.no_seg else args.seg_thr)
    auto_title = '%s%s   |   max|F|=%.1fN  max|M|=%.2fN*m  (%.0fs)' % (
        os.path.basename(path), baseline_note, s['fmax'], s['mmax'], s['dur'])
    title = style['title'] if style.get('title') else auto_title
    fig, ax1, ax2, lines, leg = make_figure(d, s, title, contact=shade_thr, style=style)

    # PNGは操作パネルを付ける前に保存（保存画像はパネル無しのきれいなグラフ）
    suffix = '_baselined' if args.baseline else ''
    base = os.path.splitext(path)[0] + suffix
    png = base + '.png'
    fig.savefig(png, dpi=style['dpi'])
    print('グラフを保存 :', png)

    # 力/モーメントを別々のPNGにも保存（--save-split）
    if args.save_split:
        for axes, sfx in (([ax1], 'force'), ([ax2], 'moment')):
            out = save_axes_region(fig, axes, base + '_' + sfx + '.png', style['dpi'])
            print('個別保存    :', out)

    if not args.no_show:
        if args.panel:
            add_control_panel(fig, ax1, ax2, lines, leg,
                              save_base=base, save_dpi=style['dpi'])
        plt.show()
    return 0


if __name__ == '__main__':
    sys.exit(main())
