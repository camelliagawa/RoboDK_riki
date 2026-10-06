# -*- coding: utf-8 -*-
"""
刃・センサ・ツール座標の対応図（修正版）を描く。

  python docs/figures/make_axis_mapping_fig.py  ->  docs/figures/axis_mapping_2026-10-06.png

軸割当は force_moment_overlay.py の AXIS_MAP_FORCE = [(1,-1),(2,-1),(0,+1)]:
  ツール+X = センサ-Fy（刃の面に垂直）
  ツール+Y = センサ-Fz（刃に対して横方向・センサ円筒軸 = J6軸）
  ツール+Z = センサ+Fx（刃渡り方向）
力の値は停止試験 16:24（空運転差引後、ストロークごとの平均）。
"""
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Circle, FancyArrowPatch, Rectangle

REV_DATE = '2026-10-06'
matplotlib.rcParams['font.family'] = ['IPAGothic', 'IPAexGothic', 'Noto Sans CJK JP', 'sans-serif']

RED, GRN, BLU = '#c0392b', '#1e8449', '#1f5fbf'
BLADE, STONE = '#dfe6ef', '#e7e0d4'


def arrow(ax, p0, p1, color, lw=2.5, ls='-', ms=18):
    ax.add_patch(FancyArrowPatch(p0, p1, arrowstyle='-|>', mutation_scale=ms,
                                 color=color, lw=lw, linestyle=ls, shrinkA=0, shrinkB=0))


# ---------------------------------------------------------------- A: 斜め上から
def panel_a(ax):
    ax.set_xlim(-6.6, 6); ax.set_ylim(-5.2, 5.5); ax.set_aspect('equal'); ax.axis('off')
    ax.set_title('A. 刃とセンサ軸の対応（HaR の姿勢、斜め上から）', loc='left', fontsize=13)

    # 斜投影の基底（ツール座標 -> 紙面）
    eX = np.array([0.0, 1.0])            # 刃の面に垂直（この姿勢で上）
    eY = np.array([-0.55, 0.75])         # 刃に対して横方向（奥へ）
    eZ = np.array([0.88, 0.47])          # 刃渡り方向
    P = lambda x, y, z: x * eX + y * eY + z * eZ

    # 刃（Z: -4.6..2.4 = 切っ先..アゴ、Y: -0.9..0.9 = 刃先..峰）
    blade = [P(0, -0.9, -4.6), P(0, -0.9, 2.4), P(0, 0.9, 2.4), P(0, 0.9, -3.8)]
    ax.add_patch(Polygon(blade, closed=True, fc=BLADE, ec='#555', lw=1.5))
    ax.plot(*np.array([P(0, -0.9, -4.6), P(0, -0.9, 2.4)]).T, color='#222', lw=2.5)
    m = P(0, -1.25, -3.0)
    ax.text(*m, '刃先（エッジ）', rotation=np.degrees(np.arctan2(eZ[1], eZ[0])),
            ha='center', va='center', fontsize=10)
    ax.text(*P(0, -0.2, -5.2), '切っ先', ha='center', fontsize=9, color='#555')

    # 柄・グリッパ（アゴ側）とセンサ円筒（J6軸 = センサFz方向 = ツール-Y）
    handle = [P(0, -0.6, 2.4), P(0, -0.6, 3.8), P(0, 0.6, 3.8), P(0, 0.6, 2.4)]
    ax.add_patch(Polygon(handle, closed=True, fc='#a0a3a8', ec='#555', lw=1.2))
    c0, c1 = P(0, 0.0, 3.1), P(0, -3.4, 3.1) + np.array([0, 0.0])
    ax.plot(*np.array([c0, c1]).T, color='#888', lw=16, solid_capstyle='butt', alpha=0.85)
    ax.plot(*np.array([c0, c1]).T, color='#444', lw=1, ls=':')
    ax.text(*(c1 + np.array([0.15, -0.55])), 'センサ／フランジ\n（ロボット側）',
            fontsize=9, ha='left', va='top')
    # J6 回転の矢印（円筒軸まわり）
    mid = c0 + 0.7 * (c1 - c0)
    ax.add_patch(FancyArrowPatch(mid + np.array([-0.55, 0.45]), mid + np.array([0.6, -0.35]),
                                 connectionstyle='arc3,rad=-1.1', arrowstyle='-|>',
                                 mutation_scale=14, color='#7d3c98', lw=1.8))
    ax.text(*(mid + np.array([0.9, 0.1])), 'J6 回転\n（180°で HaR→HaL）',
            fontsize=9, color='#7d3c98', ha='left')

    # ツール座標 triad（刃の中央）
    o = P(0, 0, -1.4)
    L = 2.6
    arrow(ax, o, o + L * eX, RED)
    arrow(ax, o, o + L * eY, GRN)
    arrow(ax, o, o + L * eZ, BLU)
    ax.text(*(o + L * eX + np.array([0.0, 0.15])),
            'ツール +X ＝ センサ −Fy\n（刃の面に垂直・この姿勢で上向き）',
            color=RED, fontsize=10.5, ha='center', va='bottom')
    ax.text(*(o + L * eY + np.array([-0.15, 0.0])),
            'ツール +Y ＝ センサ −Fz\n（刃に対して横方向・J6軸）',
            color=GRN, fontsize=10.5, ha='right', va='center', fontweight='bold')
    ax.text(*P(0, -2.0, 1.8),
            'ツール +Z ＝ センサ +Fx\n（刃渡り方向）',
            color=BLU, fontsize=10.5, ha='center', va='top')

    ax.text(-6, -4.9,
            '出典: force_moment_overlay.py の AXIS_MAP_FORCE = [(1,-1),(2,-1),(0,+1)]。'
            'CSV の fx_N/fy_N/fz_N はセンサ生値。\n'
            '※ 各軸の＋側の向き（特に Fx・Fz）は写真と軸割当からの推定。押し試験で要確認。',
            fontsize=8.5, color='#444', va='top')


# ---------------------------------------------------------------- B/C: 断面
def triad_2d(ax, o, x_up, z_away, label_z):
    s = 1.0
    arrow(ax, o, (o[0], o[1] + (s if x_up else -s)), RED, lw=2, ms=13)
    arrow(ax, o, (o[0] + s, o[1]), GRN, lw=2, ms=13)
    ax.add_patch(Circle(o, 0.14, fc='white', ec=BLU, lw=2, zorder=5))
    if z_away:   # ⊗ 奥向き
        d = 0.09
        ax.plot([o[0] - d, o[0] + d], [o[1] - d, o[1] + d], color=BLU, lw=1.6, zorder=6)
        ax.plot([o[0] - d, o[0] + d], [o[1] + d, o[1] - d], color=BLU, lw=1.6, zorder=6)
    else:        # ⊙ 手前向き
        ax.add_patch(Circle(o, 0.04, fc=BLU, ec=BLU, zorder=6))
    ax.text(o[0] + 0.12, o[1] + (s if x_up else -s), 'ツール+X（センサ −Fy）',
            color=RED, fontsize=9, va='center')
    ax.text(o[0] + s + 0.15, o[1] - 0.25, 'ツール+Y（センサ −Fz, J6軸）',
            color=GRN, fontsize=9, va='top')
    ax.text(o[0] - 0.1, o[1] + (-0.55 if x_up else 0.55), label_z, color=BLU, fontsize=9,
            va='center')


def section(ax, title, x_up, z_away, label_z, root, tip):
    ax.set_xlim(-3.6, 4.2); ax.set_ylim(-4.2, 5.8); ax.set_aspect('equal'); ax.axis('off')
    ax.set_title(title, loc='left', fontsize=13)
    triad_2d(ax, (-2.6, 4.5), x_up, z_away, label_z)

    # 砥石・刃（模式）
    ax.add_patch(Circle((0, -2.0), 2.1, fc=STONE, ec='#857a66', lw=1.5))
    ax.add_patch(Rectangle((-3.6, -4.4), 7.8, 0.2, fc='white', ec='none', zorder=3))
    ax.text(0, -2.5, '砥石', ha='center', fontsize=11, color='#555')
    ax.add_patch(Rectangle((-1.95, 0.1), 3.9, 0.16, fc=BLADE, ec='#555', lw=1.2, zorder=4))
    ax.text(-2.05, 0.18, '刃（断面・模式）', ha='right', va='center', fontsize=9, color='#444')
    # UF 方向
    ax.annotate('', xy=(2.6, -2.0), xytext=(2.6, -3.3),
                arrowprops=dict(arrowstyle='->', color='#777'))
    ax.annotate('', xy=(3.9, -3.3), xytext=(2.6, -3.3),
                arrowprops=dict(arrowstyle='->', color='#777'))
    ax.text(2.7, -2.0, 'UF +Z\n（上と仮定）', fontsize=8, color='#666', va='top')
    ax.text(3.5, -3.15, 'UF +Y', fontsize=8, color='#666')

    base = np.array([0.0, 0.26])
    sc = 0.33   # [N] -> 図の長さ
    for (mag, ang, lab, col, ls, lw) in (root + ('#111', '-', 2.6), tip + ('#888', '--', 2.2)):
        v = mag * sc * np.array([np.sin(np.radians(ang)), np.cos(np.radians(ang))])
        arrow(ax, base, base + v, col, lw=lw, ls=ls, ms=16)
        end = base + v
        ax.text(end[0] + (0.15 if ang > 0 else -0.15), end[1] + 0.1, lab, color=col,
                fontsize=10, ha='left' if ang > 0 else 'right', va='bottom')


def main():
    fig = plt.figure(figsize=(19.5, 9.6), dpi=100)
    gs = fig.add_gridspec(1, 3, width_ratios=[1.25, 1, 1], wspace=0.05,
                          left=0.01, right=0.99, top=0.88, bottom=0.12)
    panel_a(fig.add_subplot(gs[0]))
    # 角度は鉛直から、+ がツール+Y 側（紙面右）
    section(fig.add_subplot(gs[1]), 'B. HaR（右面）刃渡り方向から見た断面',
            x_up=True, z_away=True, label_z='ツール+Z：紙面の奥向き（センサ +Fx、刃渡り）',
            root=(6.5, -40, '刃元 6.5N\n上＋横(−Y)に40°'),
            tip=(3.6, -51, '刃先 3.6N\n−Y に51°'))
    section(fig.add_subplot(gs[2]), 'C. HaL（左面）J6（＝センサFz軸）を180° 反転',
            x_up=False, z_away=False, label_z='ツール+Z：紙面の手前向き（センサ +Fx、刃渡り）',
            root=(6.2, -3, '刃元 6.2N\nほぼ真上'),
            tip=(2.3, 22, '刃先 2.3N\n+Y に22°'))

    fig.text(0.01, 0.975, '刃・センサ・ツール座標の対応図', fontsize=16, fontweight='bold', va='top')
    fig.text(0.99, 0.975, '修正版  %s' % REV_DATE, fontsize=14, fontweight='bold',
             color='#b03a2e', ha='right', va='top',
             bbox=dict(boxstyle='round,pad=0.35', fc='#fdecea', ec='#b03a2e'))
    fig.text(0.99, 0.925, '修正点：J6軸はツール+Z（センサFx）ではなく、ツール+Y（センサFz・円筒軸）',
             fontsize=10.5, color='#b03a2e', ha='right', va='top')

    fig.text(0.02, 0.085,
             '矢印＝砥石が刃を押す力（停止試験 16:24、空運転差引後、ストロークごとの平均）。'
             'J6 はセンサFz（ツールY）軸まわりの回転なので、反転するとツール+X（センサFy）と'
             'ツール+Z（センサFx）は向きが逆になり、ツール+Y（センサFz）は同じ横向きのまま。\n'
             '→ 左右が鏡像なら、横方向の力（センサFz）は左右で同じになるはず。'
             '右（HaR）だけ横に大きく押されているのが今回の非対称。',
             fontsize=10.5, va='top')

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       'axis_mapping_%s.png' % REV_DATE)
    fig.savefig(out, dpi=100)
    print(out)


if __name__ == '__main__':
    main()
