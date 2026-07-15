# -*- coding: utf-8 -*-
"""
DynPick 軸校正ヘルパー（AXIS_MAP_FORCE / AXIS_MAP_MOMENT を決めるための道具）

使い方:
  python dynpick_calib.py --port COM3 --baud 921600

やること:
  センサを tare（零点実測）した後、力とモーメントを数値表示し、
  「今どのセンサ軸が主に反応しているか（index と 符号）」を大きく表示する。

校正の手順（RoboDK のツール座標triadを見ながら）:
  1) RoboDK の TCP に出ている座標軸 triad を確認（X=赤, Y=緑, Z=青）。
  2) ツールを「ツール +X（赤軸の向き）」へグッと押す。
     -> 本ツールの「主軸 FX/FY/FZ と 符号」を読む。これが toolX の割当。
  3) 同様に「ツール +Y（緑軸）」「ツール +Z（青軸）」へ押して読む。
  4) 3つの結果を報告 -> AXIS_MAP_FORCE が確定できる。

依存 : pyserial
"""

import sys
import time
import argparse

from dynpick_sensor import DynPickSensor, DEFAULT_BAUDRATE

FORCE_LABELS  = ['X', 'Y', 'Z']


def dominant(vals, threshold):
    """|値|が最大の軸を (index, 符号文字, 大きさ) で返す。全て閾値未満なら None。"""
    i = max(range(len(vals)), key=lambda k: abs(vals[k]))
    if abs(vals[i]) < threshold:
        return None
    sign = '+' if vals[i] >= 0 else '-'
    return i, sign, vals[i]


def main(argv=None):
    ap = argparse.ArgumentParser(description='DynPick 軸校正ヘルパー')
    ap.add_argument('--port', required=True, help='シリアルポート 例 COM3')
    ap.add_argument('--baud', type=int, default=DEFAULT_BAUDRATE)
    ap.add_argument('--tare-samples', type=int, default=100)
    ap.add_argument('--force-th', type=float, default=1.0, help='主軸判定の力しきい値[N]')
    ap.add_argument('--moment-th', type=float, default=0.05, help='主軸判定のモーメントしきい値[N*m]')
    ap.add_argument('--rate', type=int, default=10)
    args = ap.parse_args(argv)

    sensor = DynPickSensor(port=args.port, baudrate=args.baud)
    try:
        sensor.open()
    except Exception as e:
        print('ポートを開けません:', e)
        return 1

    print('零点測定中… ツール/センサに触れないでください（%d サンプル）' % args.tare_samples)
    try:
        sensor.tare(args.tare_samples)
    except Exception as e:
        print('零点測定に失敗:', e)

    print('\n=== 軸校正モード ===')
    print('RoboDK の TCP 座標軸(X=赤,Y=緑,Z=青)に沿ってツールを押し、下の「主軸」を読む。')
    print('力(F) の主軸が、その方向の toolX/Y/Z に割り当てるセンサ軸です。（Ctrl+C で終了）\n')

    dt = 1.0 / max(1, args.rate)
    try:
        while True:
            t0 = time.time()
            try:
                fx, fy, fz, mx, my, mz = sensor.read_wrench()
            except Exception as e:
                print('読み取りエラー:', e)
                time.sleep(0.2)
                continue

            f = [fx, fy, fz]
            m = [mx, my, mz]
            fd = dominant(f, args.force_th)
            md = dominant(m, args.moment_th)

            if fd is None:
                f_txt = '主軸F: ---            '
            else:
                i, sign, val = fd
                f_txt = '主軸F: F%s 符号%s (index %d, %s1) %+6.2fN ' % (
                    FORCE_LABELS[i], sign, i, sign, val)

            if md is None:
                m_txt = '主軸M: ---'
            else:
                i, sign, val = md
                m_txt = '主軸M: M%s 符号%s (index %d, %s1) %+6.3f' % (
                    FORCE_LABELS[i], sign, i, sign, val)

            sys.stdout.write('\rF %+6.2f %+6.2f %+6.2f  | %s | %s' % (fx, fy, fz, f_txt, m_txt))
            sys.stdout.flush()

            elapsed = time.time() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)
    except KeyboardInterrupt:
        pass
    finally:
        sensor.close()
        print('\n終了しました。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
