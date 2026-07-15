# -*- coding: utf-8 -*-
"""
DynPick 軸校正ヘルパー（対話式・平均計測版）

AXIS_MAP_FORCE / AXIS_MAP_MOMENT を確実に決めるための道具。
ツールの各座標軸(+X/+Y/+Z)方向に「押し続けながら Enter」を3回行うと、
各押しの力を1.5秒平均し、3軸を必ず別々のセンサ軸に割り当てて、
そのまま貼り付けられる AXIS_MAP_FORCE の行を表示する。

使い方:
  python dynpick_calib.py --port COM3 --baud 921600

手順:
  1) 起動 → 零点測定中はツール/センサに触れない。
  2) RoboDK の TCP 座標軸 triad を確認（X=赤 / Y=緑 / Z=青）。
  3) 画面の指示どおり「ツール +X 方向へ押し続けたまま Enter」→「+Y」→「+Z」。
     ※ 厳密でなくてOK。だいたいその軸方向に、しっかり押し続けること。
  4) 表示された AXIS_MAP_FORCE の行を報告（またはそのまま設定）。

依存 : pyserial
"""

import sys
import time
import argparse
import itertools

from dynpick_sensor import DynPickSensor, DEFAULT_BAUDRATE

AXES = ['X', 'Y', 'Z']


def capture_avg(sensor, seconds, rate):
    """seconds 秒ぶん読み取り、(fx,fy,fz,mx,my,mz) の平均を返す。"""
    n = max(1, int(seconds * rate))
    acc = [0.0] * 6
    cnt = 0
    dt = 1.0 / rate
    for _ in range(n):
        t0 = time.time()
        try:
            w = sensor.read_wrench()
        except Exception:
            continue
        for k in range(6):
            acc[k] += w[k]
        cnt += 1
        el = time.time() - t0
        if el < dt:
            time.sleep(dt - el)
    if cnt == 0:
        return [0.0] * 6
    return [a / cnt for a in acc]


def solve_map(vectors):
    """3つの平均力ベクトル(各3成分)から、3軸を別々のセンサ軸に割り当てる。
    戻り値: [(idx, sign), ...]（tool X,Y,Z の順）。"""
    best = None
    for perm in itertools.permutations(range(3)):
        score = sum(abs(vectors[i][perm[i]]) for i in range(3))
        if best is None or score > best[0]:
            best = (score, perm)
    perm = best[1]
    amap = []
    for i in range(3):
        idx = perm[i]
        val = vectors[i][idx]
        sign = 1 if val >= 0 else -1
        amap.append((idx, sign))
    return amap


def fmt_map(amap):
    return '[' + ', '.join('(%d, %+d)' % (idx, sign) for (idx, sign) in amap) + ']'


def main(argv=None):
    ap = argparse.ArgumentParser(description='DynPick 軸校正ヘルパー（対話式）')
    ap.add_argument('--port', required=True, help='シリアルポート 例 COM3')
    ap.add_argument('--baud', type=int, default=DEFAULT_BAUDRATE)
    ap.add_argument('--tare-samples', type=int, default=100)
    ap.add_argument('--seconds', type=float, default=1.5, help='各押しの平均計測秒数')
    ap.add_argument('--rate', type=int, default=50)
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

    print('\n=== 軸校正（対話式）===')
    print('RoboDK の TCP 座標軸(X=赤 / Y=緑 / Z=青)に沿って、各軸ごとに押します。')
    print('「押し続けたまま Enter」を押すと 1.5秒 平均で計測します。\n')

    vectors = []
    try:
        for ax in AXES:
            input('▶ ツール +%s 方向にツールをしっかり押し続け、押したまま Enter を押す…' % ax)
            print('  計測中… そのまま押し続けてください（%.1f秒）' % args.seconds)
            w = capture_avg(sensor, args.seconds, args.rate)
            f = w[0:3]
            vectors.append(f)
            print('  計測結果: Fx=%+6.2f  Fy=%+6.2f  Fz=%+6.2f' % (f[0], f[1], f[2]))
            mag = max(abs(f[0]), abs(f[1]), abs(f[2]))
            if mag < 2.0:
                print('  ⚠ 押しが弱い/検出小さめ（最大 %.2fN）。もっと強く押すと精度が上がります。' % mag)
            print()
    except KeyboardInterrupt:
        print('\n中断しました。')
        sensor.close()
        return 1

    sensor.close()

    amap = solve_map(vectors)

    print('====================================================')
    print(' 校正結果 — force_moment_overlay.py に設定する行:')
    print('   AXIS_MAP_FORCE  = %s' % fmt_map(amap))
    print('   AXIS_MAP_MOMENT = %s   # 取付回転は力と同じ' % fmt_map(amap))
    print('====================================================')
    print(' 割当: toolX<-sensor%d(%s), toolY<-sensor%d(%s), toolZ<-sensor%d(%s)' % (
        amap[0][0], '+' if amap[0][1] > 0 else '-',
        amap[1][0], '+' if amap[1][1] > 0 else '-',
        amap[2][0], '+' if amap[2][1] > 0 else '-'))
    print(' この AXIS_MAP_FORCE の行を報告してください（またはそのまま貼り付け）。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
