# -*- coding: utf-8 -*-
"""
DynPick 接続・校正 確認ツール（RoboDK 不要 / センサ + PC だけで動く）

用途 :
  1) ポート/ボーレートの確認   … --list でポート一覧、--scan-baud で自動判定
  2) 軸校正の確認             … ライブ表示で「押した向き」と「どの軸が＋/−に動くか」を見る
  3) スケールの目安            … 実作業で観測した力/モーメントの最大値を記録

使い方 :
  python3 dynpick_check.py --list                       # 接続中のシリアルポートを列挙
  python3 dynpick_check.py --port COM3 --scan-baud       # そのポートでボーレート自動判定
  python3 dynpick_check.py --port COM3 --baud 921600     # ライブ数値表示（Ctrl+Cで終了）
  python3 dynpick_check.py --port COM3 --no-tare         # 起動時の零点実測をしない

Linux 例: --port /dev/ttyUSB0 / Windows 例: --port COM3
依存 : pyserial（pip install pyserial）
"""

import sys
import time
import argparse

from dynpick_sensor import (
    DynPickSensor, parse_dynpick_line,
    DEFAULT_BAUDRATE, DEFAULT_ZERO, DEFAULT_SENS_FORCE, DEFAULT_SENS_MOMENT,
    raw_to_wrench,
)

# ボーレート自動判定で試す候補（よくある値）
CANDIDATE_BAUDS = [921600, 460800, 230400, 115200, 57600, 38400]


def cmd_list():
    """接続中のシリアルポートを列挙する。"""
    try:
        from serial.tools import list_ports
    except Exception as e:
        print('pyserial が必要です（pip install pyserial）:', e)
        return 1
    ports = list(list_ports.comports())
    if not ports:
        print('シリアルポートが見つかりません。センサの USB 接続とドライバを確認してください。')
        return 1
    print('検出したポート:')
    for p in ports:
        print('  %-14s %s' % (p.device, (p.description or '')))
    print('\nこの中から DynPick のポートを選び、--port で指定してください。')
    return 0


def _try_one_baud(port, baud, timeout=0.2, tries=5):
    """指定ボーレートで数回読んでみて、DynPick 応答としてパースできれば raw を返す。"""
    import serial
    ser = None
    try:
        ser = serial.Serial(port, baud, timeout=timeout)
        try:
            ser.reset_input_buffer()
        except Exception:
            pass
        for _ in range(tries):
            ser.reset_input_buffer()
            ser.write(b'R')
            line = ser.readline()
            if not line:
                continue
            try:
                raw = parse_dynpick_line(line)
                # 生値が妥当な範囲（DynPick は無負荷 ≈8192、概ね 0..16383）
                if all(0 <= v <= 20000 for v in raw):
                    return raw, line
            except Exception:
                continue
        return None, None
    finally:
        if ser is not None:
            ser.close()


def cmd_scan_baud(port):
    """port に対して候補ボーレートを順に試し、応答が取れたものを表示する。"""
    print('ポート %s でボーレートを自動判定します…' % port)
    found = []
    for baud in CANDIDATE_BAUDS:
        try:
            raw, line = _try_one_baud(port, baud)
        except Exception as e:
            print('  %7d bps : オープン失敗 (%s)' % (baud, e))
            continue
        if raw is not None:
            print('  %7d bps : OK  raw=%s  応答=%r' % (baud, raw, line.strip()))
            found.append(baud)
        else:
            print('  %7d bps : 応答なし/不一致' % baud)
    print()
    if found:
        print('=> 使用ボーレート候補: %s' % found)
        print('   force_moment_overlay.py の DYNPICK_BAUD にこの値を設定してください。')
        return 0
    print('=> どのボーレートでも応答が取れませんでした。')
    print('   ポート指定・配線・電源、または応答形式（下記）を確認してください。')
    return 1


def cmd_live(port, baud, do_tare, tare_samples, rate):
    """ライブ数値表示。押した向きと軸の対応（校正）・最大値（スケール）を確認する。"""
    sensor = DynPickSensor(port=port, baudrate=baud)
    try:
        sensor.open()
    except Exception as e:
        print('ポートを開けません:', e)
        print('--list でポートを確認、--scan-baud でボーレートを確認してください。')
        return 1

    # 応答形式の実測サンプルを1つ表示（parse 前の生文字列も確認できるように）
    try:
        sensor._ser.reset_input_buffer()
        sensor._ser.write(b'R')
        first = sensor._ser.readline()
        print('応答サンプル(生文字列):', repr(first))
    except Exception:
        pass

    if do_tare:
        print('零点測定中… ツール/センサに何も触れないでください（%d サンプル）' % tare_samples)
        try:
            z = sensor.tare(tare_samples)
            print('零点[LSB]:', tuple(round(v, 1) for v in z))
        except Exception as e:
            print('零点測定に失敗:', e)

    print('\nライブ表示開始（Ctrl+C で終了）。既知の向きに押して、どの軸が動くか確認してください。')
    print('列: Fx Fy Fz [N]   Mx My Mz [N*m]   |F| |M|')
    peak = [0.0] * 6
    dt = 1.0 / max(1, rate)
    try:
        while True:
            t0 = time.time()
            try:
                fx, fy, fz, mx, my, mz = sensor.read_wrench()
            except Exception as e:
                print('読み取りエラー:', e)
                time.sleep(0.2)
                continue
            vals = [fx, fy, fz, mx, my, mz]
            for i in range(6):
                if abs(vals[i]) > peak[i]:
                    peak[i] = abs(vals[i])
            fmag = (fx * fx + fy * fy + fz * fz) ** 0.5
            mmag = (mx * mx + my * my + mz * mz) ** 0.5
            sys.stdout.write(
                '\rF %+6.2f %+6.2f %+6.2f   M %+6.3f %+6.3f %+6.3f   |F|%6.2f |M|%6.3f'
                % (fx, fy, fz, mx, my, mz, fmag, mmag))
            sys.stdout.flush()
            elapsed = time.time() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)
    except KeyboardInterrupt:
        pass
    finally:
        sensor.close()
        print('\n\n--- 観測した最大絶対値（スケール設定の目安）---')
        print('  Fx=%.2f Fy=%.2f Fz=%.2f [N]' % (peak[0], peak[1], peak[2]))
        print('  Mx=%.3f My=%.3f Mz=%.3f [N*m]' % (peak[3], peak[4], peak[5]))
        fpk = max(peak[0:3]) or 1.0
        mpk = max(peak[3:6]) or 1.0
        print('  目安: FORCE_SCALE ≈ %.0f mm/N,  MOMENT_SCALE ≈ %.0f mm/(N*m)'
              % (200.0 / fpk, 200.0 / mpk))
        print('  （最大の力/モーメントで矢印が約200mmになる倍率。MAX_ARROW_LEN と合わせて調整）')
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description='DynPick 接続・校正 確認ツール')
    ap.add_argument('--list', action='store_true', help='シリアルポートを列挙して終了')
    ap.add_argument('--port', help='シリアルポート 例 COM3 / /dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=DEFAULT_BAUDRATE, help='ボーレート（既定 %d）' % DEFAULT_BAUDRATE)
    ap.add_argument('--scan-baud', action='store_true', help='候補ボーレートを自動判定して終了')
    ap.add_argument('--no-tare', action='store_true', help='起動時の零点実測をしない')
    ap.add_argument('--tare-samples', type=int, default=100, help='零点実測のサンプル数（既定 100）')
    ap.add_argument('--rate', type=int, default=10, help='ライブ表示の更新レート[Hz]（既定 10）')
    args = ap.parse_args(argv)

    if args.list:
        return cmd_list()
    if not args.port:
        ap.error('--port を指定してください（--list でポート確認）')
    if args.scan_baud:
        return cmd_scan_baud(args.port)
    return cmd_live(args.port, args.baud, not args.no_tare, args.tare_samples, args.rate)


if __name__ == '__main__':
    sys.exit(main())
