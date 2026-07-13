# -*- coding: utf-8 -*-
"""
DynPick / ZEF シリーズ 6軸力覚センサ  リアルタイム読み取りモジュール

対象   : Wacohtech DynPick（例: ZEF-6A100-4-RAD）。USB シリアル接続。
役割   : シリアルから 1 サンプル要求 -> 生値(LSB) 受信 -> N / N*m へ換算。
        `force_moment_overlay.py` の read_wrench() から呼ばれる。

通信仕様（DynPick 標準）:
  - 単発要求 : ホストが 'R'(1バイト) を送信。
  - 応答     : 1行 ASCII。「1桁カウンタ + 6×4桁の16進」= Fx,Fy,Fz,Mx,My,Mz の生値。
               無負荷時は各値が概ね 0x2000(=8192) 付近。
               （DPViewer 出力 CSV の 3〜8 列目と同じ生値。）
  - 感度取得 : 'p' 送信で出荷感度を返す機種もある（本モジュールでは未使用）。

換算式（取説 3.3 / riki README と同じ）:
  力[N]        = (生値[LSB] - 零点[LSB]) / 主軸感度[LSB/N]
  モーメント[Nm] = (生値[LSB] - 零点[LSB]) / 主軸感度[LSB/Nm]

依存 : pyserial （pip install pyserial）。
"""

# ---- 出荷特性データ 初期値（ZEF-6A100-4-RAD）。別センサでは書き換える ----
# 主軸感度 [LSB/N]  : Fx, Fy, Fz
DEFAULT_SENS_FORCE  = (64.650, 65.010, 65.860)
# 主軸感度 [LSB/Nm] : Mx, My, Mz
DEFAULT_SENS_MOMENT = (1630.250, 1630.250, 1572.000)
# 零点出力 [LSB]（取説 8192±655 の代表値）。tare() で実測平均に置換可能。
DEFAULT_ZERO        = (8192.0,) * 6

# 接続の既定値
DEFAULT_BAUDRATE = 921600   # DynPick 標準。機種により 230400 等の場合あり
DEFAULT_TIMEOUT  = 0.1      # [s] 1 サンプルの読み取りタイムアウト


def parse_dynpick_line(line):
    """DynPick の 'R' 応答 1 行を 6 個の生値(int, LSB)へ変換して返す。

    受理する形式:
      - '8 2001 2000 1FFF 2003 2002 2001'（空白/カンマ区切りの16進、先頭カウンタ有無どちらも）
      - '820012000...'（区切り無し・1桁カウンタ+6×4桁の固定長16進）
    先頭のカウンタ桁（0-9）は読み飛ばす。解釈できなければ ValueError。
    """
    if isinstance(line, (bytes, bytearray)):
        line = line.decode('ascii', errors='ignore')
    s = line.strip().rstrip(',').strip()
    if not s:
        raise ValueError('empty line')

    # まず区切り（空白/カンマ）で分割してみる
    tokens = [tok for tok in s.replace(',', ' ').split() if tok]
    hex_tokens = None
    if len(tokens) == 7:
        # 先頭 = カウンタ、残り 6 = Fx..Mz
        hex_tokens = tokens[1:]
    elif len(tokens) == 6:
        hex_tokens = tokens
    else:
        # 区切り無しの固定長: カウンタ1桁 + 6×4桁 = 25 桁
        if len(s) >= 25:
            body = s[1:25]
            hex_tokens = [body[i:i + 4] for i in range(0, 24, 4)]
        elif len(s) >= 24:
            hex_tokens = [s[i:i + 4] for i in range(0, 24, 4)]

    if not hex_tokens or len(hex_tokens) != 6:
        raise ValueError('unrecognized DynPick line: %r' % line)

    return tuple(int(tok, 16) for tok in hex_tokens)


def raw_to_wrench(raw, zero, sens_force, sens_moment):
    """生値6個(LSB) -> (fx,fy,fz [N], mx,my,mz [N*m])。"""
    fx = (raw[0] - zero[0]) / sens_force[0]
    fy = (raw[1] - zero[1]) / sens_force[1]
    fz = (raw[2] - zero[2]) / sens_force[2]
    mx = (raw[3] - zero[3]) / sens_moment[0]
    my = (raw[4] - zero[4]) / sens_moment[1]
    mz = (raw[5] - zero[5]) / sens_moment[2]
    return (fx, fy, fz, mx, my, mz)


class DynPickSensor:
    """DynPick 力覚センサへのシリアル接続と読み取りを管理する。

    使用例:
        sensor = DynPickSensor(port='COM3')   # Windows。Linux は '/dev/ttyUSB0'
        sensor.open()
        sensor.tare()                          # 無負荷状態で零点を実測（任意）
        fx, fy, fz, mx, my, mz = sensor.read_wrench()
        ...
        sensor.close()
    """

    def __init__(self, port, baudrate=DEFAULT_BAUDRATE, timeout=DEFAULT_TIMEOUT,
                 zero=None, sens_force=None, sens_moment=None):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.zero = list(zero if zero is not None else DEFAULT_ZERO)
        self.sens_force = tuple(sens_force if sens_force is not None else DEFAULT_SENS_FORCE)
        self.sens_moment = tuple(sens_moment if sens_moment is not None else DEFAULT_SENS_MOMENT)
        self._ser = None

    def open(self):
        import serial  # pyserial（読み取り時のみ必要）
        self._ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        # 起動直後の残留バッファをクリア
        try:
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()
        except Exception:
            pass
        return self

    def close(self):
        if self._ser is not None:
            try:
                self._ser.close()
            finally:
                self._ser = None

    def read_raw(self):
        """センサに 1 サンプル要求して生値6個(LSB, tuple)を返す。"""
        if self._ser is None:
            raise RuntimeError('DynPickSensor.open() が呼ばれていません')
        self._ser.reset_input_buffer()
        self._ser.write(b'R')
        line = self._ser.readline()
        if not line:
            raise IOError('DynPick から応答がありません（ポート/ボーレートを確認）')
        return parse_dynpick_line(line)

    def read_wrench(self):
        """(fx,fy,fz [N], mx,my,mz [N*m]) を返す。"""
        raw = self.read_raw()
        return raw_to_wrench(raw, self.zero, self.sens_force, self.sens_moment)

    def tare(self, samples=100):
        """無負荷状態でのサンプルを平均し、各軸の零点(LSB)を実測値に更新する。
        測定前に必ず無負荷（ツールに何も触れない）状態で呼ぶこと。"""
        if samples <= 0:
            return
        acc = [0.0] * 6
        n = 0
        for _ in range(samples):
            try:
                raw = self.read_raw()
            except Exception:
                continue
            for i in range(6):
                acc[i] += raw[i]
            n += 1
        if n == 0:
            raise IOError('tare(): 有効なサンプルを取得できませんでした')
        self.zero = [acc[i] / n for i in range(6)]
        return tuple(self.zero)

    # with 文サポート
    def __enter__(self):
        return self.open()

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False


if __name__ == '__main__':
    # ハードウェア無しで動く簡易セルフテスト（パース＆換算の確認）
    demo = '8 2001 2000 1FFF 2003 2002 2001'
    raw = parse_dynpick_line(demo)
    print('parsed raw:', raw)
    w = raw_to_wrench(raw, DEFAULT_ZERO, DEFAULT_SENS_FORCE, DEFAULT_SENS_MOMENT)
    print('wrench N/Nm:', tuple(round(x, 4) for x in w))
    # 区切り無し固定長も確認
    assert parse_dynpick_line('8200120001FFF200320022001') == raw
    print('fixed-length parse OK')
