# 引き継ぎ書 : 力覚センサ × RoboDK 力/モーメント リアルタイム可視化

> Claude Code での継続作業用ハンドオフ。まず本書と `force_moment_overlay.py` を読み込んでから着手すること。

---

## 1. 目的（ゴール）

FANUC ロボット（Run on Robot 接続）を動かしたときに、**DynPick 力覚センサの計測を自動でスタート**し、
**力とモーメントを RoboDK の 3D ビュー内に矢印でリアルタイム表示**する。

---

## 2. 環境・前提

| 項目 | 内容 |
|------|------|
| ロボット | FANUC LR Mate 200iD/7L |
| 力覚センサ | DynPick ZEF-6A100（6軸）。Python で生値取得済み（単位 N, N·m） |
| 研磨機 | TORMEK T-8 |
| RoboDK 接続 | **Run on Robot（ドライバ接続）** |
| 実装方針 | 既存の DynPick 読み取り Python に **robolink（RoboDK Python API）を統合して一体化** |
| 座標系基準 | riki_Assem2 |
| 対象ステーション例 | 260706_omori_kaiten |

---

## 3. 現状（Done）

- たたき台スクリプト **`force_moment_overlay.py`** を作成、構文チェック済み。
- 実装済みの仕組み：
  1. **自動スタート/ストップ** … `robot.Busy()` でロボット動作を検知。動作中のみ矢印を表示し、停止で消す。
  2. **矢印描画** … 毎ループ TCP を基点に、力ベクトル（赤）とモーメントベクトル（青）を折れ線カーブで描画。
  3. **平滑化** … EMA ローパス + デッドバンドでノイズ／ガタつきを抑制。
  4. **デモモード** … `USE_DEMO_SIGNAL=True` でセンサ未接続でもダミー波形で描画テスト可能。
  5. **DynPick 実センサ読み取り**（新規） … `dynpick_sensor.py` を実装。シリアルで `R` 要求 → 生値(LSB) 受信 →
     `(生値 − 零点) / 主軸感度` で N / N·m へ換算。`read_wrench()` から呼び出し済み。
     感度・零点の初期値は riki（力覚センサ CSV グラフビューア）の出荷特性データ（ZEF-6A100-4-RAD）に準拠。
     開始時に無負荷で零点を自動実測する `tare()` 付き。
     ※ この読み取り部分は riki リポジトリの CSV 換算仕様（列順・換算式・感度）を参考に新規実装したもの。
       riki 自体は「記録済み CSV を後から見るビューア」で、ライブ読み取りは含まれていなかった。
- 描画方式：`AddCurve(points, None, False, PROJECTION_NONE)`
  （reference=None → ステーション絶対座標、投影なし＝空中に描画）。
  1 矢印 = 1 折れ線カーブ。ちらつき低減のため「新規追加 → 旧削除」の順で更新。

---

## 4. コード構成メモ（`force_moment_overlay.py`）

### 主要パラメータ（スクリプト冒頭に集約）
- `ROBOT_NAME` … `''` で先頭ロボット。名前指定可。
- `USE_DEMO_SIGNAL` … True でダミー波形、False で実センサ。
- `FORCE_SCALE [mm/N]` / `MOMENT_SCALE [mm/(N·m)]` … 矢印長さ倍率。
- `FORCE_DEADBAND` / `MOMENT_DEADBAND` … 非表示しきい値。
- `MAX_ARROW_LEN` … 矢印長さ上限（振り切れ防止）。
- `UPDATE_RATE [Hz]` / `EMA_ALPHA` … 更新レート／ローパス係数。
- `ACTIVE_ONLY_WHEN_MOVING` … 動作中のみ表示（自動スタート）の ON/OFF。
- `AXIS_MAP_FORCE` / `AXIS_MAP_MOMENT` … センサ生値 → ツール座標の軸割当 `(index, 符号)`。

### 主要要素
- `read_wrench(t)` … **★実センサ接続点★**。`(fx,fy,fz,mx,my,mz)` を N, N·m で返す。
- `WrenchArrow` クラス … 矢印の生成・更新・消去を管理。
- `main()` … 動作検知 → 読み取り → 軸割当 → EMA → ワールド変換 → 矢印更新のループ。

### 座標変換の流れ
```
センサ生値 → AXIS_MAP で軸割当 → ツール座標のベクトル
          → EMA 平滑化
          → TCP(=robot.PoseAbs()*robot.Pose()) の回転部でワールド方向へ
          → 始点=TCP位置, 長さ=|F|×SCALE で矢印描画
```

---

## 5. 残タスク（To Do）

優先度順：

- [x] **実センサ接続** … `dynpick_sensor.py`（DynPickSensor）を実装し `read_wrench()` に接続済み。既定 `USE_DEMO_SIGNAL=False`。
- [x] **接続確認** … 実機で確定: `DYNPICK_PORT='COM3'` / `DYNPICK_BAUD=921600`。応答形式もパース確認済み。
- [x] **軸校正** … `dynpick_calib.py` で実測 + RoboDK上の向き確認により確定:
      `AXIS_MAP_FORCE = AXIS_MAP_MOMENT = [(1, -1), (2, -1), (0, +1)]`。
      3方向とも「押した向きに矢印が伸びる」ことを確認済み。
- [x] **スケール調整** … 実測レンジ(最大 力≈30N/モーメント≈1N·m)から `FORCE_SCALE=7` / `MOMENT_SCALE=200` に設定。実研削で微調整可。
- [x] **基点の見直し** … `robot.PoseAbs()*SolveFK(Joints,PoseTool)` で参照フレーム非依存の絶対TCPを基点に修正（矢印の浮き解消）。`BASE_OFFSET_TOOL` で微調整可。
- [x] **RoboDK API 安定化** … 高頻度描画で API が切れる問題に、通信断→自動再接続 + `UPDATE_RATE=10` で対応。
- [x] **動作検知の強化** … `robot.Busy()` に加え **関節角変化フォールバック** を実装（`MOTION_DETECT='both'`）。Busy が鈍い/立たない場合も関節角の変化で検知。`MOTION_HOLD_S` で検知後の途切れ防止。`--detect busy|joints|both` で切替可。
      ※ 関節角検知は RoboDK が実機に接続（Run on Robot / モニタ）して実関節角を読める前提。未接続だとシミュレーション関節角しか見えない点に注意。
- [ ] **モーメント向きの確認（任意）** … 力と同じ取付回転を適用済み。必要なら既知トルクで検証。
- [ ] **モーメント表現の改善（任意）** … 回転を表す二重矢じり／円弧矢印など。
- [x] **CSV ログ保存** … 動作中の力/モーメントを CSV 記録（`--log`）。時刻・生値(N,N·m)・|F|/|M|・TCP位置を出力。`ForceLogger` クラスで実装。
- [ ] **拡張候補（任意）** … 数値テキスト表示、別ウィンドウのリアルタイムグラフ。
- [ ] **負荷確認** … 毎ループ `AddCurve`/`Delete` の描画・メモリ負荷を長時間運用で監視。重ければ `UPDATE_RATE` を下げる or 更新方式を再検討。

---

## 6. 実行環境・運用手順（実機で確立済み）

### 実行環境（Windows + Anaconda）
- 外部 Python（Miniconda base、または Python310）から `python force_moment_overlay.py` で実行。
  RoboDK を開いておけば robolink が自動接続する。
- 必要パッケージ: `robodk`, `pyserial`（`pip install robodk pyserial`）。
  RoboDK ボタン実行するなら、ツール→オプション→Python のインタープリタに、これらを入れた Python を指定。
- スクリプトは自フォルダを `sys.path` に追加するので、どこから実行しても隣の `dynpick_sensor.py` を読める。

### 確認・運用の順序
1. **描画テスト**: `python force_moment_overlay.py --demo` → ダミー波形で赤(力)・青(モーメント)矢印が TCP から出れば RoboDK 連携OK。
2. **接続/校正の道具**（RoboDK不要, センサのみ）:
   - `python dynpick_check.py --list` / `--scan-baud` / ライブ表示 … ポート・ボーレート確認、最大値からスケール目安。
   - `python dynpick_calib.py --port COM3 --baud 921600` … 対話式で各ツール軸を押して `AXIS_MAP` を決定。
3. **実センサ手押し**: `python force_moment_overlay.py --always-on` → 停止中でも常時表示。押した向きに矢印が伸びるか確認。
4. **本番**: `python force_moment_overlay.py`（既定 = 実センサ + 動作中のみ表示）を起動 → RoboDK で研磨プログラム(kenma)を Run。
   起動時 `零点測定中…` の間はツールに触れない。**本番ツール(包丁)装着後に一度 tare し直すと零点が正確**。

### 起動オプション（ファイルを編集せず切替）
`--demo` / `--port` / `--baud` / `--robot` / `--always-on` / `--detect busy|joints|both` / `--log` / `--log-path`

### CSV記録（動作中の力/モーメントを保存）
- `python force_moment_overlay.py --log` … 実センサ + 動作中のみ表示に加え、**動作中の計測をCSVに記録**。
  保存先は `force_log_日時.csv`（スクリプトと同じフォルダ）。`--log-path C:\data\run1.csv` で任意指定。
- 記録列: `time_iso, t_s, fx_N..mz_Nm, Fmag_N, Mmag_Nm, tcp_x/y/z_mm, moving`。
  値は表示用の平滑化/軸割当ではなく **read_wrench() の生の物理量(N, N·m)** を残す（後解析用）。TCP位置併記で力と位置を対応付け可。
- 毎サンプルで flush するので Ctrl+C で止めても記録は残る。長時間で間引くなら冒頭 `LOG_EVERY` を上げる。

---

## 7. 注意点・既知のリスク

- `robot.Busy()` はドライバ依存で反映が一瞬遅れることがある（→ 関節角変化のフォールバックを検討）。
- 毎ループ `AddCurve`/`Delete` で RoboDK のツリー／アンドゥ履歴が増える可能性。長時間運用は監視。
- TCP とセンサ計測原点のオフセットで、矢印の根元位置がずれうる。
- 軸マッピングはセンサ取付向き依存。**必ず実測で校正**すること。

---

## 8. 関連ファイル

- `force_moment_overlay.py` … 本体（力/モーメント可視化 + 自動スタート + DynPick 接続）。
- `dynpick_sensor.py` … DynPick / ZEF 系 6軸センサのシリアル読み取り・LSB→N/Nm 換算モジュール。
  `python3 dynpick_sensor.py` でパース／換算のセルフテスト実行可（ハードウェア不要）。
- `dynpick_check.py` … 接続・校正 確認ツール（RoboDK 不要）。ポート列挙・ボーレート自動判定・
  数値ライブ表示・最大値記録。`--list` / `--scan-baud` / ライブ表示。
- `calibration_guide.md` … 手順書（ポート/ボーレート・軸校正・スケール・本番の動かし方）。
- `requirements.txt` … 依存パッケージ（`pyserial`。`robolink`/`robodk` は RoboDK 同梱の Python から利用）。

---

## 9. Claude Code への最初の依頼例

> 「`force_moment_overlay.py` の `read_wrench()` を、`（既存の読み取りモジュール/関数名）` を使う実装に差し替えて、`USE_DEMO_SIGNAL=False` で動くようにして。センサ生値の単位・軸順は `（実際の仕様）` です。」

軸が合わない場合：

> 「押した方向と矢印の向きが `（例：X と Y が逆）` なので、`AXIS_MAP_FORCE` を修正して。」
