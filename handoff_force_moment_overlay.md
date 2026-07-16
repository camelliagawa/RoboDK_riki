# 引き継ぎ書 : DynPick 力覚センサ 力/モーメント 記録・可視化

> Claude Code での継続作業用ハンドオフ。まず本書と `force_moment_overlay.py` を読んでから着手すること。
> 作業ブランチ: `claude/force-moment-viz-complete-fv2yz1`

---

## 0. 3行サマリ（まずこれ）

- **このロボットは RoboDK から実機をリアルタイム駆動できない**（FANUC の Stream Motion オプション R784 未導入を実機確認）。
- そのため本番は **ロボット＝ティーチペンダントで KENMA 実行 / 力＝PCで `--no-robodk` 記録** の分担運用に確定。
- **記録(`record_force.bat`)→グラフ化(`plot_force_log.py`)** まで実装・両PCで動作確認済み。**グラフは記録用・分析用どちらのPCからでも見られる**（両PCに matplotlib が入っていればよい）。残りは実研磨データの取得と微調整のみ。

---

## 1. 目的（ゴール）

FANUC ロボットで包丁の研磨（TORMEK T-8）を行う際に、**DynPick 6軸力覚センサで力・モーメントを測定して記録**し、
**時系列グラフで可視化**する。当初は「RoboDK の 3D ビューに矢印でリアルタイム表示」も狙ったが、
本機はリアルタイム駆動オプションが無いため、**記録＋グラフ化を本線**とする（矢印表示はシミュレーション/接続可能な環境での任意機能として維持）。

---

## 2. 環境・前提

| 項目 | 内容 |
|------|------|
| 設置 | 岐阜県工業技術センター |
| ロボット | FANUC LR Mate 200iD/7L（コントローラ R-30iB、iHMI ペンダント） |
| 力覚センサ | DynPick ZEF-6A100（6軸）。USBシリアル。`COM3` / `921600` |
| 研磨機 | TORMEK T-8 |
| ステーション | `260706_omori_kaiten.rdk`（ロボット / riki_Assem2 / Tormek / 曲線追従プロジェクト HaL・HaR / プログラム KENMA） |
| RoboDK 駆動 | **不可**（Stream Motion R784 未導入。FanucSM ドライバは接続不成立。ネットワークは 192.168.10.1:2000 に ping 疎通OK） |
| 座標系基準 | Kenma point / riki_Assem2 |

### PC 2台構成
- 役割の違いは「**センサが繋がっているか**」だけ。**グラフ化(`plot_force_log.py`)は両PCで動く**（matplotlib さえ入っていればどちらでも見られる）。
- **記録用PC（例: `metal2022`）** … DynPick センサを接続。conda base（`(base)`）に `pyserial`(+`robodk`) あり。ここで力を記録。**グラフも見たいので `matplotlib` を入れておく**（`pip install matplotlib` / conda なら `conda install matplotlib`）。
- **分析用PC（例: `koder`）** … Python 3.12 + `matplotlib`。CSV を受け取ってグラフ化。センサ不要。
- 両PCともリポジトリをクローンし本ブランチをチェックアウト済み。
- `plot_force_log.py` はスクリプトのあるフォルダ（無ければカレント）の最新 `force_log_*.csv` を自動選択するので、CSV をそのフォルダに置けばどちらのPCでも同じ操作で開ける。

---

## 3. 現状（Done）

### 記録・可視化パイプライン（本線）
- **`force_moment_overlay.py`** … 本体。以下のモードを持つ:
  - `--no-robodk`（**本番モード**）… RoboDK を使わずセンサ読み取り＋CSV記録のみ。常時記録。端末にライブ値表示。
    robolink を遅延 import にしたので **robodk 未インストールでも動く**。
  - 既定（RoboDK連携）… `robot.Busy()`＋関節角変化で動作検知し、動作中のみ力/モーメント矢印を RoboDK 3D に描画。
  - `--demo` … センサ無しでダミー波形。描画/記録/グラフの動作確認用。
- **`dynpick_sensor.py`** … DynPick 読み取り。`R` 要求 → 生値(LSB) → `(生値−零点)/感度` で N・N·m 換算。起動時 `tare()` で零点実測。
- **`plot_force_log.py`**（新規）… `force_log_*.csv` から力[N]・モーメント[N·m]を2段の時系列グラフに描画。PNG保存＋表示。統計出力。
- **`record_force.bat` / `plot_force.bat`**（新規）… デスクトップからダブルクリック起動用（PowerShell経由）。

### 確定したパラメータ・調整
- 接続: `DYNPICK_PORT='COM3'` / `DYNPICK_BAUD=921600`。
- 軸校正: `AXIS_MAP_FORCE = AXIS_MAP_MOMENT = [(1, -1), (2, -1), (0, +1)]`（3方向とも押した向きに矢印が伸びるのを確認）。
- 表示スケール: `FORCE_SCALE=7` / `MOMENT_SCALE=200`（矢印表示用。記録には無関係）。
- 矢印基点: `robot.PoseAbs()*SolveFK(Joints,PoseTool)` で参照フレーム非依存の絶対TCP。`BASE_OFFSET_TOOL` で微調整可。
- 動作検知: `MOTION_DETECT='both'`（Busy＋関節角変化のOR）、`MOTION_HOLD_S` で検知後の途切れ防止。
- RoboDK描画の通信断→自動再接続、`UPDATE_RATE=10`。

---

## 4. コード構成メモ

### `force_moment_overlay.py` 主要パラメータ（冒頭に集約）
- `USE_DEMO_SIGNAL` / `USE_ROBODK` … 波形と RoboDK 連携の有無（`--demo` / `--no-robodk`）。
- `DYNPICK_PORT` / `DYNPICK_BAUD` / `TARE_ON_START` / `TARE_SAMPLES`。
- `FORCE_SCALE` / `MOMENT_SCALE` / `*_DEADBAND` / `MAX_ARROW_LEN`（矢印表示用）。
- `UPDATE_RATE` / `EMA_ALPHA` / `ACTIVE_ONLY_WHEN_MOVING`。
- `MOTION_DETECT` / `JOINT_MOVE_DEG` / `MOTION_HOLD_S`（動作検知）。
- `AXIS_MAP_FORCE` / `AXIS_MAP_MOMENT` / `BASE_OFFSET_TOOL`。
- `LOG_CSV` / `LOG_PATH` / `LOG_EVERY`（記録）。

### 主要要素
- `read_wrench(t)` … `(fx,fy,fz,mx,my,mz)` を N, N·m で返す（実センサ or デモ）。
- `DynPickSensor` … シリアル接続・読み取り・`tare()`。
- `ForceLogger` … CSV追記（毎行 flush）。`tcp_pos=None` で位置列は空欄。
- `main_headless()` … `--no-robodk` の本番ループ（読み取り→記録→ライブ表示）。
- `main()` … RoboDK連携ループ（動作検知→読み取り→軸割当→EMA→ワールド変換→矢印→記録）。

### CSV 列
`time_iso, t_s, fx_N, fy_N, fz_N, mx_Nm, my_Nm, mz_Nm, Fmag_N, Mmag_Nm, tcp_x_mm, tcp_y_mm, tcp_z_mm, moving`
- 値は表示用の平滑化/軸割当ではなく **生の物理量(N, N·m)**。`--no-robodk` では TCP位置列は空欄。

---

## 5. 本番運用手順（確定版）

### A. 記録（記録用PC + センサ）
1. DynPick を USB 接続。RoboDK は不要。
2. デスクトップの **`record_force`** をダブルクリック（または `python force_moment_overlay.py --no-robodk --log`）。
3. **「零点測定中…」の間はツールに触れない**（tare）。本番ツール（包丁）装着後に一度やり直すと零点が正確。
4. 「記録開始」表示後、**ペンダントで KENMA を実行**。
   - 初回・調整時は **ペンダント速度オーバーライドを 10〜20%**（`SHIFT`＋`+%/-%` で5%刻み）、まず空中で確認。
   - プログラムの高速移動速度は「曲線追従プロジェクト HaL/HaR → その他の設定 → プログラムイベント → 高速移動速度」。
     研磨作業速度 50 mm/s とは別（移動を落としたい場合はここを 100 等に）。
5. 動作中は力がCSVに記録される（`record_force.bat` 既定で**リアルタイムグラフも別窓表示**）。
   終了は記録ウィンドウで **Ctrl+C** → `force_log_日時.csv` 保存 → **自動でグラフが開く**（`--plot`）。
   ※ リアルタイム/自動グラフには matplotlib が必要（`pip install matplotlib`）。不要なら `--live`/`--plot` を外す。

### B. グラフ化（どちらのPCでも可）
- **記録用PCでそのまま見る場合** … 記録に使ったフォルダには CSV があるので、デスクトップの **`plot_force`** をダブルクリックするだけ。
- **分析用PCで見る場合** … 記録した `force_log_*.csv` を分析用PCの `RoboDK_riki` フォルダにコピー（USB / 共有フォルダ / OneDrive 等）してから同じ操作。
- 共通操作:
  1. デスクトップの **`plot_force`** をダブルクリック（または `python plot_force_log.py`）。
  2. 最新CSVを自動選択し、グラフ表示＋同名PNG保存。ファイル指定は `python plot_force_log.py 〇〇.csv --contact 3.0`。
- どちらのPCでも見られるようにするため、**両PCに matplotlib を入れておく**（記録用PCが未導入なら `pip install matplotlib`）。

### 起動オプション（ファイルを編集せず切替）
`--demo` / `--port` / `--baud` / `--robot` / `--always-on` / `--detect busy|joints|both` / `--log` / `--log-path` / `--no-robodk` / `--no-open` / `--rate` / `--live` / `--plot`

- `--live` … 記録しながら**リアルタイムで力/モーメントを別ウィンドウ表示**（直近30秒を流し表示）。要 matplotlib。
  終了はグラフ窓の **STOPボタン / 窓を閉じる / q キー**（GUIにフォーカスがあると Ctrl+C が効かないため。端末での Ctrl+C も可）。
  ※ ライブ表示中は再描画に時間を取られ実効サンプリングが下がる（50Hz指定で約25Hz）。フル速度で録りたいときは `--live` を外す。
- `--plot` … 記録終了(Ctrl+C)後に**自動でグラフ(`plot_force_log.py`)を開く**。要 matplotlib。指定時はエクスプローラー表示より優先。
- `--no-open` … 記録終了後にCSVフォルダをエクスプローラーで自動表示しない（既定は自動で開く）。
- `--rate` … サンプリング周波数[Hz]。既定は **記録のみ(`--no-robodk`)=50Hz** / RoboDK連携=10Hz。
- ※ `record_force.bat` は既定で `--no-robodk --log --live --plot`（記録＋リアルタイム表示＋終了後グラフ）。

### `plot_force_log.py` のオプション
- `--baseline 空運転.csv` … **空運転CSVを差し引き、重力/姿勢オフセットを除去**して表示（真の接触力を見る本命）。
  - `--baseline-align` … 記録開始タイミングのズレを**波形(歯)の相互相関で自動整列**してから差引（推奨。LSが同一で尺は同じ・開始位置だけズレるケースに最適）。
  - `--baseline-persides` … 空運転を**右/左の姿勢ブロックに分け、サイドごとに整列**して差引。**研磨の順番(右先/左先)を変えても同じ空運転1本でOK**（順番非依存）。`plot_sides.bat` は既定でこれを使用。
  - `--baseline-shift 秒` … 手動で開始点を合わせる（`--baseline-align` を使わない場合）。出力PNGは `*_baselined.png`。
  - ⚠ **空運転は必ず「包丁を付けたまま砥石だけ逃がして」撮ること**。包丁を外すと自重が変わり（実測で約2.3N差）、差引後に包丁重量が“見かけの接触力”として残る。
- `--sides` … 右(HaR)/左(HaL)を自動で分けて**各サイドの|F|統計（平均/中央/p90/最大）と左右比**を表示。`--baseline` と併用で真の左右接触力を比較。
  境界がずれる場合は `--split 秒` で手動指定。（空運転差引＝姿勢ごとゼロなので、これが「左右それぞれ基準ゼロ」の正確版）
- `--auto-zero` … **空運転CSVなし**で各サイドの重力を自分自身から推定して差引（Webツールの「工程ごと自動ゼロ」相当）。
  ⚠ **砥石から離れず押しっぱなしの側（HaR）は姿勢リップルを接触力と誤認して過大に出る**（実測: air差引 HaR0.6N が auto-zero 2.1N）。
  離れる側(HaL)は概ね一致。**正確な左右比較は必ず `--baseline`（空運転）を使うこと**。auto-zero は空運転が全く無いときの目安用。

### 空運転はどれくらいの頻度で必要か
- 重力は**姿勢(W/P/R)だけ**で決まる。**食い込み量/押付け/速度/パス数を変えても姿勢は不変**なので、**同じ包丁・同じ角度なら `air.csv` を使い回せる（砥石を毎回動かす必要なし）**。
- 撮り直しが要るのは **包丁/治具の交換** か **角度(W/P/R)の大きな変更** のときだけ。
- `--contact N` … |F|≥N を加工区間として薄く塗る。
- `--seg-thr N`（既定1.0）… 接触区間の判定しきい値。**区間ごとの平均/最大力・空中ベースラインを表で出力**。
- `--no-seg` … 上記の区間解析を出さない。 / `--no-show` … 画面表示せずPNG保存のみ。
- **グラフの見た目（色/範囲/文字）の変更**:
  - コードを触らず変えるなら **`plot_config.json`**（`plot_config.example.json` をコピーして編集、同フォルダに置く。書いた項目だけ上書き）。
  - コード内なら `plot_force_log.py` 冒頭の **`STYLE` 辞書**。
  - 一時的な変更は実行時オプション: `--title` / `--xlim MIN MAX` / `--ylim-force MIN MAX` / `--ylim-moment MIN MAX` / `--figsize W H` / `--dpi`。
  - **`--panel`** … グラフ画面右に**操作パネルを常時表示**。`plot_force.bat`/`plot_sides.bat` は既定で付与。保存PNGはパネル無しのきれいな図（パネル追加前に保存）。パネルの内容:
    - **Series**（系列ON/OFF・色付きチェック）/ **Elements**（Title・Legend・X-axis・Y-axis・Grid・Shade の表示ON/OFF）
    - **Range**（X/F/M に "min max" 入力、空でAuto）/ **Auto range** / **Colors**（Default/Vivid/Warm/Mono）
    - **Line style**（Solid/Dashed/Dotted/DashDot・全線一括）/ **Title**（タイトル文字の変更）
  - 恒久設定(STYLE/plot_config.json)の追加キー: `show_title/legend/xaxis/yaxis/grid/shade`, `force_xaxis`（力グラフにもTime[s]軸）, `ls_fx`…`ls_mmag`（系列ごとの線種）。
  - 各名称の変更: `title`, `force_ylabel`, `moment_ylabel`, `xlabel`, 凡例 `label_fx`…`label_mmag`（日本語にするなら `font_family` も指定）。
  - 日本語ラベルにするなら `font_family` を `"Meiryo"` 等に（Windows）。`plot_config.json` は Git管理外。

### 補助ツール（センサのみ、RoboDK不要）
- `python dynpick_check.py --list` / `--scan-baud` / ライブ表示 … ポート・ボーレート確認、最大値からスケール目安。
- `python dynpick_calib.py --port COM3 --baud 921600` … 対話式で各ツール軸を押して `AXIS_MAP` を決定。

### （任意）RoboDK 矢印表示
`python force_moment_overlay.py --always-on` 等。ただし本機は RoboDK が実機を駆動/監視できないため、
矢印はシミュレーション姿勢に対して出る（実機位置とはズレる）。手押しでの軸/向き確認には有用。

---

## 6. 残タスク（To Do）

- [x] 実センサ接続 / 接続確認 / 軸校正 / スケール調整 / 基点見直し / API安定化
- [x] 動作検知の強化（Busy＋関節角フォールバック、`--detect`）
- [x] CSV記録（`--log`、`ForceLogger`）
- [x] RoboDK不要の記録モード（`--no-robodk`、`main_headless`）
- [x] 時系列グラフ生成（`plot_force_log.py`）＋デスクトップ起動（`*.bat`）
- [x] RoboDK駆動可否の確定（Stream Motion R784 未導入 → 不可）
- [x] **実研磨データの取得と検証**（2026-07-16, `..._124230.csv` / `..._131010.csv` の2本）。主要知見:
      - tare良好（開始/終了の空中 |F|≈0.1N、ドリフト無し）。
      - **測定値は工具自重（重力）オフセットに支配されている**（最重要）。位相1(HaR)は |F|≈8N が136秒続くが、
        **実機では包丁は砥石から離れている**（作業者確認）。つまりこの8Nは研磨力ではなく、
        開始時と違う姿勢に工具（包丁＋治具）が回り込んだことによる自重の投影。
      - 開始1回の tare は開始姿勢の重力しか消せず、HaR/HaL の研磨姿勢では最大8N級の見かけ力が残る。
        位相2(HaL)は姿勢が約180°反転して自重投影が変わり |F|≈1.5N。**生の数値からは左右の接触力を判断できない**。
      - 検証: 2本のCSVを時刻で差し引くと位相1の8Nが約0.9Nに相殺（＝共通の重力成分）。空運転差引で消える見込み。
- [x] **空運転(空研)ベースラインの取得**（2026-07-16, `..._133012.csv`, 砥石に一切非接触）。
      位相1(HaR)で非接触でも |F|≈6.0N・Mx≈-0.24・リップルまで出る → **プラトー/リップルは自重＋姿勢＋慣性で、研磨力ではない**と確定。
- [x] 差引の2課題を解明:
      (1) 空運転6.0N vs 研磨8.3N の約2.3N差は、**空運転で包丁を外したため**（自重が抜けた）。→ 空運転は**包丁付き・砥石だけ逃がして**撮る。
      (2) 空運転270s vs 研磨276s の尺差は **LS同一なので動作は同尺、記録開始ボタンのタイミング差だけ**。→ 一定シフトで整列可能。
- [x] **歯(パス)で自動整列する差引を実装**（`--baseline-align`）。相互相関で開始ズレを推定（既存データで −4.9s, 相関0.98を確認）。
- [ ] **【最優先】包丁付きの空運転を撮り直し → 自動整列差引で真の接触力を確認**:
      `python plot_force_log.py 研磨.csv --baseline 空運転_包丁付.csv --baseline-align`。
      その差引後グラフで左右(HaR/HaL)の接触力を比較し、刃の位置/反転軸・CNTを調整。
- [ ] **（空運転差引後に）左右の接触力を揃えるプロセス調整** … 差引後の左右接触力を比較し、刃の砥石中心に対する
      位置/反転軸を点検して両側を中庸・対称に。`HaR=CNT30` / `HaL=CNT60` の不一致も揃える。
- [ ] **感度/零点の実機校正（任意・精度向上）** … `dynpick_sensor.py` の `DEFAULT_SENS_*` は出荷特性値。
      厳密な絶対値が要るなら既知荷重で校正。相対変化・傾向を見る用途なら現状で十分。
- [ ] **モーメント向きの検証（任意）** … 必要なら既知トルクで確認。
- [ ] **拡張候補（任意）** … 加工区間の自動判定・集計、複数CSVの比較グラフ、Excel向け集計、数値テキスト表示。
- [ ] **CSV受け渡しの省力化（任意）** … 共有フォルダ/OneDrive に `force_log_*.csv` を保存する運用。

---

## 7. 注意点・既知のリスク

- **RoboDK からの実機駆動は不可**（Stream Motion 未導入）。RoboDK 接続を前提にした自動記録（Busy/関節角検知）は
  本機では効かないため、本番は必ず `--no-robodk`（常時記録）を使う。
- `force_log_*.csv` は Git 管理外（`.gitignore`）。PC間はファイルコピーで受け渡す。**グラフは両PCで見られる**（matplotlib が両方に必要）。記録用PCで直接見るならコピー不要。
- 軸マッピング・感度はセンサ取付向き/個体依存。**取付を変えたら必ず再校正**。
- `--no-robodk` の CSV は TCP 位置が空欄（実機位置は取れない）。力と位置を対応付けたい場合は時刻でプログラム工程と突き合わせる。
- 記録は毎行 flush なので Ctrl+C で止めても残る。長時間で間引くなら `LOG_EVERY` を上げる。
- グラフの軸ラベルは英語（matplotlib の日本語フォント依存を避けるため）。

---

## 8. 関連ファイル

- `force_moment_overlay.py` … 本体（記録 + RoboDK矢印 + DynPick接続 + `--no-robodk`）。
- `dynpick_sensor.py` … DynPick シリアル読み取り・LSB→N/Nm 換算。`python dynpick_sensor.py` でセルフテスト（HW不要）。
- `plot_force_log.py` … CSV → 力/モーメント時系列グラフ（matplotlib）。`--no-show` / `--contact N`。
- `record_force.bat` / `plot_force.bat` … デスクトップ起動用（ショートカットを作って使う）。
- `plot_sides.bat` … 最新の研磨CSVを `air.csv`（空運転）でサイド別差引＋左右サマリ表示（`--sides --auto-baseline`）。**空運転を `air.csv`（`air.csv.csv` も可）としてフォルダに置く**だけ。無ければ auto-zero にフォールバック。分岐はPython側(`--auto-baseline`)なので.batに特殊文字を入れず堅牢。
- `make_shortcuts.bat` … デスクトップに `record_force` / `plot_force` / `plot_sides` ショートカットを自動作成（最初に一度ダブルクリック）。
- `dynpick_check.py` … 接続・スケール確認ツール（RoboDK不要）。
- `dynpick_calib.py` … 軸校正ツール（対話式）。
- `calibration_guide.md` … 校正/運用の手順書。
- `requirements.txt` … `pyserial`（記録）/ `matplotlib`（グラフ）。`robodk` は矢印表示時のみ。
- `.gitignore` … `__pycache__/` `*.pyc` `force_log_*.csv`。
- `.gitattributes` … `*.bat` を CRLF で取り出し（cmd 互換）。

---

## 9. Claude Code への依頼例

- 「実研磨の `force_log_xxx.csv` を見たい。加工区間（|F|≥N）を自動判定して、区間ごとの最大/平均力を集計して。」
- 「条件違いの複数CSVを1枚のグラフに重ねて比較したい。」
- 「押した方向と符号が合わないので `AXIS_MAP_FORCE` を修正して。」
