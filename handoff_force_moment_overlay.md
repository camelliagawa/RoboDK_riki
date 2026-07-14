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
- [ ] **接続確認** … `DYNPICK_PORT`（Windows 例 `COM3` / Linux 例 `/dev/ttyUSB0`）と `DYNPICK_BAUD`（既定 921600、機種により 230400 等）を実機に合わせる。
      応答が来ない場合はポート/ボーレート、`parse_dynpick_line()` の想定形式を確認。
- [ ] **軸校正** … 既知方向に押して、`AXIS_MAP_FORCE` / `AXIS_MAP_MOMENT` の index・符号を実測で合わせる。
- [ ] **スケール調整** … `FORCE_SCALE` / `MOMENT_SCALE` を実研削レンジに合わせる。振り切れは `MAX_ARROW_LEN` で調整。
- [ ] **Busy 検知の確認** … 出足が鈍ければ、関節角変化量による動作検知フォールバックを追加。
- [ ] **基点の見直し（必要なら）** … TCP とセンサ計測原点のオフセットがある場合、センサ原点用フレームを基点にする。
- [ ] **モーメント表現の改善（任意）** … 回転を表す二重矢じり／円弧矢印など。
- [ ] **拡張候補（任意）** … 数値テキスト表示、CSV ログ保存、別ウィンドウのリアルタイムグラフ。
- [ ] **負荷確認** … 毎ループ `AddCurve`/`Delete` による描画・メモリ負荷を長時間運用で監視。重ければ `UPDATE_RATE` を下げる or 更新方式を再検討。

---

## 6. 動作確認手順

1. RoboDK 起動 → ステーション読み込み → ロボットを **Run on Robot 接続**。
2. `USE_DEMO_SIGNAL=True` のまま実行 → ダミー波形で赤（力）・青（モーメント）矢印が動くことを確認。
3. `read_wrench()` を実センサに接続 → `USE_DEMO_SIGNAL=False`。
4. RoboDK 側で研磨プログラム（kenma 等）を Run → 動作中に矢印が自動表示されることを確認。
5. 軸・スケールを校正。

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
