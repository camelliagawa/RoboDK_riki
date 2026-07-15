@echo off
rem ============================================================
rem  力覚センサ 記録開始 (RoboDK 不要)
rem  デスクトップに置いたショートカットからダブルクリックで実行。
rem  - 「零点測定中...」の間はツールに触れないでください。
rem  - 記録中はロボットをティーチペンダントで動かしてください。
rem  - 終了/保存は、開いたウィンドウで Ctrl+C。
rem  ※ このファイル自体はリポジトリ(force_moment_overlay.py と同じ
rem     フォルダ)に置いたまま、そのショートカットをデスクトップへ。
rem ============================================================
powershell -NoExit -ExecutionPolicy Bypass -Command "Set-Location -LiteralPath '%~dp0'; python force_moment_overlay.py --no-robodk --log"
