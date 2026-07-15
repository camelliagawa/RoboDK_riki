@echo off
rem ============================================================
rem  最新の記録CSVから 力/モーメントの時系列グラフを作成
rem  デスクトップに置いたショートカットからダブルクリックで実行。
rem  最新の force_log_*.csv を自動で選び、グラフを表示＋PNG保存する。
rem  ※ 初回のみ matplotlib が必要: pip install matplotlib
rem ============================================================
powershell -NoExit -ExecutionPolicy Bypass -Command "Set-Location -LiteralPath '%~dp0'; python plot_force_log.py"
