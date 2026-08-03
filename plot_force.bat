@echo off
rem ============================================================
rem  Plot force/moment time series from the latest log CSV.
rem  Double-click the desktop shortcut to run.
rem  Picks the newest force_log_*.csv, shows the graph and
rem  saves a PNG next to it. --panel shows on-screen controls
rem  (series show/hide, axis range, color themes).
rem  First time only: pip install matplotlib
rem ============================================================
rem  起動時に自動で最新版へ更新します（git pull）。更新できない環境でも今の版で起動。
powershell -NoExit -ExecutionPolicy Bypass -Command "Set-Location -LiteralPath '%~dp0'; Write-Host '最新版に更新中 (git pull)...'; try { git pull } catch { Write-Host '（更新スキップ: 今の版で起動します）' }; python plot_force_log.py --panel %*"
