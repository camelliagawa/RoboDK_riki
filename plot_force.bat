@echo off
rem ============================================================
rem  Plot force/moment time series from the latest log CSV.
rem  Double-click the desktop shortcut to run.
rem  Picks the newest force_log_*.csv, shows the graph and
rem  saves a PNG next to it. --panel shows on-screen controls
rem  (series show/hide, axis range, color themes).
rem  First time only: pip install matplotlib
rem
rem  Auto-updates to the latest version (git pull) before start;
rem  still launches the current version if the pull is blocked.
rem
rem  NOTE: keep this file ASCII-only. Non-ASCII text (e.g.
rem  Japanese) can break the shortcut on non-UTF-8 code pages.
rem ============================================================
powershell -NoExit -ExecutionPolicy Bypass -Command "Set-Location -LiteralPath '%~dp0'; $env:GIT_TERMINAL_PROMPT=0; Write-Host 'Updating to latest (git pull)...'; git pull; python plot_force_log.py --panel %*"
