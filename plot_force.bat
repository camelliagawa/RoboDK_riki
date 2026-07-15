@echo off
rem ============================================================
rem  Plot force/moment time series from the latest log CSV.
rem  Double-click the desktop shortcut to run.
rem  Picks the newest force_log_*.csv, shows the graph and
rem  saves a PNG next to it.
rem  First time only: pip install matplotlib
rem ============================================================
powershell -NoExit -ExecutionPolicy Bypass -Command "Set-Location -LiteralPath '%~dp0'; python plot_force_log.py"
