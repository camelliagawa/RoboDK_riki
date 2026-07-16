@echo off
rem ============================================================
rem  Plot the latest grind log with the air-run baseline removed
rem  and print a per-side (HaR right / HaL left) contact summary.
rem
rem  SETUP (once): put your air-run CSV in this folder and rename
rem  it to  air.csv  (knife mounted, wheel backed off).
rem  Then double-click this to compare the newest force_log_*.csv
rem  against air.csv with auto time-alignment.
rem  Needs matplotlib: pip install matplotlib
rem ============================================================
powershell -NoExit -ExecutionPolicy Bypass -Command "Set-Location -LiteralPath '%~dp0'; if (!(Test-Path 'air.csv')) { Write-Host 'air.csv がありません。空運転CSV(包丁付き・砥石逃がし)を air.csv という名前でこのフォルダに置いてください。' -ForegroundColor Yellow } else { python plot_force_log.py --baseline air.csv --baseline-align --sides }"
