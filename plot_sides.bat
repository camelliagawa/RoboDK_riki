@echo off
rem ============================================================
rem  Plot the latest grind log with a per-side (right HaR /
rem  left HaL) contact summary.
rem
rem  SETUP (once): put your air-run CSV in this folder named
rem  air.csv  (knife mounted, wheel backed off). air.csv.csv is
rem  also accepted. One air.csv works for any grind order and
rem  any engagement/speed as long as the tool and angles match.
rem  If no air.csv is found it falls back to a rough auto-zero.
rem
rem  Needs matplotlib: pip install matplotlib
rem ============================================================
powershell -NoExit -ExecutionPolicy Bypass -Command "Set-Location -LiteralPath '%~dp0'; python plot_force_log.py --sides --auto-baseline --panel"
