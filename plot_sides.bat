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
rem  末尾/冒頭の「研磨でない山」を消したいときは Trim を使う。例:
rem     plot_sides.bat --trim - 254     （254秒より後を丸ごと除外）
rem  追加の引数(%*)はそのまま plot_force_log.py に渡ります。
powershell -NoExit -ExecutionPolicy Bypass -Command "Set-Location -LiteralPath '%~dp0'; python plot_force_log.py --sides --auto-baseline --panel %*"
