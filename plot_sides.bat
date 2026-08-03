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
rem
rem  This launcher auto-updates to the latest version (git pull)
rem  before starting. If git / network / local changes block the
rem  pull, it still launches the current version.
rem
rem  To drop non-grinding peaks (retract spike at the end, entry
rem  spike at the start), use Trim. Example:
rem      plot_sides.bat --trim 0 254
rem  Extra args (%*) are passed straight to plot_force_log.py.
rem
rem  NOTE: keep this file ASCII-only. Non-ASCII text (e.g.
rem  Japanese) can break the shortcut on non-UTF-8 code pages.
rem ============================================================
powershell -NoExit -ExecutionPolicy Bypass -Command "Set-Location -LiteralPath '%~dp0'; $env:GIT_TERMINAL_PROMPT=0; Write-Host 'Updating to latest (git pull)...'; git pull; python plot_force_log.py --sides --auto-baseline --panel %*"
