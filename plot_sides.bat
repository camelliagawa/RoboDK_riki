@echo off
rem ============================================================
rem  Plot the latest grind log with a per-side (HaR right /
rem  HaL left) contact summary.
rem
rem  If  air.csv  exists in this folder (your air run: knife
rem  mounted, wheel backed off), it is subtracted for an ACCURATE
rem  per-side result. You only need ONE air.csv and can reuse it
rem  for all runs with the same tool and same orientations.
rem
rem  If  air.csv  is missing, it falls back to --auto-zero
rem  (rough, no air run needed). NOTE: auto-zero over-reads a
rem  side that stays in contact the whole time (e.g. HaR).
rem
rem  Needs matplotlib: pip install matplotlib
rem ============================================================
powershell -NoExit -ExecutionPolicy Bypass -Command "Set-Location -LiteralPath '%~dp0'; if (Test-Path 'air.csv') { Write-Host 'Using air.csv baseline (accurate).' -ForegroundColor Green; python plot_force_log.py --baseline air.csv --baseline-align --sides } else { Write-Host 'No air.csv -> using --auto-zero (rough). Put your air run as air.csv for accuracy.' -ForegroundColor Yellow; python plot_force_log.py --auto-zero --sides }"
