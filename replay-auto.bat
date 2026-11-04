@echo off
rem Runs an Auto in the sim and opens it in AdvantageScope (tools\advantagescope\replay_auto.py). Close AdvantageScope first.
rem
rem   replay-auto.bat                                  every Auto on one field
rem   replay-auto.bat one_cycle_garden_park            just that one, with the balls
rem   replay-auto.bat one_cycle_garden_park --blue
rem   replay-auto.bat --no-sim                         reopen the last run's logs
rem
rem Then press play (space) in AdvantageScope.
cd /d "%~dp0"
py -3 tools\advantagescope\replay_auto.py %*
if "%~1"=="" pause
