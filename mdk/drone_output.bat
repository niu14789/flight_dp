@echo off
set exe_path=%cd%
cd %exe_path%
cd ../common/tools
UV4.exe %exe_path%\output\flight.hex ../../output/flight_v -v -xf %exe_path%\output\flight.axf -aof 0x8000