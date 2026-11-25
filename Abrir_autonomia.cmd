@echo off
cd /d "%~dp0"
python -B Supervisorio_Autonomo.py
if errorlevel 1 pause
