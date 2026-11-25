@echo off
cd /d "%~dp0"
python Supervisorio_Desktop.py
if errorlevel 1 pause
