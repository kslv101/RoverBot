@echo off
setlocal

REM === ��������� ===
set PROJECT_ROOT=%~dp0
set BUILD_DIR=%PROJECT_ROOT%out\build\x64-debug
set EXE_NAME=RoverBot.exe

REM === ��������: ���������� �� ����� ������? ===
if not exist "%BUILD_DIR%" (
    echo [!] ����� ������ �� �������: %BUILD_DIR%
    echo     ��������� ������ ���� �� ���� ��� ����� Visual Studio.
    pause
    exit /b 1
)

REM === ������� � ����� ������ ===
cd /d "%BUILD_DIR%"

REM === ������ (������������) ===
echo [+] Building in %BUILD_DIR%...
cmake --build . --config Debug

REM === ������, ���� exe ���������� ===
if exist "%BUILD_DIR%\%EXE_NAME%" (
    echo [+] Starting %EXE_NAME%...
    start "RoverBot" "%EXE_NAME%"
) else (
    echo [!] %EXE_NAME% �� ������ � %BUILD_DIR%
    pause
)

endlocal