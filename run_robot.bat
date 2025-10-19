@echo off
setlocal

REM === Настройки ===
set PROJECT_ROOT=%~dp0
set BUILD_DIR=%PROJECT_ROOT%out\build\x64-debug
set EXE_NAME=RoverBot.exe

REM === Проверка: существует ли папка сборки? ===
if not exist "%BUILD_DIR%" (
    echo [!] Папка сборки не найдена: %BUILD_DIR%
    echo     Запустите проект хотя бы один раз через Visual Studio.
    pause
    exit /b 1
)

REM === Переход в папку сборки ===
cd /d "%BUILD_DIR%"

REM === Сборка (инкрементная) ===
echo [+] Building in %BUILD_DIR%...
cmake --build . --config Debug

REM === Запуск, если exe существует ===
if exist "%BUILD_DIR%\%EXE_NAME%" (
    echo [+] Starting %EXE_NAME%...
    start "RoverBot" "%EXE_NAME%"
) else (
    echo [!] %EXE_NAME% не найден в %BUILD_DIR%
    pause
)

endlocal