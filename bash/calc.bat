@echo off
setlocal enabledelayedexpansion

set "target_dir=%~1"
if "%target_dir%"=="" set "target_dir=."

set cpp_count=0
set h_count=0
set total_lines=0

echo ����ɨ��Ŀ¼: %target_dir%
echo ----------------------------

for /r "%target_dir%" %%f in (*.cpp *.h) do (
    if "%%~xf"==".cpp" (
        set /a cpp_count+=1
    ) else if "%%~xf"==".h" (
        set /a h_count+=1
    )
    
    for /f "usebackq" %%l in (`type "%%f" ^| find /c /v ""`) do (
        set "lines=%%l"
    )
    set /a total_lines+=!lines!
    echo ɨ���ļ�: %%~nxf �� !lines! ��
)

echo ----------------------------
echo ͳ�ƽ��:
echo    Cpp�ļ���: %cpp_count%
echo    H�ļ���  : %h_count%
echo �ļ�����    : %= �������� =%
set /a total_files=cpp_count + h_count
echo    %total_files%
echo ������      : %total_lines%
echo ----------------------------
pause
