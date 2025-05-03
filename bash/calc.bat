@echo off
setlocal enabledelayedexpansion

set "target_dir=%~1"
if "%target_dir%"=="" set "target_dir=."

set cpp_count=0
set h_count=0
set total_lines=0

echo 正在扫描目录: %target_dir%
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
    echo 扫描文件: %%~nxf → !lines! 行
)

echo ----------------------------
echo 统计结果:
echo    Cpp文件数: %cpp_count%
echo    H文件数  : %h_count%
echo 文件总数    : %= 计算总数 =%
set /a total_files=cpp_count + h_count
echo    %total_files%
echo 总行数      : %total_lines%
echo ----------------------------
pause
