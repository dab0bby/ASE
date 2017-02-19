@echo off
setlocal enabledelayedexpansion

set /a count=0

if not exist ..\ASE\x64\Release\ASE.exe (
    echo Can not find ASE.exe. Please make a x64 release build.
    exit
)

if not exist results mkdir results

for /d %%D in (*) do (
    set folder=%%~fD
    if exist !folder!\scene.pcd (
        if exist !folder!\object.pcd (
            if exist !folder!\reference.stl (
                set /a count="!count!+1"
                ..\ASE\x64\Release\ASE.exe !folder!\scene.pcd !folder!\object.pcd !folder!\reference.stl %~dp0results\result_!count!_no_view.txt
                ..\ASE\x64\Release\ASE.exe !folder!\scene.pcd !folder!\object.pcd !folder!\reference.stl %~dp0results\result_!count!_view.txt -v
            )
        )
    )
)