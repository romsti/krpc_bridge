@echo off
rem ---------------------------------------------------------------------------
rem  KRPC.Bridge build helper (Windows)
rem
rem    build.cmd                            find KSP by itself, build to dist\
rem    build.cmd deploy                     ... and copy into GameData
rem    build.cmd "D:\Games\KSP_1.12.5"      use that install, and remember it
rem    build.cmd "D:\Games\KSP_1.12.5" deploy
rem    build.cmd verify                     type-check only, no KSP needed at all
rem    build.cmd forget                     drop the remembered KSP path
rem
rem  YOU DO NOT NORMALLY NEED TO GIVE THE PATH. tools\find_ksp.py looks at the KSPROOT
rem  variable, at ksp.path in this folder, at every Steam library declared in
rem  libraryfolders.vdf - which is what finds an install on a second drive - and then at
rem  the usual locations. Give the path once if all that fails and it is remembered.
rem
rem  A path is needed at all only because the build compiles against the game's own
rem  assemblies, which may not be redistributed and so cannot live in this repo.
rem
rem  Output lands in dist\GameData\, laid out exactly like the destination.
rem  Nothing is written into your install unless you pass "deploy".
rem ---------------------------------------------------------------------------
setlocal EnableDelayedExpansion

set "REPO=%~dp0"
set "DIST=%REPO%dist\GameData\KRPC.Bridge"

if /I "%~1"=="verify" goto :verify
if /I "%~1"=="forget" (
  python "%REPO%tools\find_ksp.py" --forget
  exit /b 0
)

rem  First argument is either a KSP path or the word "deploy".
set "KSP=%~1"
set "DEPLOY=%~2"
if /I "%KSP%"=="deploy" (
  set "KSP="
  set "DEPLOY=deploy"
)

echo.
echo === 1/4  type-check against stubs (no KSP needed) ===
dotnet build "%REPO%build\verify\Verify.csproj" -v minimal --nologo
if errorlevel 1 goto :failed

echo.
echo === 2/4  locating KSP ===
if not "%KSP%"=="" (
  rem  An explicit path wins, and is remembered so this is the last time you type it.
  if not exist "%KSP%\KSP_x64_Data\Managed\Assembly-CSharp.dll" (
    echo   NOT A KSP INSTALL: "%KSP%"
    echo   Expected "%KSP%\KSP_x64_Data\Managed\Assembly-CSharp.dll".
    goto :failed
  )
  > "%REPO%ksp.path" echo %KSP%
  echo   %KSP%   ^(given, and remembered for next time^)
) else (
  for /f "usebackq delims=" %%K in (`python "%REPO%tools\find_ksp.py" --quiet 2^>nul`) do set "KSP=%%K"
  if "!KSP!"=="" (
    python "%REPO%tools\find_ksp.py"
    goto :failed
  )
  echo   !KSP!   ^(found automatically^)
  set "KSP=!KSP!"
)

if not exist "!KSP!\GameData\kRPC\KRPC.Core.dll" (
  echo.
  echo   kRPC IS NOT INSTALLED in that copy of KSP.
  echo   Expected "!KSP!\GameData\kRPC\KRPC.Core.dll".
  echo   Contents of GameData\kRPC:
  dir /b "!KSP!\GameData\kRPC\*.dll" 2>nul
  echo.
  echo   Install kRPC 0.6.x there, or point at another copy:
  echo       .\build.cmd "path\to\other\KSP"
  goto :failed
)
set "KSP=!KSP!"

echo.
echo === 3/4  build the solution ===
dotnet build "%REPO%KRPC.Bridge.sln" -c Release -p:KSPRoot="%KSP%" -v minimal --nologo
if errorlevel 1 goto :failed

echo.
echo === 4/4  validate every kRPC signature with kRPC's own scanner ===
rem  This is the check that matters. One malformed signature in ANY loaded assembly
rem  disables the whole kRPC server in game, not just the offending service - so
rem  catching it here, in a second, is worth more than every other test in the repo.
rem  NO --nologo on this line. It is not an option `dotnet run` understands, so it gets
rem  forwarded to the tool, which reads it as the KSP path and fails with
rem  "KSP managed assemblies not found under '--nologo'".
set "SCANARGS="
for %%F in ("%DIST%\*.dll" "%DIST%\Plugins\*.dll") do set "SCANARGS=!SCANARGS! "%%~fF""
dotnet run --project "%REPO%build\scan\Scan.csproj" -c Release -v quiet -- "!KSP!" !SCANARGS!
if errorlevel 1 (
  echo.
  echo   SIGNATURE CHECK FAILED - do NOT deploy this build.
  echo   In game these errors would disable the entire kRPC server.
  goto :failed
)

if /I not "!DEPLOY!"=="deploy" goto :done

echo.
echo === deploying to GameData ===
xcopy /E /I /Y /Q "%REPO%dist\GameData\KRPC.Bridge" "%KSP%\GameData\KRPC.Bridge\" >nul
if errorlevel 1 goto :failed
echo   copied to "%KSP%\GameData\KRPC.Bridge\"
echo.
echo   If you are upgrading from the single-DLL version, DELETE the old
echo   GameData\KRPC.Bridge\Plugins\KRPC.Bridge.dll by hand. Leaving it there makes
echo   two assemblies declare the same kRPC service names, and kRPC refuses the
echo   duplicate - which takes the whole server down.
goto :after

:done
echo.
echo Built to %REPO%dist\GameData\KRPC.Bridge\
echo Copy that folder into "!KSP!\GameData\", or re-run with:
echo    .\build.cmd deploy

:after
echo.
echo Then start KSP, start the kRPC server, and run:
echo    python "%REPO%python\check_bridge.py"
exit /b 0

:verify
echo.
echo === type-check against stubs (no KSP needed) ===
dotnet build "%REPO%build\verify\Verify.csproj" -v minimal --nologo
if errorlevel 1 goto :failed
echo.
echo Sources type-check. This produces NOTHING usable in game - it links the stubs
echo in build\stubs\, and it does NOT validate kRPC signatures. Run build.cmd with
echo your KSP path for the real DLLs and the signature scan.
exit /b 0

:failed
echo.
echo BUILD FAILED
exit /b 1
