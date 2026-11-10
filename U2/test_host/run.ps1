# 主机侧回归测试: 把 U2 工程里真实的 .c 拷进 build/ 再用 gcc 编译运行。
# 每次运行都重新拷，所以不存在“测试的是旧代码”的问题。
param(
    [string]$Gcc = "C:\Program Files\mingw64\bin\gcc.exe"
)

$ErrorActionPreference = "Stop"

$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$u2   = Split-Path -Parent $here
$core = Join-Path $u2 "Core"
$work = Join-Path $here "build"

if (-not (Test-Path $Gcc)) { throw "找不到 gcc: $Gcc (用 -Gcc 指定路径)" }

New-Item -ItemType Directory -Force -Path $work | Out-Null
Get-ChildItem $work -File -ErrorAction SilentlyContinue | Remove-Item -Force

# --- 1. 把被测的真实源码拷进来 ---
Copy-Item (Join-Path $core "Src\bsp_motor.c")      $work -Force
Copy-Item (Join-Path $core "Inc\bsp_motor.h")      $work -Force
Copy-Item (Join-Path $core "Src\protocol_frame.c") $work -Force
Copy-Item (Join-Path $core "Inc\protocol_frame.h") $work -Force
Copy-Item (Join-Path $here "test_motor.c")         $work -Force
Copy-Item (Join-Path $here "test_frame.c")         $work -Force
Copy-Item (Join-Path $here "stub\main.h")          $work -Force

# --- 2. 从 bsp_tmc2209.c 抽出 tmc_crc8() 函数体 ---
$tmcSrc = Join-Path $core "Src\bsp_tmc2209.c"
$src = Get-Content $tmcSrc -Raw
$m = [regex]::Match($src, 'static uint8_t tmc_crc8\([^\)]*\)\s*\{.*?\r?\n\}\r?\n', 'Singleline')
if (-not $m.Success) { throw "没能从 $tmcSrc 抽出 tmc_crc8()，测试中止" }
Set-Content -Path (Join-Path $work "crc8_extract.c") -Value $m.Value -Encoding UTF8

# --- 3. 编译并运行 ---
$result = 0
foreach ($t in @("test_frame", "test_motor")) {
    Write-Host ""
    Write-Host "=== $t ===" -ForegroundColor Cyan
    & $Gcc -std=c99 -Wall -Wextra -I $work (Join-Path $work "$t.c") -o (Join-Path $work "$t.exe")
    if ($LASTEXITCODE -ne 0) { throw "编译失败: $t" }
    & (Join-Path $work "$t.exe")
    if ($LASTEXITCODE -ne 0) { $result = 1 }
}

Write-Host ""
if ($result -eq 0) {
    Write-Host "全部主机侧测试通过" -ForegroundColor Green
} else {
    Write-Host "有测试失败" -ForegroundColor Red
}
exit $result
