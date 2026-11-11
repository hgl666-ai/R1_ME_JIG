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
Copy-Item (Join-Path $core "Src\link_proto.c")     $work -Force
Copy-Item (Join-Path $core "Inc\link_proto.h")     $work -Force
Copy-Item (Join-Path $here "test_motor.c")         $work -Force
Copy-Item (Join-Path $here "test_frame.c")         $work -Force
Copy-Item (Join-Path $here "test_link.c")          $work -Force
Copy-Item (Join-Path $here "stub\main.h")          $work -Force

# --- 1b. 校验 U2/U3 两份 link_proto 完全一致 ---
$u3inc = Join-Path (Split-Path -Parent $u2) "U3\Core\Inc\link_proto.h"
$u3src = Join-Path (Split-Path -Parent $u2) "U3\Core\Src\link_proto.c"
foreach ($pair in @(@((Join-Path $core "Inc\link_proto.h"), $u3inc),
                    @((Join-Path $core "Src\link_proto.c"), $u3src))) {
    if (Test-Path $pair[1]) {
        $h1 = (Get-FileHash $pair[0] -Algorithm SHA256).Hash
        $h2 = (Get-FileHash $pair[1] -Algorithm SHA256).Hash
        if ($h1 -ne $h2) {
            throw "U2/U3 两份 $(Split-Path -Leaf $pair[0]) 内容不一致（链路契约必须完全相同）"
        }
    } else {
        Write-Host "提示: 找不到 $($pair[1])，跳过一致性校验" -ForegroundColor Yellow
    }
}

# --- 2. 从 bsp_tmc2209.c 抽出 tmc_crc8() 函数体 ---
$tmcSrc = Join-Path $core "Src\bsp_tmc2209.c"
$src = Get-Content $tmcSrc -Raw
$m = [regex]::Match($src, 'static uint8_t tmc_crc8\([^\)]*\)\s*\{.*?\r?\n\}\r?\n', 'Singleline')
if (-not $m.Success) { throw "没能从 $tmcSrc 抽出 tmc_crc8()，测试中止" }
Set-Content -Path (Join-Path $work "crc8_extract.c") -Value $m.Value -Encoding UTF8

# --- 3. 编译并运行 ---
$result = 0
foreach ($t in @("test_frame", "test_motor", "test_link")) {
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
