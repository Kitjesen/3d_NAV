# sync_sunrise.ps1 — 把本地 main 同步到 sunrise@192.168.66.190
# 用法: 在 brain/lingtu 根目录执行 pwsh scripts/deploy/sync_sunrise.ps1
#
# 原理: git bundle -> scp -> ssh apply
# (sunrise 无法直连 GitHub，用此脚本替代 git pull)

param(
    [string]$SunriseHost = "sunrise@192.168.66.190",
    [string]$RemotePath = "/home/sunrise/data/SLAM/navigation"
)

$ErrorActionPreference = "Stop"
$RepoRoot = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$BundleTmp = "$env:TEMP\lingtu-sync.bundle"

Write-Host "=== LingTu → sunrise 同步 ===" -ForegroundColor Cyan
Write-Host "本地: $RepoRoot"
Write-Host "目标: ${SunriseHost}:$RemotePath"
Write-Host ""

# Step 1: 打 bundle
Write-Host "[1/4] git bundle create..." -ForegroundColor Yellow
Push-Location $RepoRoot
git bundle create $BundleTmp main
if ($LASTEXITCODE -ne 0) { throw "git bundle failed" }
$size = (Get-Item $BundleTmp).Length / 1MB
Write-Host "  bundle size: $([math]::Round($size,1)) MB"
Pop-Location

# Step 2: scp
Write-Host "[2/4] scp bundle..." -ForegroundColor Yellow
scp $BundleTmp "${SunriseHost}:/tmp/lingtu-sync.bundle"
if ($LASTEXITCODE -ne 0) { throw "scp failed" }

# Step 3: 远端 apply
Write-Host "[3/4] 远端 apply..." -ForegroundColor Yellow
$remote_cmd = @"
export GIT_DISCOVERY_ACROSS_FILESYSTEM=1
cd $RemotePath
git fetch /tmp/lingtu-sync.bundle main:refs/remotes/bundle/main 2>&1 | grep -v '^warning:'
git update-ref refs/heads/main refs/remotes/bundle/main
rm /tmp/lingtu-sync.bundle
echo DONE
"@
ssh $SunriseHost $remote_cmd
if ($LASTEXITCODE -ne 0) { throw "remote apply failed" }

# Step 4: 验证
Write-Host "[4/4] 验证..." -ForegroundColor Yellow
$result = ssh $SunriseHost "export GIT_DISCOVERY_ACROSS_FILESYSTEM=1; cd $RemotePath && git log --oneline -1"
Write-Host "  sunrise HEAD: $result"

# 清理
Remove-Item $BundleTmp -Force -ErrorAction SilentlyContinue

Write-Host ""
Write-Host "=== 同步完成 ===" -ForegroundColor Green
