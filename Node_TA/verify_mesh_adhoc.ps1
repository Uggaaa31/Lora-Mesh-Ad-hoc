# verify_mesh_adhoc.ps1
# Verifikasi cepat bahwa semua node masih menerapkan LoRa mesh ad hoc berbasis AODV
# Jalankan dari folder: d:\Skripsi\Tugas Akhir_Lora\Node_TA\

$ErrorActionPreference = "Stop"

$nodeChecks = @(
    @{
        Name = "LoRa_Mesh_Node"
        Sketch = "LoRa_Mesh_Node.ino"
        RequireFatigueStatusHandler = $false
        RequireVehicleTelemetrySender = $false
    },
    @{
        Name = "LoRa_Node_TRK002"
        Sketch = "LoRa_Node_TRK002.ino"
        RequireFatigueStatusHandler = $false
        RequireVehicleTelemetrySender = $false
    },
    @{
        Name = "LoRa_Node_TRK003"
        Sketch = "LoRa_Node_TRK003.ino"
        RequireFatigueStatusHandler = $false
        RequireVehicleTelemetrySender = $false
    },
    @{
        Name = "lora_saenab"
        Sketch = "lora_saenab.ino"
        RequireFatigueStatusHandler = $true
        RequireVehicleTelemetrySender = $false
    },
    @{
        Name = "lora_nailah"
        Sketch = "lora_nailah.ino"
        RequireFatigueStatusHandler = $false
        RequireVehicleTelemetrySender = $true
    }
)

$configPatterns = @(
    '#define LORA_FREQUENCY',
    '#define ROUTE_TIMEOUT',
    '#define RREQ_TIMEOUT',
    '#define RREQ_RETRIES',
    '#define HELLO_INTERVAL',
    '#define MAX_HOP_COUNT',
    '#define PKT_TYPE_FATIGUE_IMU',
    '#define PKT_TYPE_FATIGUE_STATUS',
    '#define PKT_TYPE_VEHICLE_TELEMETRY'
)

$aodvPatterns = @(
    'class AODVRouting',
    'void begin();',
    'void update();',
    'bool hasRouteTo',
    'uint8_t getNextHop',
    'void initiateRouteDiscovery',
    'void handleRREQ',
    'void handleRREP',
    'void handleRERR',
    'void handleHello'
)

$sketchPatterns = @(
    'aodv.begin\(\)',
    'aodv.update\(\)',
    'aodv.handleRREQ',
    'aodv.handleRREP',
    'aodv.handleRERR',
    'aodv.handleHello',
    'aodv.initiateRouteDiscovery',
    'nextHop == NODE_ID',
    'aodv.getNextHop',
    'hopCount',
    'PKT_TYPE_FATIGUE_IMU',
    'PKT_TYPE_FATIGUE_STATUS',
    'PKT_TYPE_VEHICLE_TELEMETRY',
    'PKT_TYPE_TIMESYNC|case 0x06'
)

$packetPatterns = @(
    'createTimeSyncPacket',
    'case 0x06: return "TIMESYNC"',
    'case 0x07: return "FATIGUE_IMU"',
    'case 0x08: return "FATIGUE_STATUS"',
    'case 0x09: return "VEHICLE_TELEMETRY"'
)

function Assert-Pattern {
    param(
        [string]$Content,
        [string]$Pattern,
        [string]$Label
    )

    if ($Content -notmatch $Pattern) {
        throw "Missing pattern [$Pattern] in $Label"
    }
}

Write-Host "=== Verifikasi LoRa Mesh Ad Hoc ===" -ForegroundColor Cyan

foreach ($node in $nodeChecks) {
    $nodeDir = Join-Path $PSScriptRoot $node.Name
    $sketchPath = Join-Path $nodeDir $node.Sketch
    $configPath = Join-Path $nodeDir "config.h"
    $aodvPath = Join-Path $nodeDir "AODV_Routing.h"
    $packetPath = Join-Path $nodeDir "LoRa_Packet.cpp"

    if (!(Test-Path $nodeDir)) {
        throw "Folder node tidak ditemukan: $nodeDir"
    }
    if (!(Test-Path $sketchPath)) {
        throw "Sketch node tidak ditemukan: $sketchPath"
    }
    if (!(Test-Path $configPath)) {
        throw "config.h tidak ditemukan: $configPath"
    }
    if (!(Test-Path $aodvPath)) {
        throw "AODV_Routing.h tidak ditemukan: $aodvPath"
    }
    if (!(Test-Path $packetPath)) {
        throw "LoRa_Packet.cpp tidak ditemukan: $packetPath"
    }

    $configContent = Get-Content -Path $configPath -Raw
    $aodvContent = Get-Content -Path $aodvPath -Raw
    $sketchContent = Get-Content -Path $sketchPath -Raw
    $packetContent = Get-Content -Path $packetPath -Raw

    foreach ($pattern in $configPatterns) {
        Assert-Pattern -Content $configContent -Pattern ([regex]::Escape($pattern)) -Label "$($node.Name)\config.h"
    }

    foreach ($pattern in $aodvPatterns) {
        Assert-Pattern -Content $aodvContent -Pattern ([regex]::Escape($pattern)) -Label "$($node.Name)\AODV_Routing.h"
    }

    foreach ($pattern in $sketchPatterns) {
        Assert-Pattern -Content $sketchContent -Pattern $pattern -Label "$($node.Name)\$($node.Sketch)"
    }

    foreach ($pattern in $packetPatterns) {
        Assert-Pattern -Content $packetContent -Pattern ([regex]::Escape($pattern)) -Label "$($node.Name)\LoRa_Packet.cpp"
    }

    if ($node.RequireFatigueStatusHandler) {
        Assert-Pattern -Content $sketchContent -Pattern 'applyAlarmStatusPayload' -Label "$($node.Name)\$($node.Sketch)"
        Assert-Pattern -Content $sketchContent -Pattern 'createFatigueImuPacket' -Label "$($node.Name)\$($node.Sketch)"
    }

    if ($node.RequireVehicleTelemetrySender) {
        Assert-Pattern -Content $sketchContent -Pattern 'sendVehicleTelemetry' -Label "$($node.Name)\$($node.Sketch)"
        Assert-Pattern -Content $sketchContent -Pattern 'createVehicleTelemetryPacket' -Label "$($node.Name)\$($node.Sketch)"
    }

    Write-Host "[OK] $($node.Name) memenuhi pola mesh ad hoc AODV" -ForegroundColor Green
}

Write-Host "`nSemua node lolos verifikasi LoRa mesh ad hoc." -ForegroundColor Cyan
