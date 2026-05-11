# setup_nodes.ps1
# Script untuk menyalin file library ke setiap folder node
# Jalankan dari folder: d:\Tugas Akhir_Lora\Node_TA\
# Cara: klik kanan di folder lalu "Run with PowerShell"

$sourceDir = "LoRa_Mesh_Node"
$libraryFiles = @(
    "AODV_Routing.cpp",
    "AODV_Routing.h",
    "LoRa_Packet.cpp",
    "LoRa_Packet.h",
    "config.h",
    "WebConfig.h"     # Web Config UI (WiFi AP mode)
)
$nodeFolders = @(
    "LoRa_Node_TRK002",
    "LoRa_Node_TRK003"
)

Write-Host "=== Setup Node Library Files ===" -ForegroundColor Cyan
foreach ($folder in $nodeFolders) {
    if (!(Test-Path $folder)) {
        Write-Host "SKIP: Folder $folder tidak ditemukan" -ForegroundColor Yellow
        continue
    }
    foreach ($file in $libraryFiles) {
        $src  = Join-Path $sourceDir $file
        $dest = Join-Path $folder $file
        if (Test-Path $src) {
            Copy-Item $src $dest -Force
            Write-Host "  [OK] $folder\$file" -ForegroundColor Green
        } else {
            Write-Host "  [SKIP] $src tidak ada" -ForegroundColor Red
        }
    }
    Write-Host "Folder $folder selesai." -ForegroundColor Cyan
}

Write-Host "`nSetup selesai! Semua folder node siap di-compile di Arduino IDE." -ForegroundColor Green
Write-Host "Ingat: Ubah SF/BW di config.h lalu jalankan script ini lagi untuk update semua node." -ForegroundColor Yellow
Write-Host "Catatan: folder custom lora_saenab dan lora_nailah tidak ikut disinkronkan otomatis oleh script ini." -ForegroundColor Yellow
