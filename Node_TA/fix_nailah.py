
import sys

with open(r"D:\Skripsi\Tugas Akhir_Lora_Gabungan_parameter_Retry\Node_TA\lora_nailah_dummy\lora_nailah_dummy.ino", "r", encoding="utf-8") as f:
    code = f.read()

init_bno = """bool initBNO055() {
    Serial.println("[IMU] DUMMY BNO055 initialized.");
    return true;
}"""

init_gps = """void initGPS() {
    Serial.println("[GPS] DUMMY GPS initialized.");
}"""

update_tel = """void updateTelemetry(unsigned long nowMs) {
    float dt = (nowMs - lastMillis) / 1000.0f;
    if (dt <= 0.0f || dt > 1.0f) {
        lastMillis = nowMs;
        return;
    }
    lastMillis = nowMs;
    lastDt = dt;

    headingDeg = 45.0f; 
    float linAx = 0.5f;
    float linAy = 0.1f;
    float linAz = 9.8f;
    lastAx = linAx; lastAy = linAy; lastAz = linAz;
    lastAccelForward = 0.5f;

    bool isGpsValid = true;
    float hdopValue = 1.0f;
    uint16_t satsValue = 8;
    float gpsSpeedOut = 15.0f;
    float gpsHeadingOut = 45.0f;

    drActive = false;
    lat = -5.1476 + (random(-10, 10) / 100000.0);
    lon = 119.4320 + (random(-10, 10) / 100000.0);
    hasInitialGpsFix = true;
    speedMps = gpsSpeedOut;

    lastMx = 20.0f; lastMy = -10.0f; lastMz = 40.0f;
    calSys = 3; calGyro = 3; calAccel = 3; calMag = 3;

    pitchDeg = 5.0f; rollDeg = -2.0f; rawYawDeg = headingDeg;

    isRollover = 0; isRolloverRisk = 0;
    digitalWrite(BUZZER_PIN, LOW);

    isHarshBraking = 0; isOverspeed = 0;

    lastGpsValid = isGpsValid;
    lastGpsSpeed = gpsSpeedOut;
    lastGpsHeading = gpsHeadingOut;
    lastHdop = hdopValue;
    lastSatellites = satsValue;
}"""

import re
code = re.sub(r"bool initBNO055\(\)\s*\{.*?\n\}", init_bno, code, flags=re.DOTALL)
code = re.sub(r"void initGPS\(\)\s*\{.*?\n\}", init_gps, code, flags=re.DOTALL)
code = re.sub(r"void updateTelemetry\(unsigned long nowMs\)\s*\{.*?\n\}(?=\n\nbool sendVehicleTelemetry)", update_tel, code, flags=re.DOTALL)

with open(r"D:\Skripsi\Tugas Akhir_Lora_Gabungan_parameter_Retry\Node_TA\lora_nailah_dummy\lora_nailah_dummy.ino", "w", encoding="utf-8") as f:
    f.write(code)
print("Done")

