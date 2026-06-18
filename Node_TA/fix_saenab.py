
import sys
import re

with open(r"D:\Skripsi\Tugas Akhir_Lora_Gabungan_parameter_Retry\Node_TA\lora_saenab_s3_dummy\lora_saenab_s3_dummy.ino", "r", encoding="utf-8") as f:
    code = f.read()

dummy_read = """static void read_bno055_and_filter(unsigned long nowMs) {
    pitch = 5.0f;
    roll  = -2.0f;
    
    float dummy_gx = 0.1f;
    float dummy_gy = 0.2f;
    float dummy_gz = 0.3f;
    
    if (!gyroInit) {
        f_gx = dummy_gx; f_gy = dummy_gy; f_gz = dummy_gz;
        gyroInit = true;
    } else {
        f_gx = alpha * dummy_gx + (1.0f - alpha) * f_gx;
        f_gy = alpha * dummy_gy + (1.0f - alpha) * f_gy;
        f_gz = alpha * dummy_gz + (1.0f - alpha) * f_gz;
    }
    cal_sys = 3; cal_gyro = 3; cal_acc = 3; cal_mag = 3;
}"""

# Replace read function
code = re.sub(r"static void read_bno055_and_filter\(unsigned long nowMs\)\s*\{.*?\n\}", dummy_read, code, flags=re.DOTALL)

# Replace BNO055 Init in setup()
bno_init_orig = """    // BNO055 Init
    Wire.begin(I2C_SDA, I2C_SCL);
    if (!bno.begin()) Serial.println("? BNO055 Fail");
    else bno.setExtCrystalUse(true);"""

bno_init_dummy = """    // BNO055 Init (DUMMY)
    Serial.println("[IMU] DUMMY BNO055 initialized.");"""

code = code.replace(bno_init_orig, bno_init_dummy)

with open(r"D:\Skripsi\Tugas Akhir_Lora_Gabungan_parameter_Retry\Node_TA\lora_saenab_s3_dummy\lora_saenab_s3_dummy.ino", "w", encoding="utf-8") as f:
    f.write(code)
print("Done")

