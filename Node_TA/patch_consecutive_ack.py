import os
import glob
import re

node_dir = r'D:\Skripsi\Tugas Akhir_Lora_Gabungan_parameter_Retry\Node_TA'

for root, dirs, files in os.walk(node_dir):
    for file in files:
        if file.endswith('.ino'):
            filepath = os.path.join(root, file)
            with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
                content = f.read()

            # 1. Add consecutiveAckFailures
            if 'static uint8_t consecutiveAckFailures = 0;' not in content:
                content = content.replace(
                    'static uint8_t pendingDataRetries = 0;',
                    'static uint8_t pendingDataRetries = 0;\nstatic uint8_t consecutiveAckFailures = 0;'
                )
            
            # 2. Update processDataAckTimeout
            old_timeout = '''        // FIX: Invalidasi rute jika gagal mendapat ACK agar Route Recovery berjalan!
        Serial.println("[AODV] Link terputus (ACK Timeout)! Menghapus rute lama...");
        aodv.invalidateRoute(GATEWAY_ID);'''
            
            new_timeout = '''        consecutiveAckFailures++;
        if (consecutiveAckFailures >= 5) {
            Serial.println("[AODV] Link terputus (3x ACK Timeout berturut-turut)! Menghapus rute lama...");
            aodv.invalidateRoute(GATEWAY_ID);
            consecutiveAckFailures = 0;
        } else {
            Serial.printf("[AODV] Paket gagal, tapi rute dipertahankan (%d/3 kegagalan)\\n", consecutiveAckFailures);
        }'''
            
            if old_timeout in content:
                content = content.replace(old_timeout, new_timeout)
            
            # 3. Update ACK receive logic
            old_ack_rx = '''            pendingDataAck = false;
            Serial.printf("[ACK] received seq=%lu retries=%u\\n",'''
            
            new_ack_rx = '''            pendingDataAck = false;
            consecutiveAckFailures = 0;
            Serial.printf("[ACK] received seq=%lu retries=%u\\n",'''
            
            if old_ack_rx in content:
                content = content.replace(old_ack_rx, new_ack_rx)
            
            with open(filepath, 'w', encoding='utf-8') as f:
                f.write(content)
            print(f'Patched: {filepath}')
