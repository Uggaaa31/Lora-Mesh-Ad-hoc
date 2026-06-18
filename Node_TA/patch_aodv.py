import os
import glob
import re

node_dir = r'D:\Skripsi\Tugas Akhir_Lora_Gabungan_parameter_Retry\Node_TA'

pattern = re.compile(r'pendingDataAck\s*=\s*false;\s*return;\s*\}')

replacement = r'''pendingDataAck = false;
        
        // FIX: Invalidasi rute jika gagal mendapat ACK agar Route Recovery berjalan!
        Serial.println("[AODV] Link terputus (ACK Timeout)! Menghapus rute lama...");
        aodv.invalidateRoute(GATEWAY_ID);
        
        return;
    }'''

for root, dirs, files in os.walk(node_dir):
    for file in files:
        if file.endswith('.ino'):
            filepath = os.path.join(root, file)
            with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
                content = f.read()
            
            if 'aodv.invalidateRoute(GATEWAY_ID)' in content and 'lora_nailah_dummy' not in filepath:
                print(f'Already patched: {filepath}')
                continue
                
            new_content, count = pattern.subn(replacement, content)
            
            if count > 0:
                with open(filepath, 'w', encoding='utf-8') as f:
                    f.write(new_content)
                print(f'Patched {count} locations in: {filepath}')
            else:
                print(f'No match found in: {filepath}')
