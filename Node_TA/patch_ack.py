import os
import glob
import re

node_dir = r'D:\Skripsi\Tugas Akhir_Lora_Gabungan_parameter_Retry\Node_TA'

pattern = re.compile(r'uint32_t getDataAckTimeoutMs\(\) \{\s*switch \(runtimeCfg\.sf\) \{\s*case 7:\s*return DATA_ACK_TIMEOUT_SF7_MS;\s*case 9:\s*return DATA_ACK_TIMEOUT_SF9_MS;\s*case 12:\s*return DATA_ACK_TIMEOUT_SF12_MS;\s*default:\s*return DATA_ACK_TIMEOUT_MS;\s*\}\s*\}', re.MULTILINE)

replacement = r'''uint32_t getDataAckTimeoutMs() {
    uint32_t baseMs;
    switch (runtimeCfg.sf) {
        case 7:  baseMs = DATA_ACK_TIMEOUT_SF7_MS; break;
        case 9:  baseMs = DATA_ACK_TIMEOUT_SF9_MS; break;
        case 12: baseMs = DATA_ACK_TIMEOUT_SF12_MS; break;
        default: baseMs = DATA_ACK_TIMEOUT_MS; break;
    }
    uint8_t hops = aodv.getRouteHopCount(GATEWAY_ID);
    if (hops == 0) hops = 1;
    return (baseMs * hops) + ((hops - 1) * 500);
}'''

for root, dirs, files in os.walk(node_dir):
    for file in files:
        if file.endswith('.ino'):
            filepath = os.path.join(root, file)
            with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
                content = f.read()
                
            new_content, count = pattern.subn(replacement, content)
            
            if count > 0:
                with open(filepath, 'w', encoding='utf-8') as f:
                    f.write(new_content)
                print(f'Patched: {filepath}')
