path = 'D:/1_in_common_use/Software/MCU/HALcode/robotcup/demo/demo/Software/navigation/navigation.h'
with open(path, 'rb') as f:
    data = f.read()
# Remove BOM
if data.startswith(b'\xef\xbb\xbf'):
    data = data[3:]

out_path = 'D:/1_in_common_use/Software/MCU/HALcode/robotcup/demo/demo/Software/navigation/nav_h_gbk_decode.txt'

# Try decode entire file as GBK
with open(out_path, 'w', encoding='utf-8') as out:
    try:
        gbk_text = data.decode('gbk')
        out.write('Full GBK decode succeeded\n')
        out.write(f'Length: {len(gbk_text)}\n\n')
        out.write(gbk_text)
    except Exception as e:
        out.write(f'Full GBK decode error: {e}\n')
        gbk_text = data.decode('gbk', errors='replace')
        out.write(f'GBK decode with replace, length: {len(gbk_text)}\n\n')
        out.write(gbk_text)
