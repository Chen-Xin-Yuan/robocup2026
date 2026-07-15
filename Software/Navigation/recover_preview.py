import os

NAV_DIR = "D:/1_in_common_use/Software/MCU/HALcode/robotcup/demo/demo/Software/navigation"

def recover_file(filename):
    filepath = os.path.join(NAV_DIR, filename)
    with open(filepath, 'rb') as f:
        raw = f.read()

    # Remove BOM if present
    if raw.startswith(b'\xef\xbb\xbf'):
        raw = raw[3:]
        has_bom = True
    else:
        has_bom = False

    # Decode as UTF-8 (the current encoding)
    text = raw.decode('utf-8', errors='replace')

    result_bytes = bytearray()
    unrecoverable_positions = []
    for i, ch in enumerate(text):
        cp = ord(ch)
        if cp == 0xFFFD:
            # Replacement character - original bytes lost
            result_bytes.extend(b'?')
            unrecoverable_positions.append(i)
        elif cp <= 0x7F:
            # ASCII - direct byte, compatible with GBK
            result_bytes.append(cp)
        elif 0x80 <= cp <= 0xFF:
            # These were originally GBK bytes misinterpreted as Latin-1
            result_bytes.append(cp)
        else:
            # Proper Unicode character (e.g. correctly encoded Chinese)
            # Convert to GBK
            try:
                result_bytes.extend(ch.encode('gbk'))
            except UnicodeEncodeError:
                result_bytes.extend(b'?')
                unrecoverable_positions.append(i)

    # Now decode the recovered bytes as GBK to verify
    try:
        recovered_text = result_bytes.decode('gbk', errors='replace')
    except Exception as e:
        recovered_text = result_bytes.decode('gbk', errors='ignore')

    return recovered_text, unrecoverable_positions, has_bom, bytes(result_bytes)

files = ['flash.c', 'flash.h', 'kalman.c', 'kalman.h', 'navigation.c', 'navigation.h']

for fname in files:
    text, unrec, bom, raw_gbk = recover_file(fname)
    out_path = os.path.join(NAV_DIR, f"recover_preview_{fname}.txt")
    with open(out_path, 'w', encoding='utf-8') as f:
        f.write(f"FILE: {fname}\n")
        f.write(f"Unrecoverable chars: {len(unrec)}\n")
        f.write(f"BOM: {bom}\n")
        f.write('='*60 + '\n')
        f.write(text)
    print(f"Written preview for {fname} ({len(unrec)} unrecoverable)")
