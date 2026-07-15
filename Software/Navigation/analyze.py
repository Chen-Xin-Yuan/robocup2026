import os

NAV_DIR = "D:/1_in_common_use/Software/MCU/HALcode/robotcup/demo/demo/Software/navigation"

def analyze_file(filename):
    filepath = os.path.join(NAV_DIR, filename)
    with open(filepath, 'rb') as f:
        raw = f.read()

    # Count various patterns
    fffd_count = raw.count(b'\xef\xbf\xbd')

    # Check for classic double-encoding patterns (c2 xx, c3 xx, etc.)
    # These are Latin-1 chars 0x80-0xFF encoded as UTF-8
    double_enc = 0
    i = 0
    while i < len(raw) - 1:
        if 0xc2 <= raw[i] <= 0xdf and 0x80 <= raw[i+1] <= 0xbf:
            double_enc += 1
            i += 2
        else:
            i += 1

    # Check for classic UTF-8 as GBK corruption: 锟斤拷 = e9 94 9f e6 96 a4 e6 8b b7
    junk_count = raw.count(b'\xe9\x94\x9f\xe6\x96\xa4\xe6\x8b\xb7')

    with open(os.path.join(NAV_DIR, f"analyze_{filename}.txt"), 'w', encoding='utf-8') as out:
        out.write(f"File: {filename}\n")
        out.write(f"Size: {len(raw)} bytes\n")
        out.write(f"U+FFFD count: {fffd_count}\n")
        out.write(f"Double-encoded Latin-1 count: {double_enc}\n")
        out.write(f"'锟斤拷' count: {junk_count}\n")
        out.write(f"\nFirst 300 bytes hex:\n")
        out.write(raw[:300].hex())
        out.write("\n\n")

files = ['flash.c', 'flash.h', 'kalman.c', 'kalman.h', 'navigation.c', 'navigation.h']
for fname in files:
    analyze_file(fname)

# Experiment with corruption reversal
with open(os.path.join(NAV_DIR, "analyze_experiments.txt"), 'w', encoding='utf-8') as out:
    out.write("=== Experiment: 锟斤拷 reversal ===\n")
    # The common pattern: UTF-8 text is opened as GBK, then saved as UTF-8
    # For example, if original text was '年' (UTF-8: e5 b9 b4)
    # Step 1: e5b9b4 decoded as GBK fails at b4 (incomplete sequence)
    # So it's not that simple.

    # Another common pattern: file saved as UTF-8, then opened as GBK,
    # then the user edits and saves as GBK. Later opened as UTF-8 again.
    # In this case, the GBK file contains valid GBK bytes that decode to
    # Chinese characters. When opened as UTF-8, those GBK bytes become
    # mojibake (garbled chars). If the user then saves that as UTF-8,
    # the garbled chars get UTF-8 encoded.

    # Let's test: take some Chinese text, encode as GBK, then decode as UTF-8 with errors,
    # then encode as UTF-8...
    text = "世界坐标系"
    gbk_bytes = text.encode('gbk')
    out.write(f"Original: {text}\n")
    out.write(f"GBK bytes: {gbk_bytes.hex()}\n")

    # Decode GBK bytes as UTF-8 (with replace)
    utf8_garbled = gbk_bytes.decode('utf-8', errors='replace')
    out.write(f"GBK bytes as UTF-8: {repr(utf8_garbled)}\n")

    # Now encode that garbled text as UTF-8
    final_bytes = utf8_garbled.encode('utf-8')
    out.write(f"Garbled text re-encoded as UTF-8: {final_bytes.hex()}\n")

    # Now try to recover: decode as UTF-8, then encode each char as latin1 if it's in range
    recovered = utf8_garbled.encode('utf-8').decode('utf-8')
    out.write(f"This doesn't help because replacement chars destroy info.\n")

    out.write("\n=== Experiment: GBK -> Latin1 -> UTF-8 reversal ===\n")
    # Original GBK bytes decoded as Latin1, then saved as UTF-8
    text2 = "俯仰角"
    gbk_bytes2 = text2.encode('gbk')
    out.write(f"Original: {text2}\n")
    out.write(f"GBK bytes: {gbk_bytes2.hex()}\n")

    latin1_text = gbk_bytes2.decode('latin-1')
    out.write(f"GBK bytes as Latin1: {repr(latin1_text)}\n")

    utf8_bytes2 = latin1_text.encode('utf-8')
    out.write(f"Latin1 text as UTF-8: {utf8_bytes2.hex()}\n")

    # Recovery: decode UTF-8, then encode as latin1
    decoded = utf8_bytes2.decode('utf-8')
    recovered_bytes = decoded.encode('latin-1')
    recovered_text = recovered_bytes.decode('gbk')
    out.write(f"Recovered: {recovered_text}\n")

    out.write("\n=== Check navigation.h date line ===\n")
    # navigation.h line 4: 2024锟斤拷10锟斤拷16锟斤拷
    # Let's see what the original might have been
    s = "2024锟斤拷10锟斤拷16锟斤拷"
    b = s.encode('utf-8')
    out.write(f"Corrupted text: {s}\n")
    out.write(f"UTF-8 bytes: {b.hex()}\n")
    out.write(f"Length: {len(b)} bytes\n")
    # Try decoding the whole thing as GBK
    try:
        gbk_decoded = b.decode('gbk')
        out.write(f"As GBK: {repr(gbk_decoded)}\n")
    except Exception as e:
        out.write(f"GBK decode error: {e}\n")
        # Try decoding parts
        parts = b.split(b'\xe9\x94\x9f\xe6\x96\xa4\xe6\x8b\xb7')
        out.write(f"Split parts: {[p.hex() for p in parts]}\n")
        for p in parts:
            try:
                out.write(f"  {p.hex()} as GBK: {repr(p.decode('gbk'))}\n")
            except Exception as e2:
                out.write(f"  {p.hex()} GBK error: {e2}\n")
