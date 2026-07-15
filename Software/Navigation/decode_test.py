import os

NAV_DIR = "D:/1_in_common_use/Software/MCU/HALcode/robotcup/demo/demo/Software/navigation"

def test_decode(data):
    results = []
    for enc in ['gbk', 'gb2312', 'big5', 'shift_jis', 'euc-jp', 'euc-kr', 'latin-1', 'cp1252']:
        try:
            text = data.decode(enc)
            results.append((enc, text))
        except Exception as e:
            results.append((enc, f"ERROR: {e}"))
    return results

# Test the hex from navigation.h
hex_str = "e98eafee889aee87b1e6b5a3e5b685d0ad"
data = bytes.fromhex(hex_str)

with open(os.path.join(NAV_DIR, "decode_test.txt"), 'w', encoding='utf-8') as f:
    f.write(f"Testing bytes: {hex_str}\n\n")
    for enc, text in test_decode(data):
        f.write(f"{enc}: {repr(text)}\n")

    # Also test individual 2-byte chunks as GBK
    f.write("\n=== GBK 2-byte chunks ===\n")
    for i in range(0, len(data)-1, 2):
        chunk = data[i:i+2]
        try:
            f.write(f"{chunk.hex()} -> {chunk.decode('gbk')}\n")
        except Exception as e:
            f.write(f"{chunk.hex()} -> ERROR: {e}\n")

    # Test individual 2-byte chunks, but allow overlap for single-byte chars
    f.write("\n=== GBK variable-length chunks ===\n")
    i = 0
    while i < len(data):
        if data[i] >= 0x81:
            chunk = data[i:i+2]
            try:
                f.write(f"{chunk.hex()} -> {chunk.decode('gbk')}\n")
                i += 2
            except:
                f.write(f"{data[i:i+1].hex()} -> {data[i:i+1].decode('latin-1')}\n")
                i += 1
        else:
            f.write(f"{data[i:i+1].hex()} -> {data[i:i+1].decode('latin-1')}\n")
            i += 1

    # Now let's look at all of navigation.h and try to recover
    f.write("\n\n=== Full navigation.h recovery attempts ===\n")
    with open(os.path.join(NAV_DIR, "navigation.h"), 'rb') as fin:
        nav_data = fin.read()

    # Remove BOM
    if nav_data.startswith(b'\xef\xbb\xbf'):
        nav_data = nav_data[3:]

    # Try decode as UTF-8
    utf8_text = nav_data.decode('utf-8', errors='replace')
    f.write(f"UTF-8 decode (replace): {repr(utf8_text[:200])}\n\n")

    # Count non-ASCII chars
    non_ascii = [c for c in utf8_text if ord(c) > 0x7f]
    f.write(f"Non-ASCII chars: {len(non_ascii)}\n")
    f.write(f"Unique non-ASCII: {sorted(set(non_ascii))}\n\n")

    # Try the mixed approach: if char is in U+0080-U+00FF, treat as GBK byte
    # Otherwise treat as Unicode and encode to GBK
    mixed_bytes = bytearray()
    for ch in utf8_text:
        cp = ord(ch)
        if cp == 0xFFFD:
            mixed_bytes.extend(b'?')
        elif cp <= 0x7F:
            mixed_bytes.append(cp)
        elif 0x80 <= cp <= 0xFF:
            mixed_bytes.append(cp)
        else:
            try:
                mixed_bytes.extend(ch.encode('gbk'))
            except:
                mixed_bytes.extend(b'?')

    f.write(f"Mixed approach decode as GBK:\n")
    try:
        mixed_text = mixed_bytes.decode('gbk', errors='replace')
        f.write(mixed_text[:500])
    except Exception as e:
        f.write(f"ERROR: {e}\n")

    # Another approach: the file might be valid UTF-8 but with some characters
    # that are actually UTF-8 bytes of Chinese characters that were decoded as GBK
    # and then saved. For example, 锟斤拷 = e9949fe696a4e68bb7
    # These 9 bytes might correspond to 3 original UTF-8 bytes that were decoded as GBK.
    # But since 锟斤拷 decodes to 3 chars, and GBK is 2 bytes per char,
    # 9 bytes = 4.5 chars which doesn't make sense.

    # Actually let's test: what if the original was GBK and got decoded as UTF-8,
    # and the invalid sequences were replaced with U+FFFD, but then some valid
    # UTF-8 sequences happened to match GBK bytes?

    # The most likely scenario for navigation.h:
    # Original was UTF-8 Chinese text.
    # Opened as GBK -> some bytes form valid GBK chars, some are invalid.
    # Saved as UTF-8 with replacement chars.
    # But wait, we don't see many U+FFFD in navigation.h (only 15).

    # Let's check if the non-ASCII chars in UTF-8 decode are mostly CJK
    cjk_chars = [c for c in non_ascii if '一' <= c <= '鿿']
    f.write(f"\nCJK chars count: {len(cjk_chars)}\n")
    f.write(f"CJK chars: {''.join(cjk_chars[:50])}\n")
