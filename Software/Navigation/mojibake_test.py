# Test classic mojibake patterns
patterns = [
    ("锟斤拷", "gbk"),
    ("閿熸枻鎷�", "gbk"),
]

with open('D:/1_in_common_use/Software/MCU/HALcode/robotcup/demo/demo/Software/navigation/mojibake_test.txt', 'w', encoding='utf-8') as f:
    for text, enc in patterns:
        utf8_bytes = text.encode('utf-8')
        f.write(f"Text: {text}\n")
        f.write(f"UTF-8 bytes: {utf8_bytes.hex()}\n")
        f.write(f"Length: {len(utf8_bytes)} bytes\n")

        # Decode bytes as GBK
        try:
            gbk_decoded = utf8_bytes.decode('gbk')
            f.write(f"As GBK: {gbk_decoded}\n")
            f.write(f"GBK decoded hex: {gbk_decoded.encode('gbk').hex()}\n")
        except Exception as e:
            f.write(f"GBK decode error: {e}\n")
            try:
                gbk_decoded = utf8_bytes.decode('gbk', errors='replace')
                f.write(f"GBK decode (replace): {gbk_decoded}\n")
            except Exception as e2:
                f.write(f"GBK replace error: {e2}\n")

        f.write("\n")

    # Now test the reverse: what original UTF-8 bytes would produce 锟斤拷
    # when decoded as GBK then re-encoded as UTF-8?
    f.write("=== Reverse engineering 锟斤拷 ===\n")
    # 锟斤拷 as GBK bytes
    gbk_bytes = "锟斤拷".encode('gbk')
    f.write(f"锟斤拷 as GBK bytes: {gbk_bytes.hex()}\n")
    # Now decode these GBK bytes as UTF-8
    try:
        utf8_decoded = gbk_bytes.decode('utf-8')
        f.write(f"GBK bytes as UTF-8: {utf8_decoded}\n")
    except Exception as e:
        f.write(f"UTF-8 decode error: {e}\n")
        utf8_decoded = gbk_bytes.decode('utf-8', errors='replace')
        f.write(f"GBK bytes as UTF-8 (replace): {utf8_decoded}\n")

    f.write("\n")

    # Test: what if original text was "年" in UTF-8, opened as GBK, then saved as UTF-8?
    f.write("=== 年 corruption chain ===\n")
    original = "年"
    utf8_b = original.encode('utf-8')
    f.write(f"Original: {original}, UTF-8: {utf8_b.hex()}\n")

    # Open as GBK (with replace)
    gbk_view = utf8_b.decode('gbk', errors='replace')
    f.write(f"UTF-8 bytes as GBK: {repr(gbk_view)}\n")

    # Save as UTF-8
    final_b = gbk_view.encode('utf-8')
    f.write(f"Saved as UTF-8: {final_b.hex()}\n")
    f.write(f"Final text: {final_b.decode('utf-8')}\n")

    f.write("\n")

    # Try with a full string
    f.write("=== Full string corruption ===\n")
    orig_str = "定位姿态"
    utf8_b2 = orig_str.encode('utf-8')
    f.write(f"Original: {orig_str}, UTF-8: {utf8_b2.hex()}\n")
    gbk_view2 = utf8_b2.decode('gbk', errors='replace')
    f.write(f"As GBK: {repr(gbk_view2)}\n")
    final_b2 = gbk_view2.encode('utf-8')
    f.write(f"Saved as UTF-8: {final_b2.hex()}\n")
    f.write(f"Final text: {final_b2.decode('utf-8')}\n")

    # Recovery: decode final as UTF-8, encode as GBK bytes, decode as UTF-8
    f.write("\nRecovery attempt:\n")
    recovered = final_b2.decode('utf-8').encode('gbk', errors='replace')
    f.write(f"Recovered bytes: {recovered.hex()}\n")
    f.write(f"Recovered as UTF-8: {recovered.decode('utf-8', errors='replace')}\n")
