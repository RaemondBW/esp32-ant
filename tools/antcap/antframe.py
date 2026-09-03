"""
antframe.py - the ANT / ShockBurst on-air frame, in Python.

A byte-exact mirror of components/ant/src/ant_phy_shockburst.c and
ant_sb_link.c so the SDR tooling can build the frames the ESP32 firmware
builds and recognise them in a capture. `make check` in this directory
compiles the C and diffs it against this file, so the two cannot drift.

Frame (MSB-first on air, no whitening):

    preamble(1) | address(3..6) | payload | crc16(2, big-endian)

    preamble  0xAA if address[0] bit7 set, else 0x55
    crc       CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over address+payload

The real ANT+ link (ant_sb_link, verified against a commercial HRM strap
with the HackRF and on the ESP32-S3): 6-byte address

    marker(2) | devnum_hi | devnum_lo | devtype | trans_type
    marker = f(network key) as computed by the nRF52 ANT SoftDevice
             (ANT+ key -> a6 c5, ANT-FS -> 3b a3, public -> 5b 25)

and a 9-byte payload: flags/ctrl byte (0x0A for a broadcast) + 8 data
bytes (the ANT+ page).
"""

ADDR_MIN, ADDR_MAX = 3, 6
PAYLOAD_MAX = 32
LINK_MARKER_LEN = 2
LINK_ADDR_LEN = 6
LINK_DATA_LEN = 8
LINK_PAYLOAD_LEN = 1 + LINK_DATA_LEN
LINK_FRAME_LEN = 1 + LINK_ADDR_LEN + LINK_PAYLOAD_LEN + 2      # 18

ANTPLUS_FREQ_MHZ = 2457
# Keys in "Set Network Key" message order.
ANTPLUS_NETWORK_KEY = bytes([0xB9, 0xA5, 0x21, 0xFB, 0xBD, 0x72, 0xC3, 0x45])
PUBLIC_NETWORK_KEY = bytes([0xE8, 0xE4, 0x21, 0x3B, 0x55, 0x7A, 0x67, 0xC1])
ANTFS_NETWORK_KEY = bytes([0xA8, 0xA4, 0x23, 0xB9, 0xF5, 0x5E, 0x63, 0xC1])
ANTPLUS_DEVTYPE_HRM = 120
ANTPLUS_DEVTYPE_POWER = 11
ANTPLUS_PERIOD_HRM = 8070
ANTPLUS_PERIOD_POWER = 8182
TICKS_PER_SEC = 32768

CTRL_TYPE_MASK, CTRL_BROADCAST, CTRL_ACK, CTRL_ACK_RESP, CTRL_BURST = 0xC0, 0x00, 0x40, 0x80, 0xC0
CTRL_REVERSE, CTRL_LAST, CTRL_SEQ_MASK, CTRL_TAG = 0x20, 0x10, 0x0F, 0x0A
CTRL_TYPE_NAMES = {CTRL_BROADCAST: "BROADCAST", CTRL_ACK: "ACK", CTRL_ACK_RESP: "ACK_RESP", CTRL_BURST: "BURST"}

# nRF52 ANT SoftDevice key check / marker tables (ESPwn32 ant_network_keys).
_VALID_AND = (0xec, 0x3f, 0xd7, 0xdb, 0x79, 0xf7, 0xbe, 0xef)
_VALID_XOR = (0x20, 0x1a, 0x47, 0x11, 0x50, 0x93, 0x36, 0x8f)
_MARK_KEYSEL = (0xfe, 0xff, 0x1c, 0x7c, 0xfc, 0x0c, 0x04, 0x3c)
_MARK_AND = (0x41, 0x10, 0x28, 0x86, 0x08, 0xc0, 0x13, 0x24)


def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT-FALSE, identical to ant_crc16_ccitt (check: 'A'*? -> pinned 0x29B1 for '123456789')."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def preamble_for(first_address_byte: int) -> int:
    return 0xAA if first_address_byte & 0x80 else 0x55


def network_key_valid(key: bytes) -> bool:
    """ant_sb_network_key_valid: would the SoftDevice accept this key?"""
    order = [key[2], key[3], key[4], key[5], key[6], key[7], key[1], key[0]]
    t = 0
    for i in range(8):
        t ^= order[i]
        if (t & _VALID_AND[i]) ^ _VALID_XOR[i]:
            return False
    return True


def network_marker(key: bytes = PUBLIC_NETWORK_KEY) -> bytes:
    """ant_sb_link_network_marker: the 2 bytes that lead the on-air address."""
    if not any(key):
        key = PUBLIC_NETWORK_KEY                       # ANT network 0 default
    if not network_key_valid(key):
        f = 0
        for b in key:
            f ^= b
        return bytes([0x80 | ((0xA6 ^ f) & 0x7F), (0xC5 ^ key[0] ^ key[7]) & 0xFF])
    lo = hi = 0
    for i in range(8):
        t = 0
        for j in range(8):
            if _MARK_KEYSEL[i] & (1 << j):
                t ^= key[j]
        t &= _MARK_AND[i]
        if i < 4:
            lo |= t
        else:
            hi |= t
    return bytes([lo, hi])


def link_address(device_num: int, device_type: int, trans_type: int,
                 key: bytes = ANTPLUS_NETWORK_KEY, addr_len: int = LINK_ADDR_LEN) -> bytes:
    full = network_marker(key) + bytes([(device_num >> 8) & 0xFF, device_num & 0xFF,
                                        device_type & 0xFF, trans_type & 0xFF])
    return full[:addr_len]


def build_frame(address: bytes, payload: bytes) -> bytes:
    """preamble | address | payload | crc  (= ant_sb_build_frame)."""
    if not ADDR_MIN <= len(address) <= ADDR_MAX or len(payload) > PAYLOAD_MAX:
        raise ValueError("bad address/payload length")
    body = bytes(address) + bytes(payload)
    crc = crc16_ccitt(body)
    return bytes([preamble_for(address[0])]) + body + bytes([crc >> 8, crc & 0xFF])


def verify_body(body: bytes) -> bool:
    """body = address | payload | crc (no preamble), as ant_sb_verify_frame."""
    if len(body) < ADDR_MIN + 2:
        return False
    return crc16_ccitt(body[:-2]) == (body[-2] << 8 | body[-1])


def link_ctrl(ctrl: int) -> int:
    """ant_sb_link_build's low-nibble rule: bursts keep their sequence, everything else carries 0x0A."""
    if ctrl & CTRL_TYPE_MASK == CTRL_BURST:
        return ctrl
    return (ctrl & ~CTRL_SEQ_MASK) | CTRL_TAG


def link_frame(device_num: int, device_type: int, trans_type: int, data: bytes,
               ctrl: int = CTRL_BROADCAST, key: bytes = ANTPLUS_NETWORK_KEY) -> bytes:
    """The exact 18-byte frame ant_mac transmits for one broadcast/ack/burst packet."""
    if len(data) != LINK_DATA_LEN:
        raise ValueError("data must be 8 bytes")
    return build_frame(link_address(device_num, device_type, trans_type, key),
                       bytes([link_ctrl(ctrl)]) + bytes(data))


def describe_ctrl(ctrl: int) -> str:
    s = CTRL_TYPE_NAMES[ctrl & CTRL_TYPE_MASK]
    if ctrl & CTRL_REVERSE:
        s += "|REV"
    if ctrl & CTRL_TYPE_MASK == CTRL_BURST:
        s += f"|seq{ctrl & CTRL_SEQ_MASK}"
        if ctrl & CTRL_LAST:
            s += "|LAST"
    return s


def describe_hrm_page(page: bytes) -> str:
    """ANT+ HRM common trailer: beat time (1/1024 s), beat count, computed HR."""
    if len(page) != 8:
        return ""
    n = page[0] & 0x7F
    t = page[4] | page[5] << 8
    return f"HRM page{n} toggle={page[0] >> 7} beat_time={t} beats={page[6]} hr={page[7]} bpm"


def hrm_page0(hr: int, beats: int, event_time: int, toggle: bool = False) -> bytes:
    """antplus_hrm_encode_page0."""
    return bytes([0x80 if toggle else 0x00, 0xFF, 0xFF, 0xFF,
                  event_time & 0xFF, (event_time >> 8) & 0xFF, beats & 0xFF, hr & 0xFF])


def hexs(b: bytes) -> str:
    return " ".join(f"{x:02x}" for x in b)


if __name__ == "__main__":
    # `python3 antframe.py` prints the reference vectors `make check` compares against C.
    assert crc16_ccitt(b"123456789") == 0x29B1
    assert network_marker(ANTPLUS_NETWORK_KEY) == bytes([0xa6, 0xc5])
    key = ANTPLUS_NETWORK_KEY
    page = hrm_page0(72, 1, 1024)
    print("crc123456789", f"{crc16_ccitt(b'123456789'):04x}")
    print("marker_public", hexs(network_marker(PUBLIC_NETWORK_KEY)))
    print("marker_zero", hexs(network_marker(bytes(8))))
    print("marker_antplus", hexs(network_marker(key)))
    print("marker_antfs", hexs(network_marker(ANTFS_NETWORK_KEY)))
    print("marker_junk", hexs(network_marker(bytes(range(1, 9)))))
    print("valid", "".join("1" if network_key_valid(k) else "0"
                           for k in (ANTPLUS_NETWORK_KEY, PUBLIC_NETWORK_KEY, ANTFS_NETWORK_KEY,
                                     bytes(8), bytes(range(1, 9)))))
    print("frame_public", hexs(link_frame(0x1234, ANTPLUS_DEVTYPE_HRM, 1, page, key=PUBLIC_NETWORK_KEY)))
    print("frame_antplus", hexs(link_frame(0x1234, ANTPLUS_DEVTYPE_HRM, 1, page, key=key)))
    print("frame_ack", hexs(link_frame(0x1234, ANTPLUS_DEVTYPE_HRM, 1, page, ctrl=CTRL_ACK | CTRL_REVERSE, key=key)))
    print("frame_burst", hexs(link_frame(0x3042, ANTPLUS_DEVTYPE_HRM, 1, bytes(range(8)),
                                         ctrl=CTRL_BURST | CTRL_REVERSE | CTRL_LAST | 3, key=key)))
    # the strap frame the S3 received off the air (device 0x6941, 67 bpm)
    strap = bytes.fromhex("a6c5694178010a00ffffff9a47c743bfd2")
    assert verify_body(strap) and link_frame(0x6941, 0x78, 1, strap[7:15]) == bytes([0xAA]) + strap
    print("frame_strap", hexs(link_frame(0x6941, 0x78, 1, strap[7:15])))
    print("frame_addr3", hexs(build_frame(bytes([0xE7, 0xE7, 0xE7]), bytes([0x40]) + bytes(range(8)))))
