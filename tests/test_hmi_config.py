"""Check the packed Delta HMI's register bindings against the firmware map."""
import gzip
from pathlib import Path
import struct


def main():
    project = Path(__file__).resolve().parents[1]
    packed = (project / 'MotorControlHMI.dpa').read_bytes()
    decoded = gzip.decompress(packed[packed.index(b'\x1f\x8b\x08'):])
    payload = bytes(value ^ 0x64 for value in decoded[41:])
    assert struct.unpack('<I', decoded[37:41])[0] == len(payload)
    assert b'DefPLCStationNr0=17\r\n' in payload
    assert b'baud0=9600\r\n' in payload
    for address, width in ((b'C8', b'2'), (b'CA', b'6')):
        binding = b'ReadVar={MotorController}17@R-' + address + b'\r\n'
        assert payload.count(binding) == 1
        widget = payload.split(binding)[1].split(b'[State]')[0]
        assert b'MemLen=' + width + b'\r\n' in widget
        assert b'MemFmt=1\r\n' in widget
        assert b'DotNum=3\r\n' in widget
        assert b'HiWordFirst=1\r\n' in widget
        if address == b'CA':
            assert b'HiDwordFirst=1\r\n' in widget
    for coil in range(4):
        assert f'WriteVar={{MotorController}}17@RWB-{coil}\r\n'.encode() in payload
    assert b'ReadVar={MotorController}17@RW-' not in payload
    print('HMI telemetry addresses, formats, word order, and control coils passed.')


if __name__ == '__main__':
    main()
