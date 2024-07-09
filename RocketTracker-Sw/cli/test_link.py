from link import Frame, Datum, cobs_decode, cobs_encode
import lib.proto.protocol_pb2 as protocol


def test_datum_decode():
    d = Datum()
    header = bytes.fromhex("01")
    body = bytes([i for i in range(16)])
    header += int.to_bytes(len(body), 2, byteorder='little')

    n = d.load_bytes(header + body + bytes([0xFF] * 10))
    assert ((len(header) + len(body)) == n)
    assert d.data == body
    assert d.typeid == protocol.INFO_Raw
    print(d)


def test_datum_parse():
    d = Datum()
    header = bytes.fromhex("07")
    body = bytes.fromhex("080a108002")
    header += int.to_bytes(len(body), 2, byteorder='little')

    n = d.load_bytes(header + body + bytes([0xFF] * 10))

    D = d.to_dict()
    assert D['log_size'] == 10
    assert D['log_maxsize'] == 256

    O = d.to_protobuf()
    assert O.log_size == 10
    assert O.log_maxsize == 256


def test_datum_serdes():
    alt = protocol.Altitude()
    alt.alt_m = 123.4
    d = Datum(protocol.INFO_Altitude)
    d.load_protobuf(alt)

    assert abs(d.to_dict()['alt_m'] - alt.alt_m) < 1e-5


def test_frame_encode():
    d = Datum()
    header = bytes.fromhex("07")
    body = bytes.fromhex("080a108002")
    header += int.to_bytes(len(body), 2, byteorder='little')

    n = d.load_bytes(header + body + bytes([0xFF] * 10))

    F = Frame()
    F.add_datum(d)
    fbytes = F.get_bytes()

    F2 = Frame()
    F2.load_from_bytes(fbytes)

    assert F2.datums[0].to_dict() == F.datums[0].to_dict()

    for i, d in enumerate(F2.datums):
        print(f"datums[{i}] = {d}")


def test_cobs_encode():
    assert cobs_encode(bytes.fromhex("00FF")) == bytes.fromhex("FF02FF01")


def test_cobs_decode():
    assert cobs_decode(bytes.fromhex("FF02FF01")) == bytes.fromhex("00FF")
