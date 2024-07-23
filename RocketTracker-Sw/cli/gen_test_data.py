from link import Frame, Datum, cobs_encode
import lib.proto.protocol_pb2 as protocol

d = Datum(protocol.INFO_SensorData)
d.load_protobuf(
    protocol.SensorData(
        lsm_acceleration_g=[1., 2., 3.],
        lsm_gyro_dps=[-1., -2., -3.],
        lis_magnetic_mG=[0.1, 0.2, 0.3],
        lps_pressure_hPa=123.4,
        adxl_acceleration_g=[10.0, 20.0, 30.0],
    )
)
f = Frame()
f.add_datum(d)
dbs = f.get_bytes()
print(len(dbs))

bs = bytes.fromhex("0000") + \
    cobs_encode(int.to_bytes(len(dbs), 2, 'little')) + \
    cobs_encode(dbs) + \
    bytes.fromhex("0000")
estr = "{ " + ', '.join([hex(e) for e in bs]) + " }"
print(f"uint8_t data[] = {estr};")
