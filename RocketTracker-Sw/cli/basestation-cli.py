#!/usr/bin/env python3
from lib.comms_lib.crctabgen import crc16
import link as link
from link import Frame, Datum, cobs_decode, cobs_encode
import argparse
from serial import Serial
import signal
from queue import Queue
import time
import lib.proto.protocol_pb2 as protocol
from threading import Thread
from distutils.util import strtobool

serialport: Serial | None = None
rx_queue: Queue[Datum] = Queue()
tx_queue: Queue[Datum] = Queue()

send_thread = None
recv_thread = None
EX = False

# valid_types can be an array of valid types, or None to accept all types


def sigint_handler(signum, frame):
    global serialport, EX
    EX = True
    print('Exiting!')
    exit(0)
    # if (serialport is not None and serialport.is_open):
    #     serialport.close()


signal.signal(signal.SIGINT, sigint_handler)


def receiver_thread():
    global serialport, EX
    buffer = []

    while (not EX):
        for char in serialport.read_all():
            if (char == 0x00):
                if (len(buffer)):
                    # TODO: Process buffer!!!
                    data = cobs_decode(bytes(buffer))
                    # print(data.hex(':'))
                    length = int.from_bytes(data[0:2], byteorder='little')
                    if (length == (len(data)-2)):
                        f = Frame()
                        f.load_from_bytes(data[2:])
                        for d in f.datums:
                            rx_queue.put(d)
                buffer.clear()
                continue
            else:
                buffer.append(char)
    pass


def sender_thread():
    global serialport, EX

    while (not EX):
        # Send just under the keepalive
        try:
            d = tx_queue.get(timeout=0.4)
            f = Frame()
            f.add_datum(d)
            bdata = f.get_bytes()

            packet = cobs_encode(int.to_bytes(
                len(bdata), 2, byteorder='little') + bdata)
            serialport.write(packet)
            serialport.write(bytes.fromhex('0000'))
        except SystemExit:
            return

        except:

            serialport.write(bytes.fromhex('0000'))


# NOTE: Need to send zeros to the port for keepalive!!! (timeout = 0.5s)


def wait_for_datum(valid_types=None, timeout=None) -> None | Datum:
    global EX
    stime = time.time()
    while ((time.time() - stime) < timeout) and not EX:
        datum = None
        try:
            datum = rx_queue.get(timeout=timeout)
        except SystemExit:
            return
        except:
            datum = None
        # TODO: Don't discard datums here!!!! just pass them up
        if (datum is None) or (valid_types is None) or (datum.typeid in valid_types):
            return datum
    return None


def init_cli(args):
    global send_thread, recv_thread, serialport

    try:
        serialport = Serial(baudrate=115200, timeout=0.5)
        serialport.port = args.port
        serialport.rts = False  # Don't reset the esp32
        serialport.open()
    except:
        print("Error! could not open serial port")
        exit(1)

    send_thread = Thread(target=sender_thread, daemon=True)
    recv_thread = Thread(target=receiver_thread, daemon=True)

    send_thread.start()
    recv_thread.start()


def log_stop(args):
    init_cli(args)

    confirm = strtobool(input("Are you sure you want to stop logging?:"))


def log_start(args):
    global serialport
    init_cli(args)
    print("log_start")
    pass


def log_delete(args):
    global serialport
    init_cli(args)
    print("log_delete")
    pass


def log_clean(args):
    global serialport
    init_cli(args)
    print("log_clean")

    pass


def log_status(args):
    global serialport
    init_cli(args)
    print("log_status")

    pass


def log_download(args):
    # Log download:
    # -> CMD_StopLog
    # <- RESP_StopLog(Resp_BasicError)
    # -> CMD_LogStatus
    # <- INFO_LogStatus
    # -> CMD_DownloadLog
    # <- ACK_DownloadLog_Segment * N
    # IGNORE IGNORE IGNORE IGNORE IGNORE -> ACK_DownloadLog_Segment // TBD whether this will be implemented!!!
    # <- ACK_Download_Complete
    # -> ACK_Download_Complete
    global serialport
    init_cli(args)
    print("log_download")

    pass


# TODO: Make a nice "view" command with a TUI
def monitor(args):
    global EX
    init_cli(args)

    while (not EX):
        d = wait_for_datum(None, 1.5)
        if d is not None:
            print(d)


def ping(args):
    init_cli(args)

    print("Pinged")

    d = Datum(protocol.CMD_Ping)
    d.load_protobuf(protocol.Command_Ping(link=protocol.USBSerial))

    tx_queue.put(d)

    pong = wait_for_datum([protocol.RESP_Ping], 1.5)
    print(
        f"Got pong! connected over link type '{protocol.LinkID.DESCRIPTOR.values_by_number[pong.to_protobuf().link].name}'")


def receiver_config(args):
    # We are talking to the VGPS port now!!
    vgps = Serial(args.port, baudrate=115200, timeout=0.5)

    if args.spreading_factor is not None:
        vgps.write(f"$PSF,{args.spreading_factor}\r\n".encode('ascii'))
    if args.freq_mhz is not None:
        vgps.write(f"$PFRF,{args.freq_mhz}\r\n".encode('ascii'))
    if args.bandwidth is not None:
        vgps.write(f"$PBW,{args.bandwidth}\r\n".encode('ascii'))
    if args.power is not None:
        vgps.write(f"$PPOW,{args.power}\r\n".encode('ascii'))

    vgps.flush()
    vgps.close()


def argparse_restricted_float(mi, ma):
    def worker(x):
        try:
            x = float(x)
        except ValueError:
            raise argparse.ArgumentTypeError(
                "%r not a floating-point literal" % (x,))
        if x < mi or x > ma:
            raise argparse.ArgumentTypeError(
                f"{x} not in range [{mi}, {ma}]")
        return x
    return worker


parser = argparse.ArgumentParser(
    prog='basestation-cli',
    description='CLI interface for rocket tracking system components (receiver and tracker v0.3)')
subparsers = parser.add_subparsers(help='sub-command help')
parser.add_argument('-p', '--port', required=True, type=str)

parser_rxconfig = subparsers.add_parser('rxconfig')
parser_rxconfig.set_defaults(func=receiver_config)
parser_rxconfig.add_argument(
    '-f', '--freq-mhz', type=argparse_restricted_float(903.0, 927.0))
parser_rxconfig.add_argument(
    '-sf', '--spreading-factor', type=int, choices=[6, 7, 8, 9, 10, 11, 12])
parser_rxconfig.add_argument(
    '-bw', '--bandwidth', type=str, choices=["500", "250", "125", "62.5", "41.7", "31.25", "20.8", "15.6", "10.4", "7.8"])
parser_rxconfig.add_argument(
    '-p', '--power', type=int, choices=[3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 20])

parser_monitor = subparsers.add_parser('monitor')
parser_monitor.set_defaults(func=monitor)

parser_ping = subparsers.add_parser('ping')
parser_ping.set_defaults(func=ping)

# TODO: Add proper help/usage for __ALL__ commands and subcommands!!!
parser_log = subparsers.add_parser('log')
log_subparsers = parser_log.add_subparsers()
# parser_log.add_argument('bar', type=int, help='bar help')

parser_log_stop = log_subparsers.add_parser('stop')
parser_log_stop.set_defaults(func=log_stop)

parser_log_start = log_subparsers.add_parser('start')
parser_log_start.set_defaults(func=log_start)

parser_log_delete = log_subparsers.add_parser('delete')
parser_log_delete.set_defaults(func=log_delete)

parser_log_clean = log_subparsers.add_parser('clean')
parser_log_clean.set_defaults(func=log_clean)

parser_log_status = log_subparsers.add_parser('status')
parser_log_status.set_defaults(func=log_status)

parser_log_download = log_subparsers.add_parser('download')
parser_log_download.add_argument(
    '-o', '--outfile', required=True, type=argparse.FileType('w'))
parser_log_download.add_argument(
    '-f', '--format', required=True, type=str, choices=['rlog', 'csv', 'json'])
parser_log_download.set_defaults(func=log_download)

opts = parser.parse_args()
opts.func(opts)
