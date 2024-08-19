#!/usr/bin/env python3
if True:
    import sys
    sys.path.append("../rtrk-config/")

from colorama import Fore, Back, Style
from colorama import init as colinit
from util import *
import json
import socket
from pyquaternion import Quaternion
import math
from mag_cal import calibrate_magnetometer, calculate_2_pt_calibration
import curses
import numpy as np
from tqdm import tqdm
from configgen import encode_key
from configparse import parse_configuration, flatten_config_values, flatten_config_types, FloatType, EnumType, BoolType, IntType, StringType
from os import path
import pickle
from str2bool import str2bool as strtobool
import sys
from threading import Thread
import lib.proto.protocol_pb2 as protocol
import time
from threading import Condition
from queue import Queue
import signal
import serial
from serial import Serial
import argparse
from link import Frame, Datum, cobs_decode, cobs_encode
import link as link
from lib.comms_lib.crctabgen import crc16
import serial.tools.list_ports
import serial.tools
from logtool import log_convert, log_info
from loglib import RTRKLog

colinit()


CONFIG_DIR = path.abspath(path.join(
    sys.path[0], "..", "rtrk-config/configfiles/"))
CONFIGURATION = parse_configuration(
    open(path.join(CONFIG_DIR, "tracker_config.jsonc")), root=CONFIG_DIR)
CONFIG_KEY_ENCODE = {v.path: encode_key(
    v.path) for v in flatten_config_values(CONFIGURATION)}
CONFIG_KEY_DECODE = {v: k for k, v in CONFIG_KEY_ENCODE.items()}
CONFIG_LOOKUP = {
    v.path: v for v in flatten_config_values(CONFIGURATION)
}

serialport: Serial | None = None
serialport_automatic = None
rx_queue: Queue[Datum] = Queue()
tx_queue: Queue[Datum] = Queue()
tx_done = Condition()

send_thread = None
recv_thread = None
EX = False

# valid_types can be an array of valid types, or None to accept all types


def sigint_handler(signum, frame):
    global serialport, EX

    if (EX):
        print("Force quitting")
        exit(0)
    EX = True
    print('Exiting!')
    # exit(0)
    # if (serialport is not None and serialport.is_open):
    #     serialport.close()


signal.signal(signal.SIGINT, sigint_handler)


def receiver_thread(args):
    global serialport, EX
    buffer = []

    while (True):
        for char in serialport.read_all():
            if (char == 0x00):
                if (len(buffer)):
                    # TODO: Process buffer!!!
                    data = cobs_decode(bytes(buffer))
                    # print(data.hex(':'))
                    length = int.from_bytes(data[0:2], byteorder='little')
                    # print(length, len(data))

                    if (length == (len(data)-2)):
                        f = Frame()
                        f.load_from_bytes(data[2:])
                        if args.verbose:
                            print(f"Got frame: {f}")
                        for d in f.datums:
                            # print(d)
                            rx_queue.put(d)
                    else:
                        # print(f"Received corrupt frame: {data.hex(':')}")
                        print(f"Length mismatch:", length, len(data)-2)
                buffer.clear()
                continue
            else:
                buffer.append(char)
    pass


def sender_thread(args):
    global serialport, EX

    while (True):
        # Send just under the keepalive
        try:
            d = tx_queue.get(timeout=0.4)
            f = Frame()
            f.add_datum(d)

            if (args.verbose):
                print(f"Sending Frame: {f}")

            bdata = f.get_bytes()

            packet = cobs_encode(int.to_bytes(
                len(bdata), 2, byteorder='little') + bdata)
            serialport.write(bytes.fromhex('0000'))
            serialport.write(packet)
            serialport.write(bytes.fromhex('0000'))
            tx_done.acquire()
            tx_done.notify()
            tx_done.release()
        except SystemExit:
            return

        except:

            serialport.write(bytes.fromhex('0000'))


# NOTE: Need to send zeros to the port for keepalive!!! (timeout = 0.5s)


def wait_for_datum(valid_types=None, timeout=None) -> None | Datum:
    global EX
    stime = time.time()
    while ((time.time() - stime) < timeout):
        datum = None
        try:
            datum = rx_queue.get(timeout=0.1)
        except SystemExit or KeyboardInterrupt:
            return
        except:
            datum = None
        # TODO: Don't discard datums here!!!! just pass them up
        if (datum is not None) and ((valid_types is None) or (datum.typeid in valid_types)):
            return datum
    return None


def flush_rx():
    global serialport, rx_queue
    while (not rx_queue.empty()):
        rx_queue.get()
    serialport.read_all()


# Returns (data port, vgps port)
def autodetect_receiver(comports):
    rxports = [p for p in comports if p.vid == 0xCAFE]
    rxports.sort(key=lambda v: v.name)
    if (len(rxports) >= 2):
        # print(f"Data: {rxports[0].name}, VGPS: {rxports[1].name}")
        return (rxports[0], rxports[1])
    else:
        return None


def autodetect_tracker(comports):
    trackerports = [p for p in comports if p.vid == 0x303A]
    if (len(trackerports) >= 1):
        return (trackerports[0], None)
    else:
        return None


def init_cli(args):
    global send_thread, recv_thread, serialport, serialport_automatic

    if (args.port is None):
        comports = serial.tools.list_ports.comports()
        # Autodetect
        tracker = autodetect_tracker(comports)
        if tracker is not None:
            args.port = tracker[0].name
            serialport_automatic = tracker[0]
        else:
            rx = autodetect_receiver(comports)
            if rx is not None:
                args.port = rx[0].name
                serialport_automatic = rx[0]

    try:
        serialport = Serial(baudrate=115200, timeout=0.5)
        serialport.port = args.port
        serialport.rts = False  # Don't reset the esp32
        serialport.open()
        serialport.flush()
        serialport.read_all()
    except:
        print("Error! could not open serial port")
        exit(1)

    send_thread = Thread(target=sender_thread, daemon=True, args=[args])
    recv_thread = Thread(target=receiver_thread, daemon=True, args=[args])

    send_thread.start()
    recv_thread.start()


def ensure_usb():
    global serialport, serialport_automatic
    if (serialport_automatic is not None):
        is_tracker = serialport_automatic.vid == 0x303A
        if is_tracker:
            return
    else:
        # Need to ping
        d = Datum(protocol.CMD_Ping)
        d.load_protobuf(protocol.Command_Ping(link=protocol.USBSerial))

        tx_queue.put(d)

        pong = wait_for_datum([protocol.RESP_Ping], 1.5)
        if (pong is not None and pong.to_protobuf().link == protocol.USBSerial):
            return

    print("Error, this function requires direct USB connection to the tracker")
    exit(1)


def log_stop(args):
    init_cli(args)

    confirm = args.yes or strtobool(
        input("Are you sure you want to stop logging?\n[y/n]:"))
    if (confirm):
        command = protocol.Command_ConfigureLogging(setting=protocol.Stopped)

        d = Datum(protocol.CMD_ConfigureLogging)
        d.load_protobuf(command)

        tx_queue.put(d)

        resp = wait_for_datum([protocol.RESP_ConfigureLogging], 5.0)
        # print(resp)
        if (resp is None):
            print("Error: timed out")
        elif ("error" in resp.to_dict()):
            print(f"Error stopping log: {resp.to_dict()['error']}")
        else:
            print(f"Successfully stopped logging")


def log_start(args):
    init_cli(args)

    print("Starting logging")
    command = protocol.Command_ConfigureLogging(
        setting=protocol.ManualHz, parameter=args.hz)

    d = Datum(protocol.CMD_ConfigureLogging)
    d.load_protobuf(command)

    tx_queue.put(d)

    resp = wait_for_datum([protocol.RESP_ConfigureLogging], 5.0)
    # print(resp)
    if (resp is None):
        print("Error: timed out")
    elif ("error" in resp.to_dict()):
        print(f"Error starting logging: {resp.to_dict()['error']}")
    else:
        print(f"Successfully started logging at {args.hz}Hz")


def log_arm(args):
    init_cli(args)

    print("Arming automatic logging")
    command = protocol.Command_ConfigureLogging(
        setting=protocol.Armed)

    d = Datum(protocol.CMD_ConfigureLogging)
    d.load_protobuf(command)

    tx_queue.put(d)

    resp = wait_for_datum([protocol.RESP_ConfigureLogging], 5.0)
    # print(resp)
    if (resp is None):
        print("Error: timed out")
    elif ("error" in resp.to_dict()):
        print(f"Error arming automatic logging: {resp.to_dict()['error']}")
    else:
        print(f"Successfully armed logging")


def log_delete(args):
    init_cli(args)

    confirm = args.yes or strtobool(
        input("Are you sure you want to erase the current log? all data will be lost\nThis may take a while\n[y/n]:"))
    if (confirm):
        command = protocol.Command_EraseLog(type=protocol.Erase_Log)

        d = Datum(protocol.CMD_EraseLog)
        d.load_protobuf(command)

        tx_queue.put(d)

        resp = wait_for_datum([protocol.RESP_EraseLog], 400.0)
        # print(resp)
        if (resp is None):
            print("Error: timed out")
        elif ("error" in resp.to_dict()):
            print(f"Error erasing log: {resp.to_dict()['error']}")
        else:
            print(f"Successfully deleted log")
    else:
        print("Aborted, did not erase log")


def log_mark(args):
    init_cli(args)

    data = bytes.fromhex("DEADBEEF") if (
        args.data is None) else bytes.fromhex(args.data)

    print("Writing a marker to the log NOW")
    print(f"Marker data: {data.hex(':')}")
    d = Datum(protocol.INFO_Raw)
    d.set_data(data)
    tx_queue.put(d)

    tx_done.acquire()
    tx_done.wait()
    tx_done.release()


def log_clean(args):
    init_cli(args)
    confirm = args.yes or strtobool(
        input("Are you sure you want to CLEAN the log memory? all data will be lost.\nThe entire log memory will be erased, this may take up to 7 minutes!\n[y/n]:"))
    if (confirm):
        command = protocol.Command_EraseLog(type=protocol.Erase_Clean)

        d = Datum(protocol.CMD_EraseLog)
        d.load_protobuf(command)

        tx_queue.put(d)

        # Timeout 400 seconds, max erase time in flash memory datasheet!
        resp = wait_for_datum([protocol.RESP_EraseLog], 400)
        # print(resp)
        if (resp is None):
            print("Error: timed out")
        elif ("error" in resp.to_dict()):
            print(f"Error cleaning log memory: {resp.to_dict()['error']}")
        else:
            print(f"Successfully cleaned log memory")
    else:
        print("Aborted, did not clean log memory")


def log_status(args):
    init_cli(args)

    print("Requesting log status...")
    d = Datum(protocol.CMD_LogStatus)
    d.set_data(bytes())

    tx_queue.put(d)

    status = wait_for_datum([protocol.INFO_LogStatus], 12.0)
    if (status is None):
        print("Error: timed out")
    else:
        print(status.to_dict())


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
    init_cli(args)

    # print("Testing connection... ")
    # d = Datum(protocol.CMD_Ping)
    # d.load_protobuf(protocol.Command_Ping(link=protocol.USBSerial))

    # tx_queue.put(d)

    # pong = wait_for_datum([protocol.RESP_Ping], 1.5)
    # if (pong is None):
    #     print("Error: timed out")
    #     return 2
    # elif (pong.to_protobuf().link == protocol.LoRa):
    #     print("Error: Cannot download logs over LoRa/receiver connection")
    #     return 1
    # else:
    #     print("connected over USB")
    ensure_usb()

    d = Datum(protocol.CMD_ConfigureLogging)
    d.load_protobuf(protocol.Command_ConfigureLogging(
        setting=protocol.Stopped))

    print("Stopping logging... ", end='')
    tx_queue.put(d)
    resp = wait_for_datum([protocol.RESP_ConfigureLogging], 5.0)
    # print(resp)
    if (resp is None):
        print("Error: log stop timed out")
        return 2
    elif ("error" in resp.to_dict()):
        print(f"Error stopping logging: {resp.to_dict()['error']}")
        return 1
    else:
        print("stopped")

    print("Requesting log status... ", end='')
    d = Datum(protocol.CMD_LogStatus)
    d.set_data(bytes())

    tx_queue.put(d)

    status = wait_for_datum([protocol.INFO_LogStatus], 5.0)
    if (status is None):
        print("Error: timed out")
        return 2
    else:
        print("success")

    print("Starting download sequence... ", end='')
    d = Datum(protocol.CMD_DownloadLog)
    d.set_data(bytes())
    tx_queue.put(d)

    resp = wait_for_datum([protocol.RESP_DownloadLog], 5.0)
    if (resp is None):
        print("Error: timed out")
        return 1
    elif ("error" in resp.to_dict()):
        print(f"Error initiating download: {resp.to_dict()['error']}")
        return 1
    else:
        print("started!")

    segments = []
    while (True):
        segment = wait_for_datum(
            [protocol.ACK_Download_Complete, protocol.RESP_DownloadLog_Segment], 5.0)
        if (segment is None):
            print("Error: timed out, stopping download")
            return 1
        elif (segment.typeid == protocol.ACK_Download_Complete):
            print(
                f"Download complete! Downloaded {status.to_dict()['log_size']}")
            segments.append(segment.to_dict())  # Get the ACK too!
            # print(segments)

            if (args.raw):
                pickle.dump(segments, args.outfile)
                args.outfile.flush()
                args.outfile.close()
                print("Saved to pickle")
            else:
                # Semi-decode
                log = RTRKLog()
                log.load_segments(segments)
                args.outfile.write(log.data)
                args.outfile.flush()
                pass

            segments, ack = segments[:-1], segments[-1]
            # print(ack)
            crc_xor = ack['log_crc16']
            xor = 0
            for s in segments:
                xor ^= s['segment_crc16']
            if (xor != crc_xor):
                print(Fore.RED + "ERROR! Downloaded log CRC mismatch")
                print(Fore.YELLOW +
                      "Please re-download log to prevent data loss/corruption!")
                print(Style.RESET_ALL)
            else:
                print(Fore.GREEN + "Log CRC verified")
                print(Style.RESET_ALL)

            break
        else:
            if ("error" in segment.to_dict()):
                print(
                    f"Error while downloading log: {segment.to_dict()['error']}")
                return
            segments.append(segment.to_dict())

    exit(0)

# TODO: Make a nice "view" command with a TUI


def monitor(args):
    global EX
    init_cli(args)

    sock = UDPJSONSender(args.udp) if args.udp is not None else None

    while (not EX):
        d = wait_for_datum(None, 1.5)
        if d is not None:  # and d.typeid != protocol.DatumTypeID.STATUS_RadioRxStatus
            print(d)
            if (args.log is not None):
                args.log.writelines([
                    f"{json.dumps({'timestamp': time.time(), 'datum': d.to_dict()})}\n"
                ])
            if (sock is not None):
                sock.send(d.to_dict())

    if (args.log is not None):
        args.log.flush()
        args.log.close()
    exit(0)


def reboot(args):
    init_cli(args)

    confirm = args.yes or strtobool(
        input("Are you sure you want to reboot the device? [y/n]:"))
    if (confirm):
        d = Datum(protocol.CMD_Reboot)
        tx_queue.put(d)
        print("Attempted to reboot the device")
    else:
        print("Aborted")


def debug(args):
    global EX
    init_cli(args)
    ensure_usb()

    datapoints = {}
    stdscr = curses.initscr()
    stdscr.nodelay(True)
    curses.noecho()
    curses.cbreak()

    sock = UDPJSONSender(args.udp) if args.udp is not None else None

    while (not EX):
        d = wait_for_datum([protocol.INFO_Debug], 1.5)
        stdscr.addstr(0, 0, "Press 'q' to exit")
        if d is not None:  # and d.typeid != protocol.DatumTypeID.STATUS_RadioRxStatus
            # print(d)

            # datapoints
            pb = d.to_protobuf()
            tup = (pb.float_value if pb.HasField(
                "float_value") else pb.int_value, pb.suffix)

            datapoints[pb.name] = tup
            # if (args.log)
            # args.log.writelines([f"{d.to_dict()}"])
            for i, (name, (value, suffix)) in enumerate(datapoints.items()):
                stdscr.addstr(
                    i+1, 0, f"{name} = {value:9.4f}{suffix}                     ")
            if sock is not None:
                sock.send(datapoints)
        else:
            stdscr.clear()
            stdscr.addstr(
                0, 0, "Timed out, maybe you forgot to enable debug monitoring over USB?")
            break

        if (stdscr.getch() == ord('q')):
            break
        stdscr.refresh()
    exit(0)
    # args.log.close()


def telemetry(args):
    global EX
    init_cli(args)

    datapoints = {}
    stdscr = curses.initscr()
    stdscr.nodelay(True)
    curses.noecho()
    curses.cbreak()

    sock = UDPJSONSender(args.udp) if args.udp is not None else None

    while (not EX):
        d = wait_for_datum([protocol.INFO_GPS, protocol.INFO_Altitude,
                           protocol.INFO_Battery, protocol.INFO_LogStatus, protocol.STATUS_RadioRxStatus], 0.1)
        stdscr.addstr(0, 0, "Press 'q' to exit")
        if d is not None:  # and d.typeid != protocol.DatumTypeID.STATUS_RadioRxStatus
            # print(d)

            # datapoints
            pb = d.to_protobuf()
            if (d.typeid == protocol.INFO_GPS):
                stdscr.addstr(1, 0, "GPS:")
                stdscr.addstr(2, 0, f"\tLatitude: {pb.lat}             ")
                stdscr.addstr(3, 0, f"\tLongitude: {pb.lon}             ")
                stdscr.addstr(4, 0, f"\tAltitude: {pb.alt} (m)               ")
                stdscr.addstr(
                    5, 0, f"\tFix Quality: {pb.fix_status if pb.HasField('fix_status') else 'unknown'}             ")
                stdscr.addstr(
                    6, 0, f"\tSatellites: {pb.sats_used if pb.HasField('sats_used') else 'sats_used'}          ")

            elif (d.typeid == protocol.INFO_Altitude):
                stdscr.addstr(7, 0, "Altimetry:")
                stdscr.addstr(
                    8, 0, f"\tFiltered Altitude: {pb.alt_m:.2f} (m)                  ")
                stdscr.addstr(
                    9, 0, f"\tVertical Speed: {f'{pb.v_speed:.2f} (m/s)' if pb.HasField('v_speed') else 'not available'}")
            elif (d.typeid == protocol.INFO_Battery):
                stdscr.addstr(
                    10, 0, f"Battery: {pb.percentage:.1f}% ({pb.battery_voltage:.2f}v)")

            elif (d.typeid == protocol.INFO_LogStatus):
                stdscr.addstr(
                    11, 0, f"Logging:")
                stdscr.addstr(
                    12, 0, f"\tSize: {pb.log_size/1000:.1f}KB ({(pb.log_size/pb.log_maxsize)*100:.1f}%)")
                sstr = ''
                if pb.is_armed:
                    sstr = f"Armed {pb.cur_logging_hz if pb.HasField('cur_logging_hz') else '-'}Hz"
                else:
                    sstr = f"Logging {pb.cur_logging_hz if pb.HasField('cur_logging_hz') else '-'}Hz"
                stdscr.addstr(
                    13, 0, f"\tState: {sstr}           ")

            elif (d.typeid == protocol.STATUS_RadioRxStatus):
                stdscr.addstr(
                    14, 0, f"Receiver:")
                stdscr.addstr(
                    15, 0, f"\tRSSI: {pb.RSSI}dBm           ")
                stdscr.addstr(
                    16, 0, f"\tSNR: {pb.SNR}           ")

            if sock is not None:
                sock.send(d.to_dict())
            if (args.log):
                args.log.writelines([
                    f"{json.dumps({'timestamp': time.time(), 'datum': d.to_dict()})}\n"
                ])
        else:
            # stdscr.clear()
            # print("Timed out")
            # Do something for timeout...
            pass

        if (stdscr.getch() == ord('q')):
            break
        stdscr.refresh()

    if (args.log is not None):
        args.log.flush()
        args.log.close()
    exit(0)
    # args.log.close()


def sensors3d(args):
    init_cli(args)
    import vpython as vp

    d = Datum(protocol.CMD_ConfigSensorOutput)
    d.load_protobuf(protocol.Command_ConfigSensorOutput(
        rate_hz=args.frequency, raw=False))
    tx_queue.put(d)

    box = vp.box(pos=vp.vector(0, 0, 0), axis=vp.vector(1, 0, 0), up=vp.vector(0, 0, 1), height=0.2, length=6,
                 width=2.5, color=vp.color.green)

    stdscr = curses.initscr()
    stdscr.nodelay(True)
    curses.noecho()
    curses.cbreak()

    while (True):
        d = wait_for_datum([protocol.INFO_SensorData], 1.5)
        if d is not None:  # and d.typeid != protocol.DatumTypeID.STATUS_RadioRxStatus
            pb = d.to_protobuf()

            if (pb.HasField('filtered_orientation')):
                quat = pb.filtered_orientation
                Q = Quaternion(quat.w, quat.x, quat.y, quat.z)
                up = Q.rotate([0, 0, 1.0])
                fwd = Q.rotate([0.0, -1.0, 0.0])
                stdscr.addstr(0, 0, f"Forwards: {vector_print_just(fwd)}")
                stdscr.addstr(1, 0, f"Upwards:  {vector_print_just(up)}")

                box.axis.x = float(fwd[0])
                box.axis.y = float(fwd[1])
                box.axis.z = float(fwd[2])

                box.up.x = float(up[0])
                box.up.y = float(up[1])
                box.up.z = float(up[2])
                # box.up = vp.vector(up[0], up[1], up[2])
                # board.

                # board.addattr()

                # stdscr.addstr(
                # 5, 0, f"Filtered Orientation (roll,pitch,yaw): {vector_print_just(rpy)}")
                # stdscr.addstr(
                # 6, 0, f"Filtered Orientation (w,x,y,z): {vector_print_just([quat.w, quat.x, quat.y, quat.z])}")
                # print(quat)

        if (stdscr.getch() != -1):
            EX = True
            break

    curses.nocbreak()
    stdscr.keypad(False)
    curses.echo()
    curses.endwin()

    d = Datum(protocol.CMD_ConfigSensorOutput)
    d.load_protobuf(protocol.Command_ConfigSensorOutput(
        rate_hz=0, raw=False))
    tx_queue.put(d)


def sensormon(args):

    global EX
    init_cli(args)
    ensure_usb()

    d = Datum(protocol.CMD_ConfigSensorOutput)
    d.load_protobuf(protocol.Command_ConfigSensorOutput(
        rate_hz=args.frequency, raw=args.raw))
    tx_queue.put(d)

    stdscr = curses.initscr()
    stdscr.nodelay(True)
    curses.noecho()
    curses.cbreak()

    # port = 9870
    # ip = "127.0.0.1"
    sock = UDPJSONSender(args.udp) if args.udp is not None else None

    while (not EX):
        d = wait_for_datum([protocol.INFO_SensorData], 1.5)
        if d is not None:  # and d.typeid != protocol.DatumTypeID.STATUS_RadioRxStatus
            # print(d)
            # repeated float lsm_acceleration_g = 1;
            # repeated float lsm_gyro_dps = 2
            # repeated float adxl_acceleration_g = 3
            # repeated float lis_magnetic_mG = 4
            # required float lps_pressure_hPa = 5

            # optional Quaternion filtered_orientation = 6
            # optional Altitude filtered_altimetry = 7

            pb = d.to_protobuf()
            stdscr.addstr(
                0, 0, f"ADXL375 Acceleration (g): {vector_print_just(pb.adxl_acceleration_g)}")
            stdscr.addstr(
                1, 0, f"LSM6DSM Acceleration (g): {vector_print_just(pb.lsm_acceleration_g)}")
            stdscr.addstr(
                2, 0, f"LSM6DSM Gyro (dps): {vector_print_just(pb.lsm_gyro_dps)}")
            stdscr.addstr(
                3, 0, f"LIS3MDL Magnetic Field (mG): {vector_print_just(pb.lis_magnetic_mG)}")
            stdscr.addstr(
                4, 0, f"LPS22 Pressure (hPa): {pb.lps_pressure_hPa:9.4f}")

            if (pb.HasField('filtered_orientation')):
                quat = pb.filtered_orientation
                rpy = quat2euler((quat.w, quat.x, quat.y, quat.z))
                stdscr.addstr(
                    5, 0, f"Filtered Orientation (roll,pitch,yaw): {vector_print_just(rpy)}")
                stdscr.addstr(
                    6, 0, f"Filtered Orientation (w,x,y,z): {vector_print_just([quat.w, quat.x, quat.y, quat.z])}")

            if (pb.HasField('filtered_altimetry')):
                stdscr.addstr(
                    7, 0, f"Filtered Altitude (m): {pb.filtered_altimetry.alt_m:9.4f}")
                stdscr.addstr(
                    8, 0, f"Filtered V-Speed (m/s): {pb.filtered_altimetry.v_speed:9.4f}")

            stdscr.refresh()
            if (args.log):
                args.log.writelines([f"{d.to_dict()}\n"])

            if (sock is not None):
                sock.send(d.to_dict())

            if (stdscr.getch() != -1):
                EX = True
                break

    curses.nocbreak()
    stdscr.keypad(False)
    curses.echo()
    curses.endwin()

    d = Datum(protocol.CMD_ConfigSensorOutput)
    d.load_protobuf(protocol.Command_ConfigSensorOutput(
        rate_hz=0, raw=False))
    tx_queue.put(d)

    print("Cleaned up.")
    exit(0)
    # args.log.close()


CALIBSTATE_STANDBY = 1
CALIBSTATE_X_WAITING = 2
CALIBSTATE_X_DONE = 3
CALIBSTATE_Y_WAITING = 4
CALIBSTATE_Y_DONE = 5
CALIBSTATE_Z_WAITING = 6
CALIBSTATE_Z_DONE = 7
CALIBSTATE_DONE = 8
CALIBSTATE_QUIT = 9


def set_configval(key, val):
    if ((key not in CONFIG_KEY_ENCODE) and (key not in CONFIG_KEY_DECODE)):
        print(f"Error, config value not found with key '{key}'")
        return False

    if key in CONFIG_KEY_DECODE:
        key = CONFIG_KEY_DECODE[key]

    parameter = CONFIG_LOOKUP[key]

    d = Datum(protocol.CMD_Config)
    pb = protocol.Config(
        key_hashed=CONFIG_KEY_ENCODE[key], mode=protocol.ConfigSet)
    value = parameter.type.parse_value(val)
    if (value is None):
        print("Error converting value!")
        print(f"Type: {parameter.type}")
        return False

    if isinstance(parameter.type, FloatType):
        pb.float_value = value
    elif isinstance(parameter.type, IntType):
        pb.int_value = value
    elif isinstance(parameter.type, BoolType):
        pb.bool_value = value
    elif isinstance(parameter.type, StringType):
        pb.string_value = value
    elif isinstance(parameter.type, EnumType):
        pb.enum_value = value

    d.load_protobuf(pb)

    # print(d)
    tx_queue.put(d)
    resp = wait_for_datum([protocol.RESP_ConfigSet], 5.0)
    # print(resp)
    if (resp is None):
        print("Error: timed out")
        return False
    elif ("error" in resp.to_dict()):
        print(f"Error setting config value: {resp.to_dict()['error']}")
        return False
    else:
        print(f"Successfully set {key} to {value}")
        return True


def set_2_pt_calibration(basename, calib):
    offset, scale = calib
    offset_key = basename+"_offset"
    scale_key = basename+"_scale"

    set_configval(offset_key, offset)
    set_configval(scale_key, scale)


def calibrate_reset(args):
    init_cli(args)
    d = CONFIG_LOOKUP

    cont = input(
        "Are you sure you want to reset the device's calibration? [y/n]:")
    if (not strtobool(cont)):
        print("Aborting.")
        exit(0)

    for k, v in sorted(d.items()):
        if (k.startswith('config.calibration.')):
            d = Datum(protocol.CMD_Config)
            pb = protocol.Config(
                key_hashed=CONFIG_KEY_ENCODE[k], mode=protocol.ConfigErase)
            d.load_protobuf(pb)
            tx_queue.put(d)

            resp = wait_for_datum([protocol.RESP_ConfigSet], 5.0)
            # print(resp)
            if (resp is None):
                print(f"Error: timed out erasing parameter with key '{k}'")
            elif ("error" in resp.to_dict()):
                print(f"Error erasing config value: {resp.to_dict()['error']}")
    print("Done.")


def calibrate_acc(args):
    global EX
    init_cli(args)
    ensure_usb()

    d = Datum(protocol.CMD_ConfigSensorOutput)
    d.load_protobuf(protocol.Command_ConfigSensorOutput(
        rate_hz=200, raw=True))
    tx_queue.put(d)

    N_SAMPLES = 200

    # repeated float lsm_acceleration_g = 1;
    # repeated float lsm_gyro_dps = 2
    # repeated float adxl_acceleration_g = 3

    pos_z_adxl_samples = []
    pos_z_lsm_samples = []

    neg_y_adxl_samples = []
    neg_y_lsm_samples = []

    neg_x_adxl_samples = []
    neg_x_lsm_samples = []

    stdscr = curses.initscr()
    stdscr.nodelay(True)
    curses.noecho()
    curses.cbreak()

    stdscr.addstr(
        0, 0, "Place the tracker flat on a stable, level surface and press <space> to begin calibration.")

    # Z -> X -> Y

    state = CALIBSTATE_STANDBY
    while ((state != CALIBSTATE_DONE) and (state != CALIBSTATE_QUIT) and not EX):
        next_state = state

        if (state == CALIBSTATE_STANDBY):
            ch = stdscr.getch()
            if (ch == ord(' ')):
                next_state = CALIBSTATE_Z_WAITING
                flush_rx()
            elif ch == -1:
                next_state = state
            else:
                next_state = CALIBSTATE_QUIT
            time.sleep(1/60.0)
        elif (state == CALIBSTATE_X_WAITING):
            stdscr.addstr(1, 0, "Collecting data...")
            if len(neg_x_adxl_samples) < N_SAMPLES:
                # Collect more data
                d = wait_for_datum([protocol.INFO_SensorData], 1.0)
                if d is None:
                    stdscr.clear()
                    stdscr.addstr(0, 0, "Error! timed out")
                    stdscr.refresh()
                    time.sleep(1)
                    next_state = CALIBSTATE_QUIT
                else:
                    pb = d.to_protobuf()
                    neg_x_adxl_samples.append(pb.adxl_acceleration_g)
                    neg_x_lsm_samples.append(pb.lsm_acceleration_g)
            else:
                next_state = CALIBSTATE_X_DONE
        elif (state == CALIBSTATE_X_DONE):
            # Next: Y
            stdscr.clear()
            stdscr.addstr(
                0, 0, "Place the tracker resting on it's right side (GPS down) and press <space> to continue calibration.")
            stdscr.refresh()

            ch = stdscr.getch()
            if (ch == ord(' ')):
                next_state = CALIBSTATE_Y_WAITING
                flush_rx()
            elif ch == -1:
                next_state = state
            else:
                next_state = CALIBSTATE_QUIT
        elif (state == CALIBSTATE_Y_WAITING):
            stdscr.addstr(1, 0, "Collecting data...")
            if len(neg_y_adxl_samples) < N_SAMPLES:
                # Collect more data
                d = wait_for_datum([protocol.INFO_SensorData], 1.0)
                if d is None:
                    stdscr.clear()
                    stdscr.addstr(0, 0, "Error! timed out")
                    stdscr.refresh()
                    time.sleep(1)
                    next_state = CALIBSTATE_QUIT
                else:
                    pb = d.to_protobuf()
                    neg_y_adxl_samples.append(pb.adxl_acceleration_g)
                    neg_y_lsm_samples.append(pb.lsm_acceleration_g)
            else:
                next_state = CALIBSTATE_Y_DONE
        elif (state == CALIBSTATE_Y_DONE):
            next_state = CALIBSTATE_DONE
            # Done!
        elif (state == CALIBSTATE_Z_WAITING):
            stdscr.addstr(1, 0, "Collecting data...")
            if len(pos_z_adxl_samples) < N_SAMPLES:
                # Collect more data
                d = wait_for_datum([protocol.INFO_SensorData], 1.0)
                if d is None:
                    stdscr.clear()
                    stdscr.addstr(0, 0, "Error! timed out")
                    stdscr.refresh()
                    time.sleep(1)
                    next_state = CALIBSTATE_QUIT
                else:
                    pb = d.to_protobuf()
                    pos_z_adxl_samples.append(pb.adxl_acceleration_g)
                    pos_z_lsm_samples.append(pb.lsm_acceleration_g)
            else:
                next_state = CALIBSTATE_Z_DONE
        elif (state == CALIBSTATE_Z_DONE):
            # Next: X
            stdscr.clear()
            stdscr.addstr(
                0, 0, "Place the tracker resting on it's bottom (antenna up) and press <space> to continue calibration.")
            stdscr.refresh()

            ch = stdscr.getch()
            if (ch == ord(' ')):
                next_state = CALIBSTATE_X_WAITING
                flush_rx()
            elif ch == -1:
                next_state = state
            else:
                next_state = CALIBSTATE_QUIT

        stdscr.refresh()
        state = next_state

    curses.nocbreak()
    stdscr.keypad(False)
    curses.echo()
    curses.endwin()

    if (state == CALIBSTATE_DONE):
        print(f"Calibration done!\n")
        # print(f"pos_z_adxl_samples = {len(pos_z_adxl_samples)}")
        # print(f"pos_z_lsm_samples = {len(pos_z_lsm_samples)}")
        # print()
        # print(f"pos_y_adxl_samples = {len(pos_y_adxl_samples)}")
        # print(f"pos_y_lsm_samples = {len(pos_y_lsm_samples)}")
        # print()
        # print(f"neg_x_adxl_samples = {len(neg_x_adxl_samples)}")
        # print(f"neg_x_lsm_samples = {len(neg_x_lsm_samples)}")

        pos_z_adxl_avgs = np.average(np.array(pos_z_adxl_samples), axis=0)
        pos_z_lsm_avgs = np.average(np.array(pos_z_lsm_samples), axis=0)

        neg_y_adxl_avgs = np.average(np.array(neg_y_adxl_samples), axis=0)
        neg_y_lsm_avgs = np.average(np.array(neg_y_lsm_samples), axis=0)

        neg_x_adxl_avgs = np.average(np.array(neg_x_adxl_samples), axis=0)
        neg_x_lsm_avgs = np.average(np.array(neg_x_lsm_samples), axis=0)

        print("X", neg_x_adxl_avgs, neg_x_lsm_avgs)
        print("Y", neg_y_adxl_avgs, neg_y_lsm_avgs)
        print("Z", pos_z_adxl_avgs, pos_z_lsm_avgs)

        # Z axis calibration
        adxl_z_cal = calculate_2_pt_calibration(
            0.0, neg_x_adxl_avgs[2], 1.0, pos_z_adxl_avgs[2])
        lsm_z_cal = calculate_2_pt_calibration(
            0.0, neg_x_lsm_avgs[2], 1.0, pos_z_lsm_avgs[2])

        # Y axis calibration
        adxl_y_cal = calculate_2_pt_calibration(
            0.0, pos_z_adxl_avgs[1], -1.0, neg_y_adxl_avgs[1])
        lsm_y_cal = calculate_2_pt_calibration(
            0.0, pos_z_lsm_avgs[1], -1.0, neg_y_lsm_avgs[1])

        # X axis calibration
        adxl_x_cal = calculate_2_pt_calibration(
            0.0, pos_z_adxl_avgs[0], -1.0, neg_x_adxl_avgs[0])
        lsm_x_cal = calculate_2_pt_calibration(
            0.0, pos_z_lsm_avgs[0], -1.0, neg_x_lsm_avgs[0])

        print()
        print("Calibrations (offset, scale):")
        print("X:")
        print(f"\tADXL: {adxl_x_cal}")
        print(f"\tLSM:  {lsm_x_cal}")
        print("Y:")
        print(f"\tADXL: {adxl_y_cal}")
        print(f"\tLSM:  {lsm_y_cal}")
        print("Z:")
        print(f"\tADXL: {adxl_z_cal}")
        print(f"\tLSM:  {lsm_z_cal}")
        print()
        e = input("Write calibration to device? [y/n]:")
        if (strtobool(e)):
            print("Writing calibration...")
            set_2_pt_calibration(
                "config.calibration.lsm6dsm.accel.x", lsm_x_cal)
            set_2_pt_calibration(
                "config.calibration.lsm6dsm.accel.y", lsm_y_cal)
            set_2_pt_calibration(
                "config.calibration.lsm6dsm.accel.z", lsm_z_cal)
            set_2_pt_calibration("config.calibration.adxl375.x", adxl_x_cal)
            set_2_pt_calibration("config.calibration.adxl375.y", adxl_y_cal)
            set_2_pt_calibration("config.calibration.adxl375.z", adxl_z_cal)
    else:
        print("Aborted")

    d = Datum(protocol.CMD_ConfigSensorOutput)
    d.load_protobuf(protocol.Command_ConfigSensorOutput(
        rate_hz=0, raw=False))
    tx_queue.put(d)

    print("Cleaned up.")
    exit(0)
    # args.log.close()


# Simple null-offset calibration for the gyro
def calibrate_gyr(args):
    init_cli(args)
    ensure_usb()

    NSAMPLES = 200*4
    gyro_samples = []

    input("Place the device on a stationary surface and press <enter> to begin gyro calibration")

    d = Datum(protocol.CMD_ConfigSensorOutput)
    d.load_protobuf(protocol.Command_ConfigSensorOutput(
        rate_hz=200, raw=True))
    tx_queue.put(d)

    flush_rx()
    for i in tqdm(range(NSAMPLES)):
        d = wait_for_datum([protocol.INFO_SensorData], 0.1)
        if d is None:
            print("Error: timed out!")
            exit(1)
        else:
            pb = d.to_protobuf()
            gyro_samples.append(pb.lsm_gyro_dps)

    avg_gyro_values = np.average(np.array(gyro_samples), axis=0)
    print("Calibration finished.")
    print()
    print("Gyro Offsets:")
    print(f"\tX: {avg_gyro_values[0]}")
    print(f"\tY: {avg_gyro_values[1]}")
    print(f"\tZ: {avg_gyro_values[2]}")

    write = strtobool(input("Write calibration to device? [y/n]:"))
    if (write):
        set_configval(
            "config.calibration.lsm6dsm.gyro.x_offset", -avg_gyro_values[0])
        set_configval(
            "config.calibration.lsm6dsm.gyro.y_offset", -avg_gyro_values[1])
        set_configval(
            "config.calibration.lsm6dsm.gyro.z_offset", -avg_gyro_values[2])
        print("Successfully wrote calibration.")

    d = Datum(protocol.CMD_ConfigSensorOutput)
    d.load_protobuf(protocol.Command_ConfigSensorOutput(
        rate_hz=0, raw=False))
    tx_queue.put(d)

# Simple null-offset calibration for the gyro


def calibrate_altimetry(args):
    init_cli(args)
    ensure_usb()

    NSAMPLES = 256*10
    alt_samples = []
    accel_samples = []

    input("Place the device on a stationary surface and press <enter> to begin altimetry calibration")

    # Run at the rate that the filter will run at!
    d = Datum(protocol.CMD_ConfigSensorOutput)
    d.load_protobuf(protocol.Command_ConfigSensorOutput(
        rate_hz=256, raw=False))
    tx_queue.put(d)

    # repeated float lsm_acceleration_g = 1;
    # repeated float lsm_gyro_dps = 2
    # repeated float adxl_acceleration_g = 3
    # repeated float lis_magnetic_mG = 4
    # required float lps_pressure_hPa = 5

    def hPa2meters(hPa):
        return 44330. * (1 - pow(hPa / 1013.25, 0.190284))

    flush_rx()
    for i in tqdm(range(NSAMPLES)):
        d = wait_for_datum([protocol.INFO_SensorData], 1.0)
        if d is None:
            print("Error: timed out!")
            exit(1)
        else:
            pb = d.to_protobuf()
            accel_samples.append(pb.filtered_world_acceleration_m_s)
            alt_samples.append(hPa2meters(pb.lps_pressure_hPa))

    alt_std = np.std(alt_samples)
    acc_std = np.average(np.std(accel_samples, axis=0))
    print("Calibration finished.")
    print()
    print(f"Acceleration standard deviation: {acc_std} m/s")
    print(f"Barometric altitude standard deviation: {alt_std} m")

    d = Datum(protocol.CMD_ConfigSensorOutput)
    d.load_protobuf(protocol.Command_ConfigSensorOutput(
        rate_hz=0, raw=False))
    tx_queue.put(d)

    write = strtobool(input("Write calibration to device? [y/n]:"))
    if (write):
        set_configval("config.sensors.vert_accel_stdev", acc_std)
        set_configval("config.sensors.altitude_stdev", alt_std)
        print("Successfully wrote calibration.")


def calibrate_mag(args):
    init_cli(args)
    ensure_usb()

    mag_samples = []

    input("Rotate the device around in 3D, try to get it to point in as many directions as possible. Press <enter> to begin")

    stdscr = curses.initscr()
    stdscr.nodelay(True)
    curses.noecho()
    curses.cbreak()

    d = Datum(protocol.CMD_ConfigSensorOutput)
    d.load_protobuf(protocol.Command_ConfigSensorOutput(
        rate_hz=60, raw=True))
    tx_queue.put(d)

    stdscr.addstr(0, 0, "Press any key to finish calibration.")

    flush_rx()
    while (True):
        stdscr.addstr(1, 0, f"{len(mag_samples)} samples collected.")
        stdscr.refresh()
        d = wait_for_datum([protocol.INFO_SensorData], 0.1)
        if d is None:
            print("Error: timed out!")
            exit(1)
        else:
            pb = d.to_protobuf()
            mag_samples.append(list(pb.lis_magnetic_mG))

        if (stdscr.getch() != -1):
            break

    flush_rx()
    d = Datum(protocol.CMD_ConfigSensorOutput)
    d.load_protobuf(protocol.Command_ConfigSensorOutput(
        rate_hz=0, raw=False))
    tx_queue.put(d)

    curses.nocbreak()
    stdscr.keypad(False)
    curses.echo()
    curses.endwin()

    with open("magtest.pickle", 'wb') as f:
        pickle.dump(mag_samples, f)
    print("Magnetometer calibration finished!")
    print()

    cx, cy, cz = calibrate_magnetometer(mag_samples)
    print("Offsets:")
    print(f"\tX: {cx[0]}")
    print(f"\tY: {cy[0]}")
    print(f"\tZ: {cz[0]}")
    print("Scales:")
    print(f"\tX: {cx[1]}")
    print(f"\tY: {cy[1]}")
    print(f"\tZ: {cz[1]}")
    print()

    write = strtobool(input("Write calibration to device? [y/n]:"))
    if (write):
        # set_configval(
        #     "config.calibration.lsm6dsm.gyro.x_offset", -avg_gyro_values[0])
        set_2_pt_calibration("config.calibration.lis3mdl.x", cx)
        set_2_pt_calibration("config.calibration.lis3mdl.y", cy)
        set_2_pt_calibration("config.calibration.lis3mdl.z", cz)
        print("Success!")


def ping(args):
    init_cli(args)

    n = 1
    if (args.repeat):
        n = 100

    for i in range(n):
        if i > 0:
            time.sleep(0.666)

        d = Datum(protocol.CMD_Ping)
        d.load_protobuf(protocol.Command_Ping(link=protocol.USBSerial))

        tx_queue.put(d)
        print("Pinged")

        if i == 0:
            pong = wait_for_datum([protocol.RESP_Ping], 1.5)
            if (pong is None):
                print("Error: timed out")
            else:
                print(
                    f"Got pong! connected over link type '{protocol.LinkID.DESCRIPTOR.values_by_number[pong.to_protobuf().link].name}'")


def devinfo(args):
    init_cli(args)

    d = Datum(protocol.CMD_RequestDeviceInfo)
    tx_queue.put(d)

    print("Requested device info...")
    print()

    resp = wait_for_datum([protocol.RESP_DeviceInfo], 1.5)
    if (resp is None):
        print("Error: timed out")
    else:
        pb = resp.to_protobuf()
        print(
            f"Device Type: {Fore.GREEN}{protocol.TalkerID.DESCRIPTOR.values_by_number[pb.device_type].name}{Style.RESET_ALL}")
        print(
            f"Device name: {Fore.GREEN if pb.HasField('name') else Fore.RED}{pb.name if pb.HasField('name') else '<not set>'}{Style.RESET_ALL}")
        print(
            f"Firmware Version: {Fore.GREEN}{pb.fw_version}{Style.RESET_ALL}")
        print(f"Peripherals:")
        for p in pb.peripherals:
            print(
                f"\t- {Fore.GREEN}{protocol.PeripheralType.DESCRIPTOR.values_by_number[p].name}{Style.RESET_ALL}")


def receiver_config(args):
    # We are talking to the VGPS port now!!
    rx = autodetect_receiver(serial.tools.list_ports.comports())

    vgps = None
    if (args.port):
        vgps = Serial(args.port, baudrate=115200, timeout=0.5)
    elif rx is not None:
        print(f"Configuring receiver on {rx[1].name}")
        vgps = Serial(rx[1].name, baudrate=115200, timeout=0.5)
    else:
        print("Error, no receiver connected")
        exit(1)

    if args.configfile is not None:
        cfgjson = dict2config(json.load(args.configfile))
        if args.spreading_factor is None and 'config.radio.lora.spreading_factor' in cfgjson:
            args.spreading_factor = cfgjson['config.radio.lora.spreading_factor']
        if args.freq_mhz is None and 'config.radio.freq_hz' in cfgjson:
            args.freq_mhz = cfgjson['config.radio.freq_hz'] * 1e-6
        if args.bandwidth is None and 'config.radio.bandwidth' in cfgjson:
            # Convert...
            args.bandwidth = float(
                cfgjson['config.radio.bandwidth'].removesuffix('kHz'))
        if args.power is None and 'config.radio.power' in cfgjson:
            args.power = cfgjson['config.radio.power']

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


def config_get(args):
    init_cli(args)

    if ((args.key not in CONFIG_KEY_ENCODE) and (args.key not in CONFIG_KEY_DECODE)):
        print(f"Error, key {args.key} not found")
        exit(1)

    if args.key in CONFIG_KEY_DECODE:
        args.key = CONFIG_KEY_DECODE[args.key]
    parameter = CONFIG_LOOKUP[args.key]

    d = Datum(protocol.CMD_Config)
    pb = protocol.Config(
        key_hashed=CONFIG_KEY_ENCODE[args.key], mode=protocol.ConfigGet)
    d.load_protobuf(pb)

    tx_queue.put(d)
    resp = wait_for_datum([protocol.RESP_ConfigValue], 5.0)
    # print(resp)
    if (resp is None):
        print("Error: timed out")
    elif ("error" in resp.to_dict()):
        print(f"Error getting config value: {resp.to_dict()['error']}")
    else:
        response = resp.to_protobuf()
        value_path = CONFIG_KEY_DECODE[response.key_hashed]

        value = None
        if (response.HasField("bool_value")):
            value = response.bool_value
        elif (response.HasField("float_value")):
            value = response.float_value
        elif (response.HasField("int_value")):
            value = response.int_value
        elif (response.HasField("string_value")):
            value = response.string_value
        elif (response.HasField("enum_value")):
            value = CONFIG_LOOKUP[value_path].type.values[response.enum_value]

        print(f"{value_path} = {value}")


def config_set(args):
    init_cli(args)

    if ((args.key not in CONFIG_KEY_ENCODE) and (args.key not in CONFIG_KEY_DECODE)):
        print(f"Error, key {args.key} not found")
        exit(1)

    if args.key in CONFIG_KEY_DECODE:
        args.key = CONFIG_KEY_DECODE[args.key]
    parameter = CONFIG_LOOKUP[args.key]

    d = Datum(protocol.CMD_Config)
    pb = protocol.Config(
        key_hashed=CONFIG_KEY_ENCODE[args.key], mode=protocol.ConfigSet)
    value = None
    if isinstance(parameter.type, FloatType):
        value = float(args.value)
    elif isinstance(parameter.type, IntType):
        value = int(args.value)
    elif isinstance(parameter.type, StringType) or isinstance(parameter.type, EnumType):
        value = str(args.value)
    elif isinstance(parameter.type, BoolType):
        value = strtobool(args.value)
    value = parameter.type.parse_value(value)

    if (value is None):
        print("Error parsing value!")
        print(f"Type: {parameter.type}")
        exit(1)

    if isinstance(parameter.type, FloatType):
        pb.float_value = value
    elif isinstance(parameter.type, IntType):
        pb.int_value = value
    elif isinstance(parameter.type, BoolType):
        pb.bool_value = value
    elif isinstance(parameter.type, StringType):
        pb.string_value = value
    elif isinstance(parameter.type, EnumType):
        pb.enum_value = value

    d.load_protobuf(pb)

    # print(d)
    tx_queue.put(d)
    resp = wait_for_datum([protocol.RESP_ConfigSet], 5.0)
    # print(resp)
    if (resp is None):
        print("Error: timed out")
    elif ("error" in resp.to_dict()):
        print(f"Error setting config value: {resp.to_dict()['error']}")
    else:
        print(f"Successfully set {args.key} to {args.value}")


def config_erase(args):
    init_cli(args)

    if (args.key not in CONFIG_KEY_ENCODE):
        print(f"Error, key {args.key} not found")
        exit(1)

    d = Datum(protocol.CMD_Config)
    pb = protocol.Config(
        key_hashed=CONFIG_KEY_ENCODE[args.key], mode=protocol.ConfigErase)
    d.load_protobuf(pb)

    tx_queue.put(d)

    resp = wait_for_datum([protocol.RESP_ConfigSet], 5.0)
    # print(resp)
    if (resp is None):
        print("Error: timed out")
    elif ("error" in resp.to_dict()):
        print(f"Error erasing config value: {resp.to_dict()['error']}")
    else:
        print(f"Successfully erased {args.key}")


def config_reset(args):
    init_cli(args)
    cont = input(
        "Are you sure you want to erase the device's configuration? [y/n]:")
    if (not strtobool(cont)):
        print("Aborting.")
        exit(0)

    # n_erased = 0
    # vnames = []
    # for v in tqdm(flatten_config_values(CONFIGURATION), desc="Erasing configuration"):
    d = Datum(protocol.CMD_Config)
    pb = protocol.Config(
        key_hashed="", mode=protocol.ConfigClear)
    d.load_protobuf(pb)

    tx_queue.put(d)

    resp = wait_for_datum([protocol.RESP_ConfigSet], 5.0)
    if (resp is None):
        print("Error: timed out")
    elif ("error" in resp.to_dict()):
        print(f"Error erasing config value {resp.to_protobuf().error}")


def get_configvals():
    cfgdict = {}

    d = Datum(protocol.CMD_ConfigEnumerate)
    tx_queue.put(d)

    while (True):
        enumval = wait_for_datum(
            # [protocol.RESP_ConfigEnumerateDone, protocol.RESP_ConfigValue], 1.5)
            [protocol.RESP_ConfigEnumerateDone, protocol.RESP_ConfigValue], 10.0)
        if (enumval is None):
            print("Error: timed out")
            break
        elif enumval.typeid == protocol.RESP_ConfigEnumerateDone:
            return cfgdict
        elif enumval.typeid == protocol.RESP_ConfigValue:
            # val = enumval.to_protobuf()
            response = enumval.to_protobuf()
            value_path = CONFIG_KEY_DECODE[response.key_hashed]

            value = None
            if (response.HasField("bool_value")):
                value = response.bool_value
            elif (response.HasField("float_value")):
                value = response.float_value
            elif (response.HasField("int_value")):
                value = response.int_value
            elif (response.HasField("string_value")):
                value = response.string_value
            elif (response.HasField("enum_value")):
                value = CONFIG_LOOKUP[value_path].type.values[response.enum_value].name

            # print(f"{value_path} = {value}")
            cfgdict[value_path] = value


def config_list(args):
    init_cli(args)

    print("Requesting device configuration...\n")
    d = get_configvals()
    for k, v in sorted(d.items()):
        if (not args.calibration) and k.startswith("config.calibration"):
            continue
        vv = f"\"{v}\"" if isinstance(v, str) else v
        print(f"{k} = {Fore.GREEN}{vv}{Style.RESET_ALL}")


def config_list_default(args):
    init_cli(args)
    d = CONFIG_LOOKUP
    for k, v in sorted(d.items()):
        if (not args.calibration) and k.startswith("config.calibration"):
            continue
        # print(str(v))
        print(
            f"{Fore.GREEN}{v.path}{Style.RESET_ALL} {Fore.YELLOW}({encode_key(v.path)}){Style.RESET_ALL}")
        print(f"\tname: {v.name}")
        if ('!description' in v.data):
            print(f"\tdesc: {v.data['!description']}")
        print(f"\ttype: {v.type}")
        suf = ""
        if '!suffix' in v.data:
            suf = v.data['!suffix']
            print(f"\tunits: {suf}F")
        vv = f"\"{v.default}\"" if isinstance(v.default, str) else v.default
        if isinstance(v.type, EnumType):
            vv = str(v.type.values[v.default])

        print(f"\tdefault: {Fore.RED}{vv}{suf}{Style.RESET_ALL}")
        print("\n")


def config_save(args):
    init_cli(args)

    print("Requesting device configuration...\n")
    d = config2dict(get_configvals())
    print("Saving configuration...")
    json.dump(d, args.file, indent=4)
    args.file.flush()

    print("Done!")


def config_diff(args):
    init_cli(args)
    print("Requesting device configuration...\n")
    d_local = dict2config(json.load(args.file))
    d_remote = get_configvals()

    nodiff = True
    for k, lv, rv in [(k, d_local.get(k, None), d_remote.get(k, None)) for k in sorted(d_local.keys() | d_remote.keys())]:
        if (lv != rv):
            lstr = f"{lv:{'' if (lv is None)or isinstance(lv, str) else '.4f'}}"
            rstr = f"{rv:{'' if (rv is None) or isinstance(rv, str) else '.4f'}}"
            print(
                f"{k} = {Fore.RED}{lstr} (local){Style.RESET_ALL} -> {Fore.GREEN}{rstr} (remote){Style.RESET_ALL}")
            nodiff = False

    if nodiff:
        print(Fore.GREEN + "No differences!" + Style.RESET_ALL)


def config_load(args):
    init_cli(args)

    d = json.load(args.file)
    cfg = dict2config(d)

    # print(json.dumps(cfg, indent=4))
    # Validate configuration
    for path, v in cfg.items():
        if (path not in CONFIG_LOOKUP):
            print("Error, unknown configuration key '{path}'")
            exit(1)
        vparsed = CONFIG_LOOKUP[path].type.parse_value(v)
        if (vparsed is None):
            print("Error, could not parse value '{v}' for key '{path}'")

    print("Configuration valid!")

    if (not args.dryrun):
        cont = args.yes or strtobool(
            input("Write configuration to device? [y/n]:"))
        if cont:
            for k, v in cfg.items():
                if not set_configval(k, v):
                    print("Error setting configuration value with key '{k}'")
                    print("Aborted")
                    exit(1)
            print("Successfully wrote configuration!")
        else:
            print("Aborted")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog='cli',
        description='CLI interface for rocket tracking system components (receiver and tracker)')
    subparsers = parser.add_subparsers(help='sub-command help')
    subparsers.required = True
    parser.add_argument('-p', '--port', required=False, type=str,
                        help="Serial port of tracker/receiver. If not specified, tracker/receiver will be automatically detected")
    parser.add_argument('-v', '--verbose', action='store_true',
                        help="Prints all messages to/from the connected device")

    parser_rxconfig = subparsers.add_parser(
        'rxconfig', description="Set configuration parameters of a receiver")
    parser_rxconfig.set_defaults(func=receiver_config)
    parser_rxconfig.add_argument(
        '-f', '--freq-mhz', type=argparse_restricted_float(903.0, 927.0))
    parser_rxconfig.add_argument(
        '-sf', '--spreading-factor', type=int, choices=[6, 7, 8, 9, 10, 11, 12])
    parser_rxconfig.add_argument(
        '-bw', '--bandwidth', type=str, choices=["500", "250", "125", "62.5", "41.7", "31.25", "20.8", "15.6", "10.4", "7.8"])
    parser_rxconfig.add_argument(
        '-p', '--power', type=int, choices=[3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 20])
    parser_rxconfig.add_argument(
        '-c', '--configfile', type=argparse.FileType('r'))

    parser_monitor = subparsers.add_parser(
        'monitor', description="View the stream of incoming data from a connected tracker or receiver")
    parser_monitor.add_argument(
        '-l', '--log', type=argparse.FileType('a'), help="Write all received data (JSON) to a file")
    parser_monitor.set_defaults(func=monitor)
    parser_monitor.add_argument(
        '-u', '--udp', default=None, type=str, help="Send all received data (JSON) to ip:port")

    parser_debug = subparsers.add_parser(
        'debug', description="View the stream of debug data from a tracker")
    parser_debug.set_defaults(func=debug)
    parser_debug.add_argument(
        '-u', '--udp', default=None, type=str, help="Send all received data (JSON) to ip:port")

    parser_reboot = subparsers.add_parser(
        'reboot', description="Reboot a connected tracker")
    parser_reboot.set_defaults(func=reboot)
    parser_reboot.add_argument(
        "-y", "--yes", action='store_true', help="bypass confirmation dialog")

    parser_calib = subparsers.add_parser(
        'calibrate', description="Sensor calibration tools. USB connection required.")
    calib_subparsers = parser_calib.add_subparsers()
    calib_subparsers.required = True

    parser_calib_acc = calib_subparsers.add_parser(
        "accel", description="Calibrate the accelerometer of a connected tracker")
    parser_calib_acc.set_defaults(func=calibrate_acc)

    parser_calib_reset = calib_subparsers.add_parser(
        "reset", description="Reset all calibration parameters")
    parser_calib_reset.set_defaults(func=calibrate_reset)
    parser_calib_reset.add_argument("-y", "--yes", action='store_true')

    parser_calib_gyr = calib_subparsers.add_parser(
        "gyro", description="Calibrate the gyro of a connected tracker")
    parser_calib_gyr.set_defaults(func=calibrate_gyr)

    parser_calib_mag = calib_subparsers.add_parser(
        "mag", description="Calibrate the magnetometer of a connected tracker")
    parser_calib_mag.set_defaults(func=calibrate_mag)

    parser_calib_mag = calib_subparsers.add_parser(
        "altimetry", description="Calibrate the altimetry filter of a connected tracker")
    parser_calib_mag.set_defaults(func=calibrate_altimetry)

    parser_sensormon = subparsers.add_parser(
        'sensormon', description="View sensor data from a connected tracker. USB connection required.")
    parser_sensormon.set_defaults(func=sensormon)
    parser_sensormon.add_argument(
        "-f", "--frequency", default=60, type=argparse_restricted_int(1, 256), help="Update rate of sensor data")
    parser_sensormon.add_argument(
        '-l', '--log', type=argparse.FileType('w'), help="Write all received data (JSON) to log file")
    parser_sensormon.add_argument(
        '-r', '--raw', action='store_true', help="Raw sensor output (disable calibration)")
    parser_sensormon.add_argument(
        '-u', '--udp', default=None, type=str, help="Send all received data (JSON) to ip:port")

    parser_telemetry = subparsers.add_parser(
        'telemetry', description="View telemetry information from a connected tracker")
    parser_telemetry.set_defaults(func=telemetry)
    parser_telemetry.add_argument(
        '-u', '--udp', default=None, type=str, help="Send all received data (JSON) to ip:port")
    parser_telemetry.add_argument(
        '-l', '--log', type=argparse.FileType('w'), help="Write all received data (JSON) to log file")

    parser_3d = subparsers.add_parser(
        'sensors3d', description="Visualize the orientation of a connected tracker in 3D. Requires USB connection and a web browser with OpenGL support")
    parser_3d.set_defaults(func=sensors3d)
    parser_3d.add_argument(
        "-f", "--frequency", default=60, type=argparse_restricted_int(1, 256), help="Update rate of orientation information")

    parser_ping = subparsers.add_parser(
        'ping', description="Test the connection to a tracker")
    parser_ping.set_defaults(func=ping)
    parser_ping.add_argument('-r', '--repeat', action='store_true')

    parser_info = subparsers.add_parser(
        'info', description="Get the board type, name, and firmware revision of a connected tracker")
    parser_info.set_defaults(func=devinfo)

    # TODO: Add proper help/usage for __ALL__ commands and subcommands!!!
    parser_log = subparsers.add_parser(
        'log', description="Logging commands, configuration and downloading")
    log_subparsers = parser_log.add_subparsers()
    log_subparsers.required = True
    # parser_log.add_argument('bar', type=int, help='bar help')

    parser_log_stop = log_subparsers.add_parser(
        'stop', description="Stop logging")
    parser_log_stop.set_defaults(func=log_stop)
    parser_log_stop.add_argument("-y", "--yes", action='store_true')

    # parser_log_mark = log_subparsers.add_parser('mark')
    # parser_log_mark.set_defaults(func=log_mark)
    # parser_log_mark.add_argument('data', nargs='?', default='DEADBEEF')

    parser_log_start = log_subparsers.add_parser(
        'start', description="Manually start logging at a specified rate")
    parser_log_start.set_defaults(func=log_start)
    parser_log_start.add_argument(
        "hz", type=argparse_restricted_int(0, 200))

    parser_log_arm = log_subparsers.add_parser(
        'arm', description="Arm logging")
    parser_log_arm.set_defaults(func=log_arm)

    parser_log_delete = log_subparsers.add_parser(
        'delete', description="Delete all logs stored in a tracker's log memory")
    parser_log_delete.set_defaults(func=log_delete)
    parser_log_delete.add_argument(
        "-y", "--yes", action='store_true', help="bypass confirmation dialog")

    parser_log_clean = log_subparsers.add_parser(
        'clean', description="Erase a tracker's entire log memory")
    parser_log_clean.set_defaults(func=log_clean)
    parser_log_clean.add_argument(
        "-y", "--yes", action='store_true', help="bypass confirmation dialog")

    parser_log_status = log_subparsers.add_parser(
        'status', description="Request a tracker's current logging mode and rate")
    parser_log_status.set_defaults(func=log_status)

    parser_log_download = log_subparsers.add_parser(
        'download', description="Download the contents of a connected tracker's log memory. USB connection required.")
    parser_log_download.add_argument(
        '-o', '--outfile', required=True, type=argparse.FileType('wb'), help="Output file")
    parser_log_download.add_argument(
        '-r', '--raw', action='store_true', help="Dump log memory in raw format (pickle file of received frames)")
    # TODO: Log parsing and output formats!
    parser_log_download.set_defaults(func=log_download)

    parser_config = subparsers.add_parser(
        'config', description="Tracker configuration tools")
    config_subparsers = parser_config.add_subparsers()
    config_subparsers.required = True

    parser_config_get = config_subparsers.add_parser(
        'get', description="Retrieve the value of a specific configuration parameter")
    parser_config_get.set_defaults(func=config_get)
    parser_config_get.add_argument(
        "key", type=str)

    parser_config_set = config_subparsers.add_parser(
        'set', description="Set the value of a specific configuration parameter")
    parser_config_set.set_defaults(func=config_set)
    parser_config_set.add_argument("key", type=str)
    parser_config_set.add_argument("value", type=str)

    parser_config_erase = config_subparsers.add_parser(
        'erase', description="Erase (set to default) a single configuration value")
    parser_config_erase.set_defaults(func=config_erase)
    parser_config_erase.add_argument("key", type=str)

    parser_config_reset = config_subparsers.add_parser(
        'reset', description="Erase configuration memory, all values will be restored to default")
    parser_config_reset.set_defaults(func=config_reset)

    parser_config_list = config_subparsers.add_parser(
        'list', description="List the keys and values of all configuration parameters of a attached tracker")
    parser_config_list.set_defaults(func=config_list)
    parser_config_list.add_argument(
        "-c", "--calibration", action='store_true', help="Display calibration parameters")

    parser_config_dlist = config_subparsers.add_parser(
        'list_defaults', description="List the keys and default values for all available configuration values")
    parser_config_dlist.set_defaults(func=config_list_default)
    parser_config_dlist.add_argument(
        "-c", "--calibration", action='store_true', help="Display calibration parameters")

    parser_config_save = config_subparsers.add_parser(
        'save', description="Save configuration to a JSON file")
    parser_config_save.set_defaults(func=config_save)
    parser_config_save.add_argument(
        "file", type=argparse.FileType('w'), help="Output file")

    parser_config_load = config_subparsers.add_parser(
        'load', description="Load and write configuration from a JSON file")
    parser_config_load.set_defaults(func=config_load)
    parser_config_load.add_argument(
        "file", type=argparse.FileType('r'), help="Input file")
    parser_config_load.add_argument(
        "-d", "--dryrun", action='store_true', help="Dry-run mode, only validates configuration file")
    parser_config_load.add_argument(
        "-y", "--yes", action='store_true', help="bypass confirmation dialog")

    parser_config_diff = config_subparsers.add_parser(
        'diff', description="Show the differences between a connected device's configuration, and the configuration stored in a JSON file")
    parser_config_diff.set_defaults(func=config_diff)
    parser_config_diff.add_argument(
        "file", type=argparse.FileType('r'), help="Input file")

    parser_util = subparsers.add_parser(
        'util', description="Miscellaneous Tools")
    util_subparsers = parser_util.add_subparsers()
    util_subparsers.required = True

    parser_loginfo = util_subparsers.add_parser(
        'log_info', description="Summary of a log memory dump")
    parser_loginfo.set_defaults(func=log_info)
    parser_loginfo.add_argument("logfile", type=argparse.FileType('rb'))
    parser_loginfo.add_argument(
        "-r", "--raw", action='store_true', help="Preserve raw timestamps")

    parser_logconvert = util_subparsers.add_parser(
        'log_convert', description="Converts a log memory dump (download) to various formats")
    parser_logconvert.set_defaults(func=log_convert)
    parser_logconvert.add_argument("input", type=argparse.FileType('rb'))
    parser_logconvert.add_argument(
        "-e", "--events", action='store_true', help="Preserve events in log export")
    parser_logconvert.add_argument(
        "-r", "--raw", action='store_true', help="Preserve raw timestamps")
    parser_logconvert.add_argument(
        "-s", "--sublog", type=int, help="Split the memory dump into multiple logs, and select one (indices start at 0)")
    parser_logconvert.add_argument(
        "-c", "--calibrate", type=argparse.FileType('r'), help="Apply calibration to the log using a JSON configuration file")
    parser_logconvert.add_argument(
        "-o", "--output", required=False, type=argparse.FileType('w'), help="Output file")
    parser_logconvert.add_argument(
        "-f", "--format", required=True, choices=['json', 'csv', 'gpx'], help="Export format")

    opts = parser.parse_args()
    opts.func(opts)
