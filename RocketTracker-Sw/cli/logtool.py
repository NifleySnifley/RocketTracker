#!/usr/bin/env python3

import loglib
import argparse
from pathlib import Path

LOG_MEMORY_SIZE_B = 32000000


def log_info(args):
    log = loglib.RTRKLog()
    log.load_raw_pickle(args.logfile)

    print(f"Log '{args.logfile.name}'")
    print(
        f"Size: {log.raw_size_bytes/1000}KB ({(log.raw_size_bytes/LOG_MEMORY_SIZE_B)*100.0:.2f}% of log memory)")
    sublogs = log.split_logs()

    totaltime = sum([l.get_log_duration_sec() for l in sublogs])
    print(f"Total time: {totaltime:0.2f} seconds")
    print(
        f"Log dump contains {len(sublogs)} log{'' if len(sublogs) == 1 else 's'}")
    for i, sl in enumerate(sublogs):
        sl.normalize()
        sl.sort()
        print(f"- Log {i}:")
        print(f"\t{sl.get_log_duration_sec():.2f} seconds")
        events = sl.get_events()
        if len(events):
            print(f"\tEvents:")
            for e in events:
                print(f"\t\t- {e}")
        else:
            print(f"\tNo events")


def log_convert(args):
    infile = args.input
    outfile = args.output
    fmt = args.format

    log = loglib.RTRKLog()
    log.load_raw_pickle(infile)
    if (args.sublog is not None):
        sublogs = log.split_logs()
        if (args.sublog < (-len(sublogs)) or args.sublog >= len(sublogs)):
            print(f"Error, invalid sublog index {args.sublog}")
            exit(1)
        else:
            log = sublogs[args.sublog]

    if not args.raw:
        log.normalize()
        log.sort()

    if args.calibrate is not None:
        log.calibrate(args.calibrate)

    if not args.events:
        log.frames = [f for f in log.frames if isinstance(
            f.data, loglib.LogDataDefault)]

    if (outfile is None):
        inpath = Path(infile.name)
        outpath = inpath.parent / Path(str(inpath.stem)+f".{fmt}")
        outfile = open(outpath, 'w')

    if fmt == "json":
        log.export_json(outfile)
    elif fmt == "csv":
        n = outfile.name
        outfile.close()
        log.export_csv(n)
    elif fmt == "gpx":
        log.export_gpx(outfile)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog='logtool',
        description='A tool for inspecting, converting, and analyzing tracker logs')
    subparsers = parser.add_subparsers(help='sub-command help')

    parser_info = subparsers.add_parser('info')
    parser_info.set_defaults(func=log_info)
    parser_info.add_argument("logfile", type=argparse.FileType('rb'))

    parser_convert = subparsers.add_parser('convert')
    parser_convert.set_defaults(func=log_convert)
    parser_convert.add_argument("input", type=argparse.FileType('rb'))
    parser_convert.add_argument("-e", "--events", action='store_true')
    parser_convert.add_argument("-r", "--raw", action='store_true')
    parser_convert.add_argument("-s", "--sublog", type=int)
    parser_convert.add_argument(
        "-c", "--calibrate", type=argparse.FileType('r'))
    parser_convert.add_argument(
        "-o", "--output", required=False, type=argparse.FileType('w'))
    parser_convert.add_argument(
        "-f", "--format", required=True, choices=['json', 'csv', 'gpx'])

    opts = parser.parse_args()
    opts.func(opts)
