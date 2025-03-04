#!/usr/bin/python
"""
  Demo example how to read log file
"""
import pathlib
from datetime import timedelta

from osgar.logger import LogReader, lookup_stream_id
from osgar.lib.serialize import deserialize


def read_logfile(logfile, stream):
    only_stream = lookup_stream_id(logfile, stream)
    with LogReader(logfile, only_stream_id=only_stream) as log:
        for timestamp, stream_id, data in log:
            buf = deserialize(data)
            assert 0, buf


def main():
    import argparse

    parser = argparse.ArgumentParser(description='Extract data from logfile')
    parser.add_argument('logfile', help='recorded log file')
    parser.add_argument('--stream', help='stream ID or name', default='oak.depth')
    args = parser.parse_args()

    read_logfile(args.logfile, args.stream)


if __name__ == "__main__":
    main()

# vim: expandtab sw=4 ts=4 

