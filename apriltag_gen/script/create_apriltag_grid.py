#!/usr/bin/python

import sys
import argparse as ap


def create_apriltag_grid(args):
    pass


def main():
    parser = ap.ArgumentParser(description='Genrate PDFs of apriltag grid')
    output_args = parser.add_argument_group('Output arguments')
    output_args.add_argument('output', nargs='?', default='target',
                             help='output filename')
    output_args.add_argument('--eps', action='store_true', dest='do_eps',
                             help='Also output an eps file')

    apriltag_args = parser.add_argument_group('Apriltag arguments')
    apriltag_args.add_argument('--rows', '-r', type=int, default=7,
                               help='The number of rows of this grid')
    apriltag_args.add_argument('--cols', '-c', type=int, default=5,
                               help='The number of cols of this grid')
    apriltag_args.add_argument('--tag-size', '-s', type=float, default=0.035,
                               help='The size of one tag in [m]')
    apriltag_args.add_argument('--tag-space', '-p', type=float, default=0.2,
                               help='The space between tags in fraction of'
                                    ' the tag size')
    apriltag_args.add_argument('--tag-family', '-f', default='t36h11',
                               choices=['t36h11', 't25h9', 't16h5'],
                               help='Tag family of Apriltag')

    if len(sys.argv) == 1:
        parser.print_help()
        return

    args = parser.parse_args()
    create_apritlag_grid(args)


if __name__ == "__main__":
    sys.exit(main())