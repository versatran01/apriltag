#!/usr/bin/python2

import sys
import argparse as ap
from apriltag_gen import apriltag_gen


def main():
    parser = ap.ArgumentParser(description='Genrate PDFs of apriltag grid')
    output_args = parser.add_argument_group('Output arguments')
    output_args.add_argument('output', nargs='?', default='april_grid',
                             help='output filename')
    output_args.add_argument('--eps', '-e', action='store_true',
                             help='Also output an eps file')
    output_args.add_argument('--yaml', '-y', action='store_true',
                             help='Also output an yaml file')

    apriltag_args = parser.add_argument_group('Apriltag arguments')
    apriltag_args.add_argument('--rows', '-r', type=int, default=7,
                               help='The number of rows of this grid')
    apriltag_args.add_argument('--cols', '-c', type=int, default=5,
                               help='The number of cols of this grid')
    apriltag_args.add_argument('--tag-family', '-f', default='t36h11',
                               choices=['t36h11', 't25h9', 't16h5'],
                               help='Tag family of Apriltag')
    apriltag_args.add_argument('--black-border', '-b', type=int, default=1,
                               help='The black border of tag in bit')
    apriltag_args.add_argument('--tag-size', '-s', type=float, default=0.035,
                               help='The size of one tag in [m]')
    apriltag_args.add_argument('--tag-spacing', '-p', type=float, default=0.2,
                               help='The space between tags in fraction of'
                                    ' the tag size')
    apriltag_args.add_argument('--show-id', '-i', action='store_true',
                               help='Whether to print id under the tag')

    args = parser.parse_args()
    print(args)
    apriltag_gen.create_apriltag_grid(args)


if __name__ == "__main__":
    sys.exit(main())
