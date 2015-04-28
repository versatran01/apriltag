from __future__ import absolute_import, division, print_function
from pyx import canvas, path, color, style, deco
from apriltag_gen import tag_codes
import numpy as np
import yaml


def create_apriltag_grid(args):
    grid_canvas = canvas.canvas()

    # actual drawing
    draw_apriltag_grid(grid_canvas, args.rows, args.cols, args.tag_family,
                       args.black_border, args.tag_size, args.tag_spacing)

    if args.yaml:
        stream = file('test.yaml', 'w')
        test_yaml(args.rows, args.cols, args.tag_family, args.tag_size,
                  args.tag_spacing, stream)
        stream.close()

    # write to file
    grid_canvas.writePDFfile(args.output)
    print('Apriltag grid written to pdf: {0}'.format(args.output))
    if args.eps:
        grid_canvas.writeEPSfile(args.output)
        print('Apriltag grid written to eps: {0}'.format(args.output))


def test_yaml(rows, cols, tag_family, tag_size, tag_spacing, stream):
    tf = tag_codes.ApriltagFamily(tag_family)

    apriltag_map = dict()
    apriltag_map['tag_family'] = tag_family
    apriltag_map['tag_size'] = tag_size
    apriltag_list = []

    for r in range(rows):
        for c in range(cols):
            i = r * cols + c
            tag_code = tf.tag_codes[i]
            x = c * (1 + tag_spacing) * tag_size
            y = r * (1 + tag_spacing) * tag_size
            apriltag = dict()
            apriltag['id'] = i
            apriltag['code'] = tag_code
            apriltag['position'] = dict()
            apriltag['position']['x'] = x + tag_size / 2
            apriltag['position']['y'] = y + tag_size / 2
            apriltag['position']['z'] = 0.0
            apriltag['orientation'] = dict()
            apriltag['orientation']['w'] = 1.0
            apriltag['orientation']['x'] = 0.0
            apriltag['orientation']['y'] = 0.0
            apriltag['orientation']['z'] = 0.0
            apriltag_list.append(apriltag)

    apriltag_map['tags'] = apriltag_list
    yaml.dump(apriltag_map, stream)


def draw_apriltag_grid(grid_canvas, rows, cols, tag_family, black_border,
                       tag_size, tag_spacing, show_id=True):
    # Get tag family
    tf = tag_codes.ApriltagFamily(tag_family)

    # convert tag_size to cm
    tag_size_cm = tag_size * 100

    for r in range(rows):
        for c in range(cols):
            i = r * cols + c
            tag_code = tf.tag_codes[i]
            x = c * (1 + tag_spacing) * tag_size_cm
            y = r * (1 + tag_spacing) * tag_size_cm
            draw_apriltag(grid_canvas, tag_code, tf.tag_bits, black_border,
                          tag_size_cm, x, y)
            if show_id:
                grid_canvas.text(x, y - 0.5 * tag_size_cm * tag_spacing,
                                 "{0}:{1}".format(i, tag_code))

    draw_axis_and_caption(grid_canvas, rows, cols, tag_size_cm, tag_spacing,
                          1.5)


def draw_axis_and_caption(grid_canvas, rows, cols, tag_size, tag_spacing, k):
    x = -k * tag_size * tag_spacing
    y = x
    axis_length = tag_size / 4

    draw_xy_axis(grid_canvas, x, y, axis_length)
    caption = "{0}x{1} tags, size={2}cm and spacing={3}cm".format(
        rows, cols, tag_size, tag_spacing * tag_size)
    grid_canvas.text(x + 0.5 * tag_size, y, caption, [])


def draw_xy_axis(grid_canvas, x, y, axis_length):
    draw_axis(grid_canvas, 'x', x, y, axis_length, color.rgb.red)
    draw_axis(grid_canvas, 'y', x, y, axis_length, color.rgb.green)


def draw_axis(grid_canvas, text, x, y, l, axis_color):
    if text is 'x':
        axis = path.line(x, y, x + l, y)
        grid_canvas.text(x + l * 1.1, y, text)
    elif text is 'y':
        axis = path.line(x, y, x, y + l)
        grid_canvas.text(x, y + l * 1.1, text)

    arrow_size = l / 10
    grid_canvas.stroke(axis,
                       [axis_color, arrow_attribute(axis_color, arrow_size)])


def arrow_attribute(arrow_color, arrow_size):
    return deco.earrow([deco.stroked([arrow_color, style.linejoin.round]),
                        deco.filled([arrow_color])], size=arrow_size)


def draw_apriltag(grid_canvas, tag_code, tag_bits, black_border, tag_size, x,
                  y):
    tag_matrix = tag_code_to_matrix(tag_code, tag_bits, black_border)
    draw_tag_matrix(grid_canvas, tag_matrix, tag_size, x, y)


def tag_code_to_matrix(tag_code, tag_bits, black_border):
    total_bits = tag_bits + black_border * 2
    tag_matrix = np.zeros([total_bits, total_bits], dtype=int)
    for i in range(0, tag_bits):
        for j in range(0, tag_bits):
            # This is so that when you print the tag matrix, it will look the
            # the same as the tag
            shift = tag_bits * (tag_bits - i) - j - 1
            if tag_code & (1 << shift):
                tag_matrix[i + black_border, j + black_border] = 1

    return tag_matrix


def draw_tag_matrix(grid_canvas, tag_matrix, tag_size, x, y):
    total_bits = np.size(tag_matrix, 0)
    bit_size = tag_size / total_bits
    for i in range(total_bits):
        for j in range(total_bits):
            if tag_matrix[i, j] == 0:
                x_bit = x + bit_size * j
                # i is flipped in y
                y_bit = y + bit_size * (total_bits - i - 1)
                draw_square_bit(grid_canvas, x_bit, y_bit, bit_size)


def draw_square_bit(tag_canvas, x, y, bit_size):
    tag_canvas.fill(path.rect(x, y, bit_size, bit_size), [color.rgb.black])
