from __future__ import absolute_import, division, print_function
from pyx import canvas, path, color, style
import numpy as np


def draw_apriltag(tag_canvas, tag_code, tag_bits, black_border, tag_size, x, y):
    tag_matrix = tag_code_to_matrix(tag_code, tag_bits, black_border)
    draw_tag_matrix(tag_canvas, tag_matrix, tag_size, x, y)


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


def draw_tag_matrix(tag_canvas, tag_matrix, tag_size, x, y):
    total_bits = np.size(tag_matrix, 0)
    bit_size = tag_size / total_bits
    for i in range(total_bits):
        for j in range(total_bits):
            if tag_matrix[i, j] == 0:
                x_bit = x + bit_size * j
                # i is flipped in y
                y_bit = y + bit_size * (total_bits - i - 1)
                draw_square_bit(tag_canvas, x_bit, y_bit, bit_size)


def draw_square_bit(tag_canvas, x, y, bit_size):
    tag_canvas.fill(path.rect(x, y, bit_size, bit_size), [color.rgb.black])

