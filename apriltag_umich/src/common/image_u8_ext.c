#include "image_u8_ext.h"

#include <stdlib.h>
#include <string.h>

image_u8_t *image_u8_create_from_gray(unsigned int width, unsigned int height,
                                      uint8_t *gray) {
  int stride = width;
  uint8_t *buf = calloc(height * stride, sizeof(uint8_t));
  // const initializer
  image_u8_t tmp = {
      .width = width, .height = height, .stride = stride, .buf = buf};
  image_u8_t *im = calloc(1, sizeof(image_u8_t));
  memcpy(im, &tmp, sizeof(image_u8_t));
  memcpy(im->buf, gray, im->height * im->stride);

  return im;
}
