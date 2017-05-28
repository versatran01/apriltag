#ifndef _IMAGE_U8_EXT_H
#define _IMAGE_U8_EXT_H

#include "image_u8.h"

#ifdef __cplusplus
extern "C" {
#endif

image_u8_t *image_u8_create_from_gray(unsigned int width, unsigned int height,
                                      uint8_t *gray);

#ifdef __cplusplus
}
#endif
#endif // _IMAGE_U8_EXT_H
