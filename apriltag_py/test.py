# -*- coding: utf-8 -*-

import os
import cv2
import numpy as np
import matplotlib.pyplot as plt

# %%


def imshow(*args, **kwargs):
    n = len(args)
    figsize = kwargs.get('figsize', (7.5, 7.5))
    cmap = kwargs.get('cmap', plt.cm.gray)
    title = kwargs.get('title', [])
    if isinstance(title, basestring):
        title = [title]

    f, axes = plt.subplots(nrows=1, ncols=n, figsize=figsize)
    axes = np.atleast_1d(axes)
    for ax, im in zip(axes, args):
        ax.imshow(im, cmap=cmap)

    for ax, t in zip(axes, title):
        ax.set_title(t)
    return np.ravel(axes)

# %%
cwd = os.getcwd()
image_file = os.path.join(cwd, 'frame0001.jpg')
color = cv2.imread(image_file, cv2.IMREAD_COLOR)
gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
imshow(color, title='raw')

# %%
# Gaussian blur with downsample
gray_sample = gray
gray_seg = cv2.pyrDown(gray)
imshow(gray_seg, title='seg')

# %%
# Calculate image gradient
scale = 1
Ix = cv2.Scharr(gray_seg, cv2.CV_32F, 1, 0, scale=scale)
Iy = cv2.Scharr(gray_seg, cv2.CV_32F, 0, 1, scale=scale)
ax1, ax2 = imshow(Ix, Iy, figsize=(16, 10), title=('Ix', 'Iy'))

# %%
# Calculate gradient magnitude and angle
im_mag, im_ang = cv2.cartToPolar(Ix, Iy)
imshow(im_mag, im_ang, figsize=(16, 10), title=('mag', 'ang'))
print('max mag:', np.max(im_mag))

f, ax = plt.subplots(figsize=(12, 3))
ax.hist(im_mag.ravel(), 255)
plt.show()
