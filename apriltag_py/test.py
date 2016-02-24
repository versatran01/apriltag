# -*- coding: utf-8 -*-

import os
import cv2
import numpy as np
import matplotlib.pyplot as plt

# %%


def imshow(image, cm=None, title=None, figsize=None):
    fig = plt.figure(figsize=figsize)
    ax = fig.add_subplot(111)
    ax.imshow(image, cmap=cm)
    ax.set_title(title)


# %%
cwd = os.getcwd()
image = cv2.imread(os.path.join(cwd, 'frame0000.jpg'))
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
imshow(image, title='original')

# %%
# Gaussian blur
sigma = 0.8
ksize = 3
gray_blur = cv2.GaussianBlur(gray, (ksize, ksize), sigma)
gray_diff = gray - gray_blur
imshow(gray_blur, cm=plt.cm.gray, title='blur')
imshow(gray_diff, cm=plt.cm.gray, title='noise')

# %%