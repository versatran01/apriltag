import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
from timeit import default_timer as timer
from collections import OrderedDict
    
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
fsize2 = (16, 10)
times = OrderedDict()

# %%
cwd = os.getcwd()
image_file = os.path.join(cwd, 'frame0001.jpg')
color = cv2.imread(image_file, cv2.IMREAD_COLOR)
gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
imshow(color, title='raw')

# %%
# Gaussian blur with downsample
gray_sample = gray

start = timer()
gray_seg = cv2.pyrDown(gray)
times['1_pyrdown'] = timer() - start

imshow(gray_seg, title='seg')

# %%
# Calculate image gradient
scale = 1 / 16.0

start = timer()
Ix = cv2.Scharr(gray_seg, cv2.CV_32F, 1, 0, scale=scale)
Iy = cv2.Scharr(gray_seg, cv2.CV_32F, 0, 1, scale=scale)
times['2_scharr'] = timer() - start

ax1, ax2 = imshow(Ix, Iy, figsize=fsize2, title=('Ix', 'Iy'))

# %%
# Calculate gradient magnitude and angle

start = timer()
im_mag, im_ang = cv2.cartToPolar(Ix, Iy)
times['3_polar'] = timer() - start

imshow(im_mag, im_ang, figsize=fsize2, title=('mag', 'ang'))

# %%
mag_mean = np.mean(im_mag)
mag_median = np.median(im_mag)
mag_meme = (mag_mean + mag_median) / 2.0
f, ax = plt.subplots(figsize=(12, 3))
ax.hist(im_mag.ravel(), 255)
ax.axvline(mag_mean, color='m')
ax.axvline(mag_median, color='r')
plt.show()

# %%
num_pixels = np.size(im_mag)
mask = im_mag > mag_mean
num_mask = np.count_nonzero(mask)
imshow(mask,  title='mean: {}/{}'.format(num_mask, num_pixels))

# %%



# %%
total_time = 0.0
for key, value in times.iteritems():
    value *= 1e3
    total_time += value
    print(key, value)
print('total', total_time)