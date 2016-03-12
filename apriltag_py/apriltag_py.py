import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
from timeit import default_timer as timer
from collections import OrderedDict
from numba import jit
    
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
    
@jit
def mod2pi(theta):
  twopi = np.pi * 2.0
  twopi_inv = 1.0 / twopi
  
  abs_theta = abs(theta)
  q = abs_theta * twopi_inv + 0.5
  qi = int(q)
  
  r = abs_theta - qi * twopi;
  if theta < 0:
      return -r
  else:
      return r

# %%
fsize2 = (16, 10)
times = OrderedDict()

# %%
cwd = os.getcwd()
image_file = os.path.join(cwd, 'frame0001.png')
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
MIN_MAG = mag_mean
num_pixels = np.size(im_mag)
mask = im_mag > MIN_MAG
b = 5

mask[:, :b] = 0
mask[:, -b:] = 0
mask[:b] = 0
mask[-b:] = 0

num_mask = np.count_nonzero(mask)
imshow(mask,  title='mean: {}/{}'.format(num_mask, num_pixels))

# %%
edges = []
h, w = np.shape(im_mag)

start = timer()
mr, mc = np.nonzero(mask)

MAX_ANG_DIFF = np.deg2rad(1)

for r, c in zip(mr, mc):
    mag0 = im_mag[r, c]
    ang0 = im_ang[r, c]
    
    pid0 = r * w + c
    pid1s = (pid0 + 1, pid0 + w, pid0 + w + 1, pid0 + w - 1)
    
    for pid1 in pid1s:
        mag1 = im_mag.ravel()[pid1]
        ang1 = im_ang.ravel()[pid1]
        cost = 0
        if mag1 < MIN_MAG:
            cost = -1
        else:
            ang_diff = abs(mod2pi(ang1 - ang0));
            if ang_diff > MAX_ANG_DIFF:
                cost = -1
            else:
                cost = ang_diff

        if cost >= 0:
            edges.append([pid0, pid1, cost])
    
t = timer() - start
times['4_calc_edges'] = t

disp_edges = np.zeros_like(im_mag, dtype='uint8')
de = disp_edges.ravel()
for e in edges:
    pid0, pid1, cost = e
    de[pid0] = 1
    de[pid1] = 1
    
imshow(disp_edges, title='edges')
    
# %%
total_time = 0.0
for key, value in times.iteritems():
    value *= 1e3
    total_time += value
    print(key, value)
print('total', total_time)