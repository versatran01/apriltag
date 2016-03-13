import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
from timeit import default_timer as timer
from collections import OrderedDict
from apriltag_py.utils import DisjointSets, angle_dist, imshow, mod2pi

# %%
fsize2 = (14, 10)
times = OrderedDict()

# %%
cwd = os.getcwd()
image_file = os.path.join(cwd, 'frame0000.png')
color = cv2.imread(image_file, cv2.IMREAD_COLOR)
gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
imshow(color, title='raw')

# %%
# Gaussian blur with downsample
gray_sample = gray

IMAGE_SCALE = 0.5

start = timer()
#gray_blur = cv2.GaussianBlur(gray, (5, 5), 1)
#gray_seg = cv2.resize(gray_blur, None, fx=IMAGE_SCALE, fy=IMAGE_SCALE,
#                      method=cv2.INTER_LINEAR)
gray_seg = cv2.pyrDown(gray)
times['1_pyrdown'] = timer() - start

imshow(gray_seg, title='seg')

# %%
# Calculate image gradient
SCHARR_SCALE = 1 / 16.0

start = timer()
Ix = cv2.Scharr(gray_seg, cv2.CV_32F, 1, 0, scale=SCHARR_SCALE)
Iy = cv2.Scharr(gray_seg, cv2.CV_32F, 0, 1, scale=SCHARR_SCALE)
times['2_scharr'] = timer() - start

ax1, ax2 = imshow(Ix, Iy, figsize=fsize2, title=('Ix', 'Iy'))

# %%
# Calculate gradient magnitude and angle

start = timer()
im_mag, im_ang = cv2.cartToPolar(Ix, Iy)
times['3_polar'] = timer() - start

imshow(im_mag, im_ang, figsize=fsize2, title=('mag', 'ang'))

# %%
mag_max = np.max(im_mag)
mag_mean = np.mean(im_mag)
mag_median = np.median(im_mag)
mag_meme = (mag_mean + mag_median) / 2.0
f, ax = plt.subplots(figsize=(12, 3))
ax.hist(im_mag.ravel(), 255)
ax.axvline(mag_mean, color='m')
ax.axvline(mag_median, color='r')

# %%
num_pixels = np.size(im_mag)
MIN_MAG = mag_mean
mask = im_mag > MIN_MAG

num_mask = np.count_nonzero(mask)
imshow(mask,  title='mean: {}/{}'.format(num_mask, num_pixels))

# %%
MAX_ANG_DIFF = np.deg2rad(5)
COST_SCALE = 1024 / MAX_ANG_DIFF

start = timer()
edges = []
h, w = np.shape(im_mag)
b = 5
for c in xrange(w):
    for r in xrange(h):
        if not (b <= r < h - b and b <= c < w - b):
            continue
        mag0 = im_mag[r, c]
        ang0 = im_ang[r, c]

        if mag0 < MIN_MAG:
            continue

        pid0 = r * w + c
        pid1s = (pid0 + 1, pid0 + w, pid0 + w + 1, pid0 + w - 1)

        for pid1 in pid1s:
            mag1 = im_mag.ravel()[pid1]
            ang1 = im_ang.ravel()[pid1]
            cost = 0
            if mag1 < MIN_MAG:
                cost = -1
            else:
                ang_diff = angle_dist(ang0 - ang1)
                if ang_diff > MAX_ANG_DIFF:
                    cost = -1
                else:
                    cost = int(ang_diff * COST_SCALE)

            if cost >= 0:
                edges.append((pid0, pid1, cost))

t = timer() - start
times['4_calc_edges'] = t

disp_edges = im_mag.copy()
de = disp_edges.ravel()
for e in edges:
    de[e[0]] = mag_max * 2
    de[e[1]] = mag_max * 2

imshow(disp_edges, title='edges')

# %%
# sort edges
start = timer()
edges.sort(key=lambda x: x[-1])
t = timer() - start
times['5_sort_edges'] = t

# %%
# union find
k_ang = 200
k_mag = 360 / SCHARR_SCALE

im_mag_vec = im_mag.ravel()
im_ang_vec = im_ang.ravel()
stats = np.vstack((im_mag_vec, im_mag_vec, im_ang_vec, im_ang_vec)).T
dsets = DisjointSets(num_pixels)

start = timer()
for e in edges:
    pid0, pid1, cost = e
    sid0 = dsets.find(pid0)
    sid1 = dsets.find(pid1)
    if sid0 == sid1:
        continue

    size01 = dsets.set_size(sid0) + dsets.set_size(sid1)

    min_mag0, max_mag0, min_ang0, max_ang0 = stats[sid0]
    min_mag1, max_mag1, min_ang1, max_ang1 = stats[sid1]

    # get delta in magnitude both segments
    d_mag0 = max_mag0 - min_mag0
    d_mag1 = max_mag1 - min_mag1

    # assuming we want to merge these two segments
    # get min and max of merged mag
    min_mag01 = min(min_mag0, min_mag1)
    max_mag01 = max(max_mag0, max_mag1)

    # calculate delta in magnitude for merged segments
    d_mag01 = max_mag01 - min_mag01

    # check with criteria on magnitude
    # M(0 && 1) <= min(D(0), D(1)) + k_mag / size01
    if d_mag01 > min(d_mag0, d_mag1) + k_mag / size01:
        continue

    # get delta in angle
    d_ang0 = angle_dist(max_ang0 - min_ang0)
    d_ang1 = angle_dist(max_ang1 - min_ang1)
    
    m_ang0 = (max_ang0 + min_ang0) / 2.0
    m_ang1 = (max_ang1 + min_ang1) / 2.0
    bshift = mod2pi(m_ang1, m_ang0) + m_ang0 - m_ang1

    # get min and max of merged ang
    min_ang01 = min(min_ang0, min_ang1 + bshift)
    max_ang01 = max(max_ang0, max_ang1 + bshift)
    d_ang01 = angle_dist(min_ang01, max_ang01)

    if d_ang01 > min(d_ang0, d_ang1) + k_ang / size01:
        continue

    # union these two sets
    sid01 = dsets.union(sid0, sid1)
    stats[sid01] = (min_mag01, max_mag01, min_ang01, max_ang01)
#    stats[sid01] = (min_mag01, max_mag01, 0, 0)

t = timer() - start
times['6_union_find'] = t


# %%
# cluster pixels
MIN_SEGMENT_PIXELS = 170 * IMAGE_SCALE ** 2

start = timer()
clusters = dict()
for c in xrange(w):
    for r in xrange(h):
        pid = r * w + c
        sid = dsets.find(pid)
        mag = im_mag[r, c]
        if dsets.set_size(sid) < MIN_SEGMENT_PIXELS:
            continue

        if sid in clusters:
            clusters[sid].append((c, r, mag))
        else:
            clusters[sid] = [(c, r, mag)]

t = timer() - start
times['7_cluster_pixels'] = t

disp_clusters = np.zeros_like(im_mag, dtype='uint8')
disp_clusters = cv2.cvtColor(disp_clusters, cv2.COLOR_GRAY2BGR)
for key, value in clusters.iteritems():
    color = np.random.randint(0, 255, size=3)
    for pixel in value:
        c, r, _ = pixel
        disp_clusters[r, c, :] = color
imshow(disp_clusters, title='clusters')

# %%
total_time = 0.0
for key, value in times.iteritems():
    value *= 1e3
    total_time += value
    print(key, value)
print('total', total_time)

plt.show()
