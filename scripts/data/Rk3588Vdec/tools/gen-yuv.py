#!/usr/bin/env python3
# Synthetic I420 test video: gradient background, moving rectangles and fixed-seed noise. Deterministic.
import sys, numpy as np
w, h, n, out = int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]), sys.argv[4]
rng = np.random.RandomState(1234)
noise = rng.randint(-6, 7, size=(h, w)).astype(np.int16)
yy, xx = np.mgrid[0:h, 0:w]
with open(out, 'wb') as f:
    for t in range(n):
        y = (40 + (xx * 150) // w + (yy * 50) // h).astype(np.int16)
        y += noise if t % 3 == 0 else np.roll(noise, t, axis=1)
        cb = np.full((h // 2, w // 2), 128, np.int16); cr = np.full((h // 2, w // 2), 128, np.int16)
        for k, (cy, cu, cv) in enumerate([(200, 90, 160), (60, 200, 90), (150, 60, 60)]):
            x0 = (k * 97 + t * (3 + k * 2)) % max(1, w - w // 4); y0 = (k * 61 + t * (2 + k)) % max(1, h - h // 4)
            y[y0:y0 + h // 4, x0:x0 + w // 4] = cy
            cb[y0 // 2:(y0 + h // 4) // 2, x0 // 2:(x0 + w // 4) // 2] = cu
            cr[y0 // 2:(y0 + h // 4) // 2, x0 // 2:(x0 + w // 4) // 2] = cv
        f.write(np.clip(y, 0, 255).astype(np.uint8).tobytes()); f.write(cb.astype(np.uint8).tobytes()); f.write(cr.astype(np.uint8).tobytes())
