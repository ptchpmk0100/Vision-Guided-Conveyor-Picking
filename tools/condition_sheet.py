#!/usr/bin/env python3
"""Contact sheet of random crops per class (label sanity check by eye).  python3 tools/condition_sheet.py <dataset> <out.png> [n] [column]"""
import csv
import random
import sys
from pathlib import Path

import cv2
import numpy as np

root, out = Path(sys.argv[1]), sys.argv[2]
n = int(sys.argv[3]) if len(sys.argv) > 3 else 20
column = sys.argv[4] if len(sys.argv) > 4 else 'gt_crop'
rows = [r for r in csv.DictReader(open(root / 'index.csv')) if r[column]]
rng = random.Random(1)
sheets = []
for cls in ('intact', 'damaged'):
    pick = rng.sample([r for r in rows if r['label'] == cls], min(n, sum(r['label'] == cls for r in rows)))
    tiles = [cv2.resize(cv2.imread(str(root / r[column])), (128, 128), interpolation=cv2.INTER_CUBIC) for r in pick]
    while len(tiles) % 10:
        tiles.append(np.zeros((128, 128, 3), np.uint8))
    grid = np.vstack([np.hstack(tiles[i:i + 10]) for i in range(0, len(tiles), 10)])
    cv2.putText(grid, cls.upper(), (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    sheets.append(grid)
cv2.imwrite(out, np.vstack(sheets))
print('wrote', out)
