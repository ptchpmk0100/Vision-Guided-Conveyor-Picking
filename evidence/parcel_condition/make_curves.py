#!/usr/bin/env python3
"""Draw training curves from training/r1/history.json -> figures/training_curves.png (needs matplotlib)."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

here = Path(__file__).parent
h = json.loads((here / 'training/r1/history.json').read_text())
ep = [e['epoch'] for e in h]
fig, ax = plt.subplots(1, 2, figsize=(11, 3.8))
ax[0].plot(ep, [e['train_loss'] for e in h], label='train loss')
ax[0].plot(ep, [e['val_loss'] for e in h], label='validation loss')
ax[0].set(xlabel='epoch', title='Loss')
ax[1].plot(ep, [e['val_acc'] for e in h], label='accuracy')
ax[1].plot(ep, [e['val_damaged_recall'] for e in h], label='damaged recall')
ax[1].plot(ep, [e['val_intact_recall'] for e in h], label='intact recall')
ax[1].set(xlabel='epoch', ylim=(0, 1.02), title='Validation')
for a in ax:
    a.axvspan(1, 3, alpha=.12, label='head-only epochs')
    a.grid(alpha=.3)
    a.legend()
fig.tight_layout()
(here / 'figures').mkdir(exist_ok=True)
fig.savefig(here / 'figures/training_curves.png', dpi=130)
print('wrote figures/training_curves.png')
