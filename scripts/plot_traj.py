#!/usr/bin/env python

import matplotlib
matplotlib.use('Agg')

import os
import sys
import time
import socket
import warnings
import argparse
import datetime
import numpy as np
from pathlib import Path
import matplotlib as mpl
import matplotlib.pyplot as plt
import scipy.misc
from matplotlib.image import BboxImage
from matplotlib.transforms import Bbox, TransformedBbox
from scipy import ndimage
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
from matplotlib.cbook import get_sample_data
from mpl_toolkits.axes_grid1 import make_axes_locatable

plt.style.use('dark_background')

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Visualize the positions')
    parser.add_argument('--positions', '-p', type=str,
                        help='Path to the trajectory file',
                        required=True)
    parser.add_argument('--fname', '-o', type=str,
                        help='output file name',
                        required=True)
    parser.add_argument('--ind', '-i', type=int,
                        default=-1,
                        help='single individual id to plot',
                        required=True)
    parser.add_argument('--timesteps', '-t', type=int,
                        default=-1,
                        help='Timesteps to use in the plot',
                        required=False)
    parser.add_argument('--no-probs',  action='store_true',
                        help='No probabilities are contained within the trajectory file', default=True)
    parser.add_argument('--points',  action='store_true',
                        help='Plot points instead of trajectory lines', default=False)
    parser.add_argument('--open',  action='store_true',
                        help='No probabilities are contained within the trajectory file', default=False)
    args = parser.parse_args()

    traj = np.loadtxt(args.positions)[:, 2:]
    tsteps = traj.shape[0]

    if args.timesteps < 0:
        args.timesteps = tsteps

    if (args.no_probs):
        coef = 2
    else:
        coef = 3
    individuals = int(traj.shape[1] / coef)

    iradius = 0.192
    oradius = 0.292
    center = (0.501, 0.508)
    radius = (iradius, oradius)

    fig = plt.figure(figsize=(5, 6))
    ax = plt.gca()

    inner = plt.Circle(
        center, radius[0], color='white', fill=False)
    outer = plt.Circle(
        center, radius[1], color='white', fill=False)
    # if not args.open:
    #     ax.add_artist(inner)
    # ax.add_artist(outer)

    if args.ind < 0:
        for j in range(int(traj.shape[1] / 2)):
            if not args.points:
                plt.plot(traj[:args.timesteps, j*coef],
                         traj[:args.timesteps, j*coef + 1], linewidth=0.2)
            else:
                warnings.warn(
                    'Not supported for all individuals. Please specify an index')
    else:
        plt.plot(traj[:args.timesteps, args.ind*coef],
                 traj[:args.timesteps, args.ind*coef + 1], linewidth=0., marker='.', markersize=0.5)
    ax.set_xlim([-0.27, 0.27])
    ax.set_ylim([-0.27, 0.27])

    plt.tight_layout()
    plt.savefig(args.fname + '.png', dpi=300)
