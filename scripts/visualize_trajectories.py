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
from PIL import Image


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Visualize the positions of the fish accompanied by the feature information')
    parser.add_argument('--path', '-p', type=str,
                        help='Folder path',
                        required=True)
    parser.add_argument('--excluded-idx', '-e', type=int,
                        help='Number of experiment',
                        default=-1,
                        required=False)
    parser.add_argument('--out-dir', '-o', type=str,
                        help='Output directory name',
                        required=True)
    parser.add_argument('--fish-like', action='store_true',
                        help='Images instead of points',
                        default=False)
    parser.add_argument('--turing', action='store_true',
                        help='Same image for all individuals to perform a turing test',
                        default=False)
    parser.add_argument('--dark', action='store_true',
                        help='Render dark friendly icons',
                        default=False)
    args = parser.parse_args()

    iradius = 0.198
    oradius = 0.298
    center = (0.502824858757, 0.500324858757)

    if args.dark:
        image_path = os.getcwd() + '/exp/burst-and-coast/scripts/fish_dark.png'
    else:
        image_path = os.getcwd() + '/exp/burst-and-coast/scripts/fish.png'
    image = Image.open(image_path)
    image_path = os.getcwd() + '/exp/burst-and-coast/scripts/excluded.png'
    excluded_image = Image.open(image_path)
    image_path = os.getcwd() + '/exp/burst-and-coast/scripts/excluded_t_1.png'
    excluded_image_t_1 = Image.open(image_path)
    image_path = os.getcwd() + '/exp/burst-and-coast/scripts/robot.png'
    rimage = Image.open(image_path)

    traj = np.loadtxt(args.path + '/positions.dat')[:, 2:]
    vel = np.loadtxt(args.path + '/velocities.dat')[:, 2:]

    tsteps = traj.shape[0]

    rtraj = np.roll(traj, 1, axis=0)
    rvel = np.roll(vel, 1, axis=0)
    if not os.path.exists(args.out_dir):
        os.makedirs(args.out_dir)

    for i in range(tsteps):
        fig = plt.figure(figsize=(5, 5))
        ax = plt.gca()

        # radius = (iradius, oradius)
        # # plt.plot([center[0]], [center[1]], ls='none',
        # #  marker='o', color='black', label='Origin ' + str(center))
        # if args.dark:
        #     color = 'white'
        # else:
        #     color = 'black'
        # inner = plt.Circle(
        #     center, radius[0], color=color, fill=False)
        # outer = plt.Circle(
        #     center, radius[1], color=color, fill=False)
        # ax.add_artist(inner)
        # ax.add_artist(outer)

        for inum, j in enumerate(range(int(traj.shape[1] / 2))):
            x = traj[i, j*2]
            y = traj[i, j*2+1]

            if not args.fish_like:
                plt.scatter(x, y, marker='.',
                            label='Individual ' + str(inum) + ' ' + "{:.2f}".format(x) + ' ' + "{:.2f}".format(y))
                Q = plt.quiver(
                    x, y, vel[i,  j*2], vel[i,  j*2+1], scale=1, units='xy')
            else:
                phi = np.arctan2(vel[i,  j*2+1], vel[i,  j*2]) * 180 / np.pi

                if args.excluded_idx >= 0 and args.excluded_idx == j and not args.turing:
                    rotated_img = excluded_image.rotate(phi)
                else:
                    rotated_img = image.rotate(phi)
                ax.imshow(rotated_img, extent=[x-0.0175, x+0.0175, y -
                                               0.0175, y+0.0175], aspect='equal')

        ax.axis('off')

        ax.set_xlim((-0.27, 0.27))
        ax.set_ylim((-0.27, 0.27))
        plt.tight_layout()

        png_fname = args.out_dir + '/' + str(i).zfill(6)
        plt.savefig(
            str(png_fname) + '.png',
            transparent=True,
            dpi=300
        )
        plt.close('all')
