#!/usr/bin/env python

# TODO: Pull bag selection into command line arg

from collections import Counter

import back_projection as backProp
import roughness
import rosbag


def main(isTrack):
    # get bagfile
    fpath = '/home/todd/autorally/'
    track_bag = "alpha_autorally0_2020-07-23-16-27-57_0.bag"
    sim_bag = "2020-10-15-11-16-39.bag"
    if isTrack:
        fname = fpath + track_bag
    else:
        fname = fpath + sim_bag
    bag = rosbag.Bag(fname)

    # First pass: gather metric labels for time slots
    labels = Counter()
    times, metrics = roughness.get_labels(bag)


if __name__ == '__main__':
    main(False)
