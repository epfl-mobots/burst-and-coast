#!/usr/bin/env python
# encoding: utf-8


def build(bld):
    srcs = []
    nodes = bld.path.ant_glob('*.cpp')
    for n in nodes:
        srcs += [n]

    bld.program(features='cxx',
                source=['../../src/stat/stat_base.cpp',
                        '../../src/simulation/simulation.cpp'] + srcs,
                includes='. ../../src/',
                uselib='BOOST BOOST_SYSTEM BOOST_FILESYSTEM BOOST_REGEX EIGEN',
                target='rummy_sim')
