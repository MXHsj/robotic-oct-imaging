#! /usr/bin/env python3
import numpy as np
from pyrealsense2 import pyrealsense2 as rs


class rs_data():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # depth filter
    hole_filling = rs.hole_filling_filter()
    spat_filter = rs.spatial_filter()       # reduce temporal noise
    spat_filter.set_option(rs.option.filter_smooth_alpha, 1)
    spat_filter.set_option(rs.option.filter_smooth_delta, 50)

    def __init__(self):
        # Start streaming
        self.pipeline.start(self.config)
        align = rs.align(rs.stream.color)
