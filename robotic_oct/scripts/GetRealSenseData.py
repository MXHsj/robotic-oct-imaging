#! /usr/bin/env python3
import numpy as np
from cv2 import cv2
from pyrealsense2 import pyrealsense2 as rs


class GetRealSenseData():

    def __init__(self):
        self.__pipeline = rs.pipeline()
        self.__config = rs.config()
        self.__config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.__config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # depth filter
        self.__hole_filling = rs.hole_filling_filter()
        self.__spat_filter = rs.spatial_filter()
        self.__spat_filter.set_option(rs.option.filter_smooth_alpha, 1)
        self.__spat_filter.set_option(rs.option.filter_smooth_delta, 50)

        # align frame process
        self.__align_depth2color = rs.align(rs.stream.color)

        # image data
        self.depth_image = None
        self.depth_colormap = None
        self.color_image = None

        # Start streaming
        self.__pipeline.start(self.__config)

    def stream_color_frame(self):
        frames = self.__pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if color_frame:
            # Convert images to numpy arrays
            self.color_image = np.asanyarray(color_frame.get_data())

    def stream_depth_frame(self):
        frames = self.__pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()

        # apply depth filters
        depth_filtered = self.__spat_filter.process(depth_frame)
        depth_filtered = self.__hole_filling.process(depth_filtered)

        if depth_frame:
            # Convert images to numpy arrays
            self.depth_image = np.asanyarray(depth_filtered.get_data())
            self.depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_image, alpha=0.03), cv2.COLORMAP_JET)

    # align color to depth
    def stream_depth2color_aligned(self):
        frames = self.__pipeline.wait_for_frames()
        # align depth to color frame
        aligned_frames = self.__align_depth2color.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        # apply depth filters
        depth_filtered = self.__spat_filter.process(depth_frame)
        depth_filtered = self.__hole_filling.process(depth_filtered)

        if depth_frame and color_frame:
            self.depth_image = np.asanyarray(depth_filtered.get_data())
            self.depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_image, alpha=0.03), cv2.COLORMAP_JET)
            self.color_image = np.asanyarray(color_frame.get_data())

    def disp_frame(frame2disp):
        if not frame2disp:
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', frame2disp)

    def stop_stream(self):
        self.__pipeline.stop()
        cv2.destroyAllWindows()


# test case for GetRealSenseData class
def main():
    get_realsense_data = GetRealSenseData()
    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    while True:
        get_realsense_data.stream_depth2color_aligned()
        # cv2.imshow('RealSense', get_realsense_data.color_image)
        cv2.imshow('RealSense', get_realsense_data.depth_colormap)
        key = cv2.waitKey(33)
        if key & 0xFF == ord('q') or key == 27:
            print('quit')
            get_realsense_data.stop_stream()
            break


if __name__ == "__main__":
    main()
