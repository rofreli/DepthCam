import numpy as np
import pyrealsense2 as rs
import cv2
from common import rsResolution, rsConfig;

class rsRecorder:



    def __init__(self, defs):
        self.config = rsConfig.getRSConfig(defs);
        self.pipe = rs.pipeline()
        self.profile = self.pipe.start(self.config)
        self.dev = self.profile.get_device();

        if defs == rsResolution.DEF_640X480FPS60:
            self.frame_width = 640;
            self.frame_height = 480;
        else:
            self.frame_width = 640;
            self.frame_height = 480;


    def record(self, file, aviMovie = False):
        self.pipe.stop(); #Stop the pipeline with the default configuration
        self.config.enable_record_to_file(file+".bag");

        self.newPipe = rs.pipeline();
        self.newPipe.start(self.config); #File will be opened at this point
        self.recording(file, aviMovie);

    def recording(self, file, aviMovie):
        if (aviMovie):
            fourcc = cv2.VideoWriter_fourcc(*'DIVX')
            outRGB = cv2.VideoWriter(file + '.avi', fourcc, 30, (self.frame_width, self.frame_height));
            outDEPTH = cv2.VideoWriter(file + '-D.avi', fourcc, 30, (self.frame_width, self.frame_height));

        while True:

            if aviMovie:
                frames = self.newPipe.wait_for_frames()

                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                colorizer = rs.colorizer()
                colorizer.set_option(rs.option.color_scheme, 2);  # white to black
                colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())

                cv2.imshow('Color', color_image)
                cv2.imshow('Depth', colorized_depth)

                outRGB.write(color_image);
                outDEPTH.write(colorized_depth);

            ch = cv2.waitKey(1);
            if ch == ord('s'):
                self.newPipe.stop();
                self.pipe.start(self.config)
                break;
