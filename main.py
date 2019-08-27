import numpy as np
import pyrealsense2 as rs
import cv2
import time
from common import rsResolution, rsConfig, TestTools;
from rsRecorder import rsRecorder as rsRec;


def mouve_event(event, x, y, flags, param):

    global mouse_x, mouse_y;

    if event == cv2.EVENT_LBUTTONDOWN:
        mouse_x = x;
        mouse_y = y;
    if event == cv2.EVENT_LBUTTONDBLCLK:
        mouse_x = None;
        mouse_y = None;


if __name__ == '__main__':
    mouse_x = None;
    mouse_y = None;
    DistanceMatrix = 0;
    d = None;
    Angle = None;

    config = rsConfig.getRSConfig(rsResolution.DEF_640X480FPS30);
    pipe = rs.pipeline()
    profile = pipe.start(config)
    # ************* Inicialização para utilização de nuvem de pontos ****************************
    pc = rs.pointcloud();
    # ************* Inicialização do dispositivo ************************************************
    dev = profile.get_device();
    # ************* Configuração do sensor de profundidade **************************************
    depth_sensor = dev.first_depth_sensor();
    if depth_sensor.supports(rs.option.emitter_enabled):
        depth_sensor.set_option(rs.option.emitter_enabled, 1);
    # depth_sensor.set_option(rs.option.depth_units, float(0.001));
    # rangeM = depth_sensor.get_option_range(rs.option.laser_power);
    depth_sensor.set_option(rs.option.laser_power, 90.0);  # Set max
    # depth_sensor.set_option(RS2_OPTION_LASER_POWER, 0.f); #Disable laser
    depth_sensor.set_option(rs.option.enable_auto_exposure, 1);
    # depth_sensor.set_option(rs.option.holes_fill, )
    # *******************************************************************************************
    cv2.namedWindow('Color', cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow('Depth', cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback("Depth", mouve_event)
    try:
        while True:
            start_time = time.time()
            frames = pipe.wait_for_frames()

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            accel = frames[2].as_motion_frame().get_motion_data();
            gyro = frames[3].as_motion_frame().get_motion_data();
            #[anglex, anglez] = process_gyro(accel);
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            # images = np.hstack((color_image, colorized_depth))

            cv2.imshow('Color', color_image)

            colorizer = rs.colorizer()
            colorizer.set_option(rs.option.color_scheme, 0);  # white to black
            colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())
            if(mouse_x is not None and mouse_y is not None):
                if(DistanceMatrix==0):
                    d = depth_frame.get_distance(mouse_x, mouse_y)
                else:
                    d = TestTools.get_distance(depth_frame,mouse_x, mouse_y, DistanceMatrix, d);
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(colorized_depth, 'Distancia: %.2f' % round(d,2), (10, 400), font, 1, (255, 255, 255), 3, cv2.LINE_AA)

            if Angle is not None:
                Angle = TestTools.process_angle(accel)
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(colorized_depth, 'Ang. X,Z Acelerometro: %.2f' % round(Angle[0], 2) + ' %.2f' % round(Angle[1], 2), (10, 30), font, 1, (255, 255, 255), 3,
                        cv2.LINE_AA)

            cv2.imshow('Depth', colorized_depth)

            #print("FPS: ", 1.0 / (time.time() - start_time))  # FPS = 1 / time to process loop
            ch = cv2.waitKey(1)
            if ch == 27:
                break
            elif ch == ord('c'):
                r = cv2.selectROI("Color", color_image)
                print("ROI", r);
            elif ch == ord('d'):
                r = cv2.selectROI("Depth", colorized_depth)
                print("ROI", r);
            elif ch == ord('p'):
                cv2.waitKey();
            elif ch == ord('i'):
                if Angle is None:
                    Angle = TestTools.process_angle(accel)
                else:
                    Angle = None;
            elif ch == ord('r'):
                rs = rsRec(rsResolution.DEF_640X480FPS60);
                rs.record("C:\\Users\\rodri\\Documents\\Doutorado\\Tese\\Arquivo",False);
                break
            elif ch >= 40 and ch <= 57:
                DistanceMatrix = int(chr(ch));

    finally:
        pipe.stop()
