import pyrealsense2 as rs
import cv2
from enum import Enum
import numpy as np;


class rsResolution(Enum):
    DEF_640X480FPS90 = 0
    DEF_640X480FPS6 = 1
    DEF_640X480FPS15 = 2
    DEF_640X480FPS25 = 3
    DEF_640X480FPS30 = 4
    DEF_640X480FPS60 = 5
    DEF_480X270FPS6 = 6
    DEF_480X270FPS15 = 7
    DEF_480X270FPS25 = 8
    DEF_480X270FPS30 = 9
    DEF_480X270FPS60 = 10
    DEF_480X270FPS90 = 11

class rsConfig:

    def getRSConfig(defs):
        config = rs.config()

        # ************* Configuração dos sensores, suas resoluções e quantidade de frames por segundo
        #if defs == rsResolution.DEF_640X480:
        # implementar outras definições
        # *******************************************************
        if defs == rsResolution.DEF_640X480FPS90:
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 90)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)
        if defs == rsResolution.DEF_640X480FPS60:
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 60)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)
        if defs == rsResolution.DEF_640X480FPS30:
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        else:
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 90)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        # config.enable_stream(rs.stream.infrared, 640, 480, rs.format.y8, 30)
        config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200);
        config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 250);
        return config;

class TestTools:
    def get_distance(depth_frame, x,y, distanceMatrix, lastDistance):
        d = 0;
        count = 0;

        ri = range(x-distanceMatrix,x+distanceMatrix+1)
        for i in ri:
            if i<0 or i> depth_frame.width:
                continue;
            for j in range(y - distanceMatrix, y + distanceMatrix+1):
                if j < 0 or j > depth_frame.height:
                    continue;

                count = count + 1;
                d = d + depth_frame.get_distance(i,j);
        if lastDistance is not None:
            return (lastDistance + (d / count)) / 2;
        else:
            return d / count;

    def process_angle(data):
        gyro_anglez = np.rad2deg(np.arctan2(data.y, data.z)) + 90;
        gyro_anglex = np.rad2deg(
            np.arctan2(data.x, np.sqrt(data.y * data.y + data.z * data.z))) + 90;
        return [gyro_anglex, gyro_anglez];

    def writeFileTest(path, verts):
        f = open(path, "w+")

        for i in range(len(verts)):
            x = verts[i][0];
            y = verts[i][1];
            z = verts[i][2];
            f.write("%f\t" % x)
            f.write("%f\t" % y)
            f.write("%f\n" % z)

        f.close()