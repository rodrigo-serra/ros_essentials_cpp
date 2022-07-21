#!/usr/bin/env python

import numpy as np
import cv2

from imgProcessing import detectLine

def main():
    video_capture = cv2.VideoCapture("/home/rodrigo/catkin_ws/src/ros_essentials_cpp/src/topic03_perception/video/test2.mp4")
    # video_capture = cv2.VideoCapture("/home/rodrigo/Documents/durable/tests/ist/2022_07_06/hex/bags/2022-07-05-19-38-14_fiducial_images_compressed.mp4")
    image_name = '/home/rodrigo/catkin_ws/src/ros_essentials_cpp/src/topic03_perception/images/Frame1.jpg'
    imgNumber = 0
    readVideo = True
    saveImg = False

    if readVideo:
        while(True):
            ret, frame = video_capture.read()
            endImg = detectLine(frame)
            cv2.imshow("Frame", endImg)

            if (cv2.waitKey(10) & 0xFF == ord('q')) or ret == False:
                break

            # Save Frame by Frame into disk using imwrite method
            if imgNumber < 2 and saveImg:
                cv2.imwrite('Frame'+str(imgNumber)+'.jpg', frame)

            imgNumber += 1


        video_capture.release()

    else:
        frame = cv2.imread(image_name)

        endImg = detectLine(frame)

        cv2.imshow("Frame", endImg)

        cv2.waitKey(0)

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
