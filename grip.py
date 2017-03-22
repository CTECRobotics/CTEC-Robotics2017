import cv2
import numpy
import math
from enum import Enum
from networktables import NetworkTables

NetworkTables.initialize(server='roborio-6445-frc.local')
sd = NetworkTables.getTable('SmartDashboard')

class Vision:
    """
    An OpenCV pipeline generated by GRIP.
    """

    def __init__(self):
        """initializes all values to presets or None if need to be set
        """
        self.__hsv_threshold_hue = [38.09089118939224, 86.13255726266553]
        self.__hsv_threshold_saturation = [82.55395683453237, 255.0]
        self.__hsv_threshold_value = [197.21223021582733, 255.0]

        self.hsv_threshold_output = None

        self.__find_contours_input = self.hsv_threshold_output
        self.__find_contours_external_only = False

        self.find_contours_output = None

        self.__convex_hulls_contours = self.find_contours_output

        self.convex_hulls_output = None


    def process(self, source0):
        """
        Runs the pipeline and sets all outputs to new values.
        """
        # Step HSL_Threshold0:
        self.__hsv_threshold_input = source0
        (self.hsv_threshold_output) = self.__hsv_threshold(self.__hsv_threshold_input, self.__hsv_threshold_hue, self.__hsv_threshold_saturation, self.__hsv_threshold_value)

        # Step Find_Contours0:
        self.__find_contours_input = self.hsv_threshold_output
        (self.find_contours_output) = self.__find_contours(self.__find_contours_input, self.__find_contours_external_only)

        """# Step Convex_Hulls0:
        self.__convex_hulls_contours = self.find_contours_output
        (self.convex_hulls_output) = self.__convex_hulls(self.__convex_hulls_contours) """
        #step get data0:
        self.line_up(contours)
    @staticmethod
    def __hsv_threshold(input, hue, sat, val):
        """Segment an image based on hue, saturation, and value ranges.
        Args:
            input: A BGR numpy.ndarray.
            hue: A list of two numbers the are the min and max hue.
            sat: A list of two numbers the are the min and max saturation.
            lum: A list of two numbers the are the min and max value.
        Returns:
            A black and white numpy.ndarray.
        """
        out = cv2.cvtColor(input, cv2.COLOR_BGR2HSV)
        return cv2.inRange(out, (hue[0], sat[0], val[0]), (hue[1], sat[1], val[1]))

    @staticmethod
    def __find_contours(input, external_only):
        """Sets the values of pixels in a binary image to their distance to the nearest black pixel.
        Args:
            input: A numpy.ndarray.
            external_only: A boolean. If true only external contours are found.
        Return:
            A list of numpy.ndarray where each one represents a contour.
        """
        if(external_only):
            mode = cv2.RETR_EXTERNAL
        else:
            mode = cv2.RETR_LIST
        method = cv2.CHAIN_APPROX_SIMPLE
        im2, contours, hierarchy =cv2.findContours(input, mode=mode, method=method)
        return contours

    @staticmethod
    def __convex_hulls(input_contours):
        """Computes the convex hulls of contours.
        Args:
            input_contours: A list of numpy.ndarray that each represent a contour.
        Returns:
            A list of numpy.ndarray that each represent a contour.
        """
        output = []
        for contour in input_contours:
            output.append(cv2.convexHull(contour))
        return output

    @staticmethod
    def line_up(contour):
        distance = 0 # get distance from distance sensor will crash at 0
        try:
            cnt_0 = contour[0]
            cnt_1 = contour[1]
            M = cv2.moments(cnt_0)
            x,y,w,h = cv2.boundingRect(cnt_0)
            x_1,y_1,w_1,h_1 = cv2.boundingRect(cnt_1)
            angle_const = 68.5/2
            angle_0 = math.degrees(math.atan(w/distance))
            angle_1 = math.degrees(math.atan(w_1/distance))
            if math.floor(angle_0) == math.floor(angle_1):
                sd.putValue('lined_up',true)
            else:
                sd.putValue('lined_up',false)
                sd.putValue('angle_1',angle_0)
                sd.putValue('angle_2',angle_1)
        except IndexError as e:
            sd.putValue("IndexError", IndexError)
            
if __name__ == "__main__":
       ip = Vision()
       while(True):
           ip.process(cv2.VideoCapture(0))
