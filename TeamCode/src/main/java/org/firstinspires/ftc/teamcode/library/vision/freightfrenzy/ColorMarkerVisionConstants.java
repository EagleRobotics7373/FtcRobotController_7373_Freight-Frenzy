package org.firstinspires.ftc.teamcode.library.vision.freightfrenzy;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ColorMarkerVisionConstants {
    // Lower and upper bounds for cv::inRange() function
    public static double CONTOUR_HUE_LOWER_BOUND = 10;
    public static double CONTOUR_HUE_UPPER_BOUND = 25;
    public static double CONTOUR_SAT_LOWER_BOUND = 150;
    public static double CONTOUR_SAT_UPPER_BOUND = 255;
    public static double CONTOUR_LUM_LOWER_BOUND = 0;
    public static double CONTOUR_LUM_UPPER_BOUND = 255;
    public static int CONTOUR_ENTITY_MINWIDTH = 60;
    public static int CONTOUR_MAT_PRINTOUT_NUM = 2;
    public static double CONTOUR_DILATION_KSIZE = 2.0;
}
