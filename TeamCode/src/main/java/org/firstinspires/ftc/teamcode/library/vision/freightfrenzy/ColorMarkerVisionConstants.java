package org.firstinspires.ftc.teamcode.library.vision.freightfrenzy;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ColorMarkerVisionConstants {
    // Lower and upper bounds for cv::inRange() function
    public static double CONTOUR_HUE_LOWER_BOUND = 18;
    public static double CONTOUR_HUE_UPPER_BOUND = 40;
    public static double CONTOUR_SAT_LOWER_BOUND = 63;
    public static double CONTOUR_SAT_UPPER_BOUND = 255;
    public static double CONTOUR_LUM_LOWER_BOUND = 40;
    public static double CONTOUR_LUM_UPPER_BOUND = 255;
    public static int CONTOUR_ENTITY_MINWIDTH = 30;
    public static int MAT_OUTPUT_NUM = 2;
    public static double CONTOUR_DILATION_KSIZE = 2.0;

    public static double BOUNDARY_FIRST = 0.30; // was 0.51
    public static double BOUNDARY_SECOND = 0.51; // was 0.7

    public static double COLOR_RED_HUE_LOW = 0;
    public static double COLOR_RED_HUE_HIGH = 10;
    public static double COLOR_RED_SAT_LOW = 50;
    public static double COLOR_RED_SAT_HIGH = 245;
    public static double COLOR_BLUE_HUE_LOW = 110;
    public static double COLOR_BLUE_HUE_HIGH = 130;
    public static double COLOR_BLUE_SAT_LOW = 100;
    public static double COLOR_BLUE_SAT_HIGH = 220;
    public static double COLOR_LUM_LOW = 10;
    public static double COLOR_LUM_HIGH = 255;
    public static int COLOR_WIDTH_MIN = 65;
    public static int COLOR_WIDTH_MAX = 140;
    public static double COLOR_DILATION_KSIZE = 15.0;

    public static double MARKER_CUTOFF_LOWER = 0.3;
    public static double MARKER_CUTOFF_UPPER = 0.7;

    public static boolean SHOW_CONTOUR_MARKERS = true;
    public static boolean SHOW_CONTOUR_TSE = true;

    public static double CUTOFF_TOP = 0.20;
    public static double CUTOFF_BOTTOM = 0.80;
}
