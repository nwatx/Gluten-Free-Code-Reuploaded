package FieldStats;

public class Field {
    //length of the field in centimeters
    public static final double FIELD_LENGTH = 358.775;


    //crater1 line approximation points
    public static final double CRATER1_TOP_LEFT_X = 0;
    public static double CRATER1_TOP_LEFT_Y = 160;
    public static double CRATER1_BOTTOM_RIGHT_X = 130;
    public static double CRATER1_BOTTOM_RIGHT_Y = 0;

    //now we can calculate the other crater coordinates
    public static final double CRATER2_TOP_LEFT_X = FIELD_LENGTH - CRATER1_TOP_LEFT_X;
    public static final double CRATER2_TOP_LEFT_Y = FIELD_LENGTH - CRATER1_TOP_LEFT_Y;
    public static final double CRATER2_BOTTOM_RIGHT_X = FIELD_LENGTH - CRATER1_BOTTOM_RIGHT_X;
    public static final double CRATER2_BOTTOM_RIGHT_Y = FIELD_LENGTH - CRATER1_BOTTOM_RIGHT_Y;


    /**
     * Get's the slope of crater1
     * @return the slope
     */
    public static double getCrater1Slope(){
        return (CRATER1_TOP_LEFT_Y - CRATER1_BOTTOM_RIGHT_Y)/
                (CRATER1_TOP_LEFT_X - CRATER1_BOTTOM_RIGHT_X);
    }

    /**
     * Get's the slope of crater2
     * @return the slope
     */
    public static double getCrater2Slope(){
        return (CRATER2_TOP_LEFT_Y - CRATER2_BOTTOM_RIGHT_Y)/
                (CRATER2_TOP_LEFT_X - CRATER2_BOTTOM_RIGHT_X);
    }
}
