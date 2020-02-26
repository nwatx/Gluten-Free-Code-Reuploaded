package org.firstinspires.ftc.robotcontroller.CompleteAutonomous;

/**
 * PitScanner.java
 * This is the interface to the C++ class that scans the pit and gives a collecting location
 */
public class PitScanner {

    //These are the world coordinates of the best location to drop the collector down
    //They are set by the C++ method
    private static double bestDropX = 0;
    private static double bestDropY = 0;


    //These are the world coordinates of the best cube location
    private static double bestCubeX = 0;
    private static double bestCubeY = 0;



    /**
     * This is a native method that scans the pit for minerals.
     * @param addrRgba the address of the image you wish to scan
     * @param craterAngle the angle of the crater on the screen, need to compute
     * @param robotX the x position of the robot on the field
     * @param robotY the y position of the robot on the field
     * @param robotAngle the angle of the robot
     * @param xPositions array with sufficient length that will be modified with all the mineral
     *                   x locations on the field
     * @param yPositions array with sufficient length that will be modified with all the mineral
     *                   y locations on the field
     */
    public native static void scanPit(long addrRgba, double craterAngle,
                                      double robotX, double robotY, double robotAngle,
                                      double[] xPositions, double[] yPositions,boolean secondScan);


    /**
     * Get's the best drop zone x for the collector
     * @return the x coordinate to drop the collector at
     */
    public static double getBestDropX(){
        return bestDropX;
    }
    /**
     * Get's the best drop zone y for the collector
     * @return the y coordinate to drop the collector at
     */
    public static double getBestDropY(){
        return bestDropY;
    }


    /**
     * Get's the best cube y position world coordinates
     * @return
     */
    public static double getBestCubeY() { return bestCubeY; }

    /**
     * Get's the best cube x position world coordinates
     * @return
     */
    public static double getBestCubeX() { return bestCubeX; }
}
