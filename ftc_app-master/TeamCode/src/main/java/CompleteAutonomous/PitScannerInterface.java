package CompleteAutonomous;

import org.firstinspires.ftc.robotcontroller.CompleteAutonomous.PitScanner;

import HelperClasses.Robot;
import RobotUtilities.MyPosition;

/**
 * PitScannerInterface.java
 * @name Peter Szczeszynski
 * This class communicates with PitScanner (in the other module) to get next best collecting
 * location.
 */

public class PitScannerInterface {

    //This is the maximum delta angle to corner we can make while scanning the pit
    //The real tolerance is this *2
    private static final double SCAN_ANGLE_TOLERANCE = Math.toRadians(360);

    //Where the robot needs to be to scan the pit
    private static final double SCAN_TARGET_X = 119;
    private static final double SCAN_TARGET_Y = 119;
    //max distance from target scanning allowed
    private static final double SCAN_DISTANCE_TOLLERANCE = 300;

    //this is the main robot object, we need this to get our coordinates and other things
    private Robot myRobot;



    //Field coordinates of the last scan
    public static double scanXField = 0;
    public static double scanYField = 0;
    //this is flagged when we get our first valid scan
    private static boolean hasScannedYet = false;

    //this is a record of the best
    private static double lowestScanError = 100000000;

    /**
     * Creates new PitScannerInterface
     * @param myRobot -> alias to the main robot object
     */
    public PitScannerInterface(Robot myRobot){
        this.myRobot = myRobot;//remember the robot
    }


    /**
     * Call this after a scan was used and it is time to scan again
     */
    public void startScanning(){
        //make the lowestScanError rediculously high, easy to beat
        lowestScanError = 100000000;
        //flag that we haven't scanned yet
        hasScannedYet = false;
    }



    /**
     * Update will remember a good collecting location when it gets the chance.
     * Call this in the main loop
     */
    public void update(){
        //Calculate the angle to the crater corner
        double angleToCorner = Math.atan2(0-myRobot.getYPos(), 0-myRobot.getXPos());

        //subtract that from the robot angle to get the delta
        double angleFromCrater = MyPosition.subtractAngles(myRobot.getAngle_rad(),angleToCorner);

        //get how far we are away from our target scanning position
        double distanceToScanTarget =  Math.hypot(myRobot.getXPos() - SCAN_TARGET_X,myRobot.getYPos()-SCAN_TARGET_Y);


        myRobot.telemetry.addLine("angleFromCrater: " + Math.toDegrees(angleFromCrater));
        myRobot.telemetry.addLine("distanceToScanTarget: " + distanceToScanTarget);


        //see if we are within the angle tollerance
        if(Math.abs(angleFromCrater) < SCAN_ANGLE_TOLERANCE &&
                distanceToScanTarget < SCAN_DISTANCE_TOLLERANCE){
            //Get this scan's "error" by summing up the distance to the target and angle from the target
            //Prioritize angle
            double scanError = (distanceToScanTarget/SCAN_DISTANCE_TOLLERANCE)*0.3 + Math.abs(angleFromCrater/SCAN_ANGLE_TOLERANCE);
            //if we are better than the lowest scan error, record it
            if(scanError < lowestScanError){
                //remember our error in case it is beaten
                lowestScanError = scanError;

                //update the field coordinates of the scanning location
                scanXField = PitScanner.getBestDropX();
                scanYField = PitScanner.getBestDropY();



                //flag that we have scanned now
                hasScannedYet = true;
            }
        }
    }



}
