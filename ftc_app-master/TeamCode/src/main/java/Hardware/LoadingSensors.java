package Hardware;

import android.os.SystemClock;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import HelperClasses.Robot;

/**
 * Polls the two loading sensors and automatically
 */
public class LoadingSensors {
    //time of the last update
    private long lastUpdateTimeMillis = 0;

    private ModernRoboticsI2cRangeSensor leftLoadingSensor;
    private ModernRoboticsI2cRangeSensor rightLoadingSensor;
    private Robot myRobot;


    //light reading of the left sensor
    private double leftSensorCurrentReading = 0;
    //light reading of the right sensor
    private double rightSensorCurrentReading = 0;


    /**
     * Initializes
     * @param left the left loading sensor
     * @param right the right loading sensor
     */
    public LoadingSensors(ModernRoboticsI2cRangeSensor left,
                          ModernRoboticsI2cRangeSensor right,
                          Robot myRobot){
        leftLoadingSensor = left;
        rightLoadingSensor = right;
        //make sure to enable
        enable();
        this.myRobot = myRobot;
    }

    /**An off switch so that we don't call auto feed*/
    private boolean disabled = false;

    /**
     * Call this method to stop the loading sensors from updating
     * (we need this in auto so we can stop when we load 2)
     */
    public void disable(){
        disabled = true;
    }

    /**
     * Re enables the loading sensors
     */
    public void enable(){
        disabled = false;
    }


    /**
     * Polls all the sensors
     */
    public void update(){
        //don't do anything if we are disabled

//
//        myRobot.telemetry.addLine("\nLoading sensors:\n" +
//                "\nL: " + leftLoadingSensor.getDistance(DistanceUnit.CM)
//                + "R: " + rightLoadingSensor.getDistance(DistanceUnit.CM));
        long currTime = SystemClock.uptimeMillis();
        //don't update too fast
        //we will only update every 0 millis
        long minUpdateTimeMillis = 0;
        if(currTime - lastUpdateTimeMillis > minUpdateTimeMillis){
            lastUpdateTimeMillis = currTime;


            leftSensorCurrentReading = leftLoadingSensor.getLightDetected();
            rightSensorCurrentReading = rightLoadingSensor.getLightDetected();
            if(isLoadedLeft() && isLoadedRight() && myRobot.myAutoFeeder.isDoneAutoFeed()){
                //if we are enabled, then call auto feed
                if(!disabled){
                    myRobot.myAutoFeeder.autoFeed(true);
                }
                myRobot.myAutoCollector.abortAutoCollect();
                if(myRobot.myAutoCollector.allCollectingLocations.size() > 0){
                    //go to the next collecting location
                    myRobot.myAutoCollector.allCollectingLocations.remove(0);
                }
            }
        }
    }



    /**
     * Checks if the left loading sensor is loaded
     * @return a boolean, true if there is something there
     */
    public boolean isLoadedLeft(){
        //This is the threshold when the left sensor is considered loaded
        double leftLoadedThresh = 0.05;
        return leftSensorCurrentReading > leftLoadedThresh;
    }

    /**
     * Checks if the right loading sensor is loaded
     */
    public boolean isLoadedRight(){
        //This is the right threshold
        double rightLoadedThresh = 0.05;
        return rightSensorCurrentReading > rightLoadedThresh;
    }


    /**
     * Gets the last update time
     * @return the last update time
     */
    public long getLastUpdateTimeMillis(){
        return lastUpdateTimeMillis;
    }
}
