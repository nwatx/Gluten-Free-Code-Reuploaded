package Hardware;

import android.os.SystemClock;

import com.qualcomm.robotcore.hardware.AnalogInput;

import HelperClasses.Robot;

/**
 * This class controls the collector through it's encoder. It is not currently used
 */
public class EncoderController {
    Collector myCollector;//reference to the myCollector
    Robot myRobot;
    Tracker encoder;
    public EncoderController(Collector collector, Robot myRobot, AnalogInput collectorEncoder){
        this.myCollector = collector;
        this.myRobot = myRobot;
        encoder = new Tracker(collectorEncoder);
    }






    private double currentTargetPosition = 0;
    public double getCurrentTargetPosition() {
        return currentTargetPosition;
    }
    public void setCurrentTargetPosition(double currentTargetPosition) {
        this.currentTargetPosition = currentTargetPosition;
    }






    private double currTargetPosition = 0;

    public void initEncoder(){
        double currPos = encoder.getPos();

        /**
         * There are two targets, go to which ever one is closer
         */
        double deltaToTarget1 = (0.5-currPos)%1.0;
        //if we are just past it, go keep going forwards
        if(deltaToTarget1 < 0){
            deltaToTarget1 += 1;
        }
        //we require a while to decelerate so go around again if we are really close
        if(deltaToTarget1 < 0.1){
            deltaToTarget1 += 1;
        }


        double deltaToTarget2 = (0-currPos)%1.0;
        if(deltaToTarget2 < 0){
            deltaToTarget2 += 1;
        }
        if(deltaToTarget2 < 0.1){
            deltaToTarget2 += 1;
        }

        double deltaToTarget = deltaToTarget1<deltaToTarget2 ? deltaToTarget1 : deltaToTarget2;




        //add it back to our curr pos to get target pos
        currTargetPosition = deltaToTarget + currPos;


        //start integral power off zero and remember the time
        integralPower = 0;
        lastStopWhenEncoderUpdateTime = SystemClock.uptimeMillis();
        usingIntegral = false;
    }



    private double integralPower = 0.0;
    private long lastStopWhenEncoderUpdateTime = 0;
    private boolean usingIntegral = false;
    /**
     * This automatically turns off the myCollector when in the right orientation
     * @return
     */
    public boolean stopWhenEncoder() {
        long currTime = SystemClock.uptimeMillis();
        double elapsedSeconds = (double) (currTime - lastStopWhenEncoderUpdateTime)/1000.0;

        myRobot.telemetry.addLine("elapsed seconds: " + elapsedSeconds);


        double currPos = encoder.getPos();
        double error = currTargetPosition-currPos;
        if(Math.abs(error) < 0.3){usingIntegral=true;}
        myRobot.telemetry.addLine("error: " + error);


        integralPower += (error * elapsedSeconds * 0.2);//slowly sum up error and change power
        myRobot.telemetry.addLine("integralPower: " + integralPower);

        double proportionalPower = 0.25 * error;//instantaneous error change

        double deadZoneCorrected = integralPower;
        if(integralPower > 0){
            deadZoneCorrected += 0.05;
        }else{
            deadZoneCorrected -= 0.05;
        }

        //if we aren't using integral yet, don't track and don't use
        if(!usingIntegral){
            integralPower = 0;
            deadZoneCorrected = 0;
        }




        double power = deadZoneCorrected + proportionalPower;

        myCollector.setRollerPower(power);


        //remember the current time for next time
        lastStopWhenEncoderUpdateTime = currTime;
        return false;

    }



    private double collectorSpeedErrorAverage = 0;
    //access myCollector average error through this
    public double getCollectorSpeedErrorAverage(){
        return collectorSpeedErrorAverage;
    }
    private long lastUpdateErrorTime = 0;

    private void UpdateErrorAverage() {
//        long currTime = SystemClock.uptimeMillis();
//        double elapsedSeconds = (SystemClock.uptimeMillis()-lastUpdateErrorTime)/1000.0;
//        //this is the predicted speed of the myCollector given the current power
//
//        double actualSpeed = myCollector.getCurrCollectorSpeed();
//        double error = myCollector.collectorServoTargetSpeed - actualSpeed;
//        double secondsPerSignificantChange = 2;
//        myRobot.telemetry.addLine("Target speed: " + myCollector.collectorServoTargetSpeed);
//        myRobot.telemetry.addLine("actual speed: " + actualSpeed);
//
//        myRobot.telemetry.addLine("AVERAGE ERROR: " + collectorSpeedErrorAverage);
//
//        //update the weighted average
//        collectorSpeedErrorAverage = ((error * elapsedSeconds) + (collectorSpeedErrorAverage * secondsPerSignificantChange))/
//                (elapsedSeconds + secondsPerSignificantChange);
//
//        lastUpdateErrorTime = currTime;
    }






    public void update() {
        //get error
        UpdateErrorAverage();
        encoder.update();//count wraparounds
    }



    /**
     * Gets the myCollector encoder current position
     * @return returns the position of the encoder from 0 - 1 (no wrap around counting)
     */
    public double getEncoderCurrPosition(){
        return -encoder.getPos();
    }



}
