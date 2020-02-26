package Hardware;

import android.os.SystemClock;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import HelperClasses.Robot;
import ReturnTypes.FloatPoint;
import RobotUtilities.MovementEssentials;

/**
 * This is the base class for the collector and the lift extensions
 */
public class Extension {

    public RevMotor extensionMotor;//main extension motor
    public RevMotor encoderMotor;//set this if you want a different motor to do the encoder readings
    public Robot myRobot;//link to the robot
    protected double EXTENSION_MIN;//min encoder value
    protected double EXTENSION_MAX;//max encoder value
    protected double PERCENT_DECELERATION;//percent of range used to decelerate
    public double masterOffsetTicks = 0;//a value that is added to the return val when getPosition or getPercent is called

    //this is the time of prediction incorporated in setExtensionSpeedNice
    public double EXTENSION_PREDICTION_TIME_SECONDS = 0.1;//default to 0.1 seconds


    //the length of the extension in cm when extended
    public double LENGTH_EXTENDED;
    //the myCollector is in the front of the robot
    public double OFFSET_FROM_CENTER;




    /**
     * Initializes the extension with a separate encoder motor
     * @param myRobot
     * @param extensionMotor
     * @param encoderMotor
     */
    public Extension(Robot myRobot, RevMotor extensionMotor, RevMotor encoderMotor,
                     double extendedLength, double offsetFromCenter){
        this.myRobot = myRobot;
        this.extensionMotor = extensionMotor;
        this.encoderMotor = encoderMotor;
        this.LENGTH_EXTENDED = extendedLength;
        this.OFFSET_FROM_CENTER = offsetFromCenter;

        //since the encoder motor is different from the extension motor we can't be
        //on speed or position control
        extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    /**
     * Initializes the extension with the extension motor also having the encoder
     * @param myRobot this is a reference to the main robot class
     * @param extensionMotor the DcMotor that controls the extension
     */
    public Extension(Robot myRobot, RevMotor extensionMotor,
                     double extendedLength, double offsetFromCenter){
        this.myRobot = myRobot;
        this.extensionMotor = extensionMotor;
        this.encoderMotor = extensionMotor;//the encoder motor will be the same as the extension motor
        this.LENGTH_EXTENDED = extendedLength;
        this.OFFSET_FROM_CENTER = offsetFromCenter;
    }


    /**
     * Sets how long in seconds the extension will predict into the future.
     * @param predictionTime prediction time in seconds
     */
    public void setEXTENSION_PREDICTION_TIME_SECONDS(double predictionTime){
        this.EXTENSION_PREDICTION_TIME_SECONDS = predictionTime;
    }


    /**
     * Sets the range of the extension, min and max
     * @param EXTENSION_MIN the minimum encoder value the extension can go to
     * @param EXTENSION_MAX the maximum encoder value the extension can go to
     */
    public void setRange(double EXTENSION_MIN, double EXTENSION_MAX){
        this.EXTENSION_MIN = EXTENSION_MIN;
        this.EXTENSION_MAX = EXTENSION_MAX;
    }


    /**
     * Sets how much deceleration will be used when using speed or power mode only
     */
    public void setPercentDeceleration(double PERCENT_DECELERATION){
        this.PERCENT_DECELERATION = PERCENT_DECELERATION;
    }


    /**
     * Use this to get the raw extension motor power currently
     * @return
     */
    public double getExtensionMotorPower(){
        return extensionMotorPower;
    }

    //power that is to be applied to the extension motor in update()
    protected double extensionMotorPower = 0.0;
    //where the extension motor will try to get to
    protected double extensionMotorTargetPosition = 0;

    //position of the extension motor encoder
    private double extensionMotorCurrentPosition = 0;

    //this is the last power of the extension motor so that we only set it when it's value changes
    private double extensionMotorPowerLast = extensionMotorPower;
    private int extensionMotorTargetPositionLast = (int) extensionMotorTargetPosition;


    /**
     * Applies the motor power once so we don't have issues
     */
    public void update(){
        myRobot.telemetry.addLine("extension percent: " + getExtensionPercent());
        //set the extensionMotor's mode to either run to position or run using encoder or power control
        if(runToPositionMode == RunToPositionModes.positionControl){
            //ONLY SET THE TARGET POSITION IF IT HAS CHANGED
            if((int) extensionMotorTargetPosition != extensionMotorTargetPositionLast){
                extensionMotor.setTargetPosition((int) extensionMotorTargetPosition);
                extensionMotorTargetPositionLast = (int) extensionMotorTargetPosition;
            }
        }


        //update the mode if the extensionMotor is the same as the encoderMotor
        if(lastRunToPositionMode != runToPositionMode && extensionMotor == encoderMotor){
            if(runToPositionMode == RunToPositionModes.positionControl){
                extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else if(runToPositionMode == RunToPositionModes.speedControl){
                extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            else if(runToPositionMode == RunToPositionModes.powerControl){
                extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            lastRunToPositionMode = runToPositionMode;
        }






        //set the extension motor power if it has changed
        extensionMotor.setPower(extensionMotorPower);







        long currTime = SystemClock.uptimeMillis();
        if(currTime - lastReadEncoderTime > READ_ENCODER_MIN_TIME){
            //read the encoder current position
            extensionMotorCurrentPosition = encoderMotor.getCurrentPosition();
            //calculate the extension speed
            calculateExtensionCurrentSpeed();
            //remember the time of this update
            lastReadEncoderTime = currTime;
        }
    }




    private long lastReadEncoderTime = 0;
    //number of times per second we read the extension encoder
    private final int READ_ENCODER_MIN_TIME = 0;

    ///////calculateExtensionCurrentSpeedVariables///////
    private long lastUpdateSpeedTime = 0;
    private double lastUpdateExtensionPosition = 0;
    private double extensionCurrentSpeed = 0;//speed in encoder ticks per second
    /**
     * This method runs every update and will monitor how fast the extension is moving in
     * ticks per second
     */
    private void calculateExtensionCurrentSpeed() {
        long currTime = SystemClock.uptimeMillis();
        long elapsedTime = currTime - lastUpdateSpeedTime;
        if(elapsedTime < 50){return;}//don't update if elapsedTime is less than 100 millis
        if(elapsedTime > 1000){elapsedTime = 0;}//assume no time has passed if this is the first update

        double elapsedSeconds = elapsedTime/1000.0;//get the current time in seconds

        double currPos = getExtensionCurrPos();
        double changeInPosition = currPos-lastUpdateExtensionPosition;//get how far we've moved

        //now we can calculate how fast we're going
        extensionCurrentSpeed = changeInPosition/elapsedSeconds;


        lastUpdateExtensionPosition = currPos;
        lastUpdateSpeedTime = currTime;//remember the time for next update
    }

    /**
     * Returns our current speed
     * @return how fast we're currently going
     */
    public double getExtensionCurrentSpeedTicks(){
        return extensionCurrentSpeed;
    }


    /**
     * Returns the percent per second we are going
     */
    public double getExtensionCurrentSpeedPercent(){
        return extensionCurrentSpeed/(EXTENSION_MAX-EXTENSION_MIN);
    }








    /**
     * Sets the power raw with no deceleration, be careful
     */
    public void setExtensionPowerRaw(double power) {
        extensionMotorPower = power;
    }

    /**
     * sets the power (speed) with deceleration/limits on the end
     */
    public void setExtensionPowerNice(double power,double retractedMinPower,double extendedMinPower){


        double powerToApply = power;

        //get the current percent (0-1) the extension has extended
        //INCLUDE 0.2 SECONDS OF PREDICTION
        double predictionTimeSeconds = EXTENSION_PREDICTION_TIME_SECONDS;
        double extensionPredictedPercent = getExtensionPercent();// + (predictionTimeSeconds * getExtensionCurrentSpeedPercent());


        //the distance to the start of its range is just it's percent, but clipped to never be less than 0
        double distToStart = Range.clip(extensionPredictedPercent,0.001,1);
        //the distance to the end of it's range is just 1 minus it's percent and clipped again
        double distToEnd = Range.clip(1.0-extensionPredictedPercent,0.001,1);


        double maximumPower;//this is the fastest we are allowed to go at the moment
        if(power > 0){
            maximumPower = Range.clip(Math.abs(distToEnd/PERCENT_DECELERATION),0.001,0.9999);
        }else{
            maximumPower = Range.clip(Math.abs(distToStart/PERCENT_DECELERATION),0.001,0.9999);
        }


        boolean inFastMode = getExtensionPercent() > 0.2 && getExtensionPercent() < 0.8;

        if(inFastMode){
            //use speed control, not go to position
            setRunToPositionMode(RunToPositionModes.powerControl);
            maximumPower *= 1.0;
        }else{
            //use speed control, not go to position
            setRunToPositionMode(RunToPositionModes.speedControl);
            maximumPower *= 0.6;
        }


//        maximumPower *= 0.2;
//        myRobot.telemetry.addLine("\n\nMaximum Power: " + maximumPower);
//        myRobot.telemetry.addLine("extensionCurrPercent: " + getExtensionPercent());
//        myRobot.telemetry.addLine("percentPrediction: " + extensionPredictedPercent);
//        myRobot.telemetry.addLine("power: " + power);
//        myRobot.telemetry.addLine("distToStart: " + distToStart);
//        myRobot.telemetry.addLine("distToEnd: " + distToEnd);
//        myRobot.telemetry.addLine("maximumPower: " + maximumPower + "\n\n");

        double minPower = distToStart < distToEnd ? retractedMinPower : extendedMinPower;
        maximumPower = MovementEssentials.minPower(maximumPower,minPower);
        //clip the power to never go above the maximum power
        if(Math.abs(powerToApply) > maximumPower){
            powerToApply = powerToApply > 0 ? maximumPower : -maximumPower;
        }

//
//        if(Math.abs(power) > 0.05){
//            if(distToStart < 0.3 && power < 0){
    //                powerToApply = MovementEssentials.minPower(powerToApply,retractedMinPower);
    ////                setRunToPositionMode(RunToPositionModes.powerControl);
    //            }
    //            if(distToEnd < 0.3 && power > 0){
    //                powerToApply = MovementEssentials.minPower(powerToApply,extendedMinPower);
    ////                setRunToPositionMode(RunToPositionModes.powerControl);
//            }
//            //if we are applying like 100% power, always min power this
////            if(Math.abs(power) > 0.99){
////                powerToApply = MovementEssentials.minPower(powerToApply,
////                        distToStart < distToEnd ? retractedMinPower : extendedMinPower);
////            }
//        }





        //now counter if we are going to go outside our range
//        if(extensionPredictedPercent < 0.1 && getExtensionCurrentSpeedPercent() < -1.2){
//            powerToApply = -getExtensionCurrentSpeedPercent() * 0.13;
//        }
//        if(extensionPredictedPercent > 0.9 && getExtensionCurrentSpeedPercent() > 1.2){
//            powerToApply = -getExtensionCurrentSpeedPercent() * 0.13;
//        }






        extensionMotorPower = powerToApply;
    }


    /**
     * This are the different modes we can be on
     */
    public enum RunToPositionModes {
        powerControl,
        speedControl,
        positionControl,
    }
    //this is our runToPositionMode
    RunToPositionModes runToPositionMode = RunToPositionModes.speedControl;
    private RunToPositionModes lastRunToPositionMode = RunToPositionModes.powerControl;
    /**
     * turns the extension motor on to either not using position (speed control) or using runToPosition
     */
    public void setRunToPositionMode(RunToPositionModes mode) {
        runToPositionMode = mode;
    }

    /**
     * Gets the run to position mode
     * @return
     */
    public RunToPositionModes getRunToPositionMode(){
        return runToPositionMode;
    }




    //returns what percent the myCollector EXTENSION is activated (0 = retracted, 1 = fully extended)
    public double getExtensionPercent(){
        return (getExtensionCurrPos()- EXTENSION_MIN)
                /(EXTENSION_MAX - EXTENSION_MIN);
    }

    /**
     * Gets the percentage we are trying to go to
     * @return a percentage value of it's range that we are TARGETED to go to, this is only if
     * we are on position mode, although this will still return something when in other modes
     */
    public double getTargetPercent(){
        return (extensionMotorTargetPosition- EXTENSION_MIN)
                /(EXTENSION_MAX - EXTENSION_MIN);
    }

    /**
     * Gets the offset value of the encoder
     * PLUS AN OFFSET IF WE START OUTSIDE OUR RANGE USE THIS ALWAYS
     * @return
     */
    public double getExtensionCurrPos() {
        return extensionMotorCurrentPosition + masterOffsetTicks;
    }


    /**
     * Gets the raw value of the encoder
     * @return the raw value of the encoder
     */
    public double getExtensionPosRAW(){
        return extensionMotorCurrentPosition;
    }



    /**
     * Sets the extension motor target position raw (in encoder ticks).
     * This only matters if you are on position mode
     * @param targetPosition ticks to go to
     */
    public void setExtensionMotorTargetPosTicks(int targetPosition){
        extensionMotorTargetPosition = targetPosition;
    }

    /**
     * Sets the extension motor target position as a percent
     * @param percent percent to go to
     */
    public void setTargetPosVarWithPercent(double percent){
        percent = Range.clip(percent,0,1);//clip just in case
        extensionMotorTargetPosition = (percent * (EXTENSION_MAX-EXTENSION_MIN) + EXTENSION_MIN);
        extensionMotorTargetPosition -= masterOffsetTicks;
    }



    /** Retracts the extension to 0 position using the built in pid on runtoposition */
    public void resetExtension(double margin){
        //use position mode
        setRunToPositionMode(RunToPositionModes.positionControl);
        //set the speed of the motor
        setExtensionPowerRaw(1);
        setTargetPosVarWithPercent(margin);
    }

    /** Extends the extension to full range using the built in pid on runtoposition */
    public void extendExtension(double margin){
        setRunToPositionMode(RunToPositionModes.positionControl);
        //set the speed of the motor
        setExtensionPowerRaw(1);
        setTargetPosVarWithPercent(1-margin);
    }



    /**
     * Sets the target percent of the extension, and will now use position mode
     * @param percent
     */
    public void setExtensionTargetPercent(double percent){
        //use position mode
        setRunToPositionMode(RunToPositionModes.positionControl);
        //set the speed of the motor
        setExtensionPowerRaw(1.0);
        setTargetPosVarWithPercent(percent);
    }





    /**
     * If you want to start out of your allowed range, call this
     */
    public void setCurrentPositionTicks(double position){
        //if position was zero, the offset would be negative of what we're currently reading
        masterOffsetTicks = position-extensionMotorCurrentPosition;
    }


    /**
     * This will reset the encoder but be careful
     * THIS WILL NOT RESET THE MASTER OFFSET
     */
    public void resetEncoder(){
        DcMotor.RunMode r = extensionMotor.getMode();
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(r);
    }





    /**
     * Returns how far our the extension sticks out from the origin of the robot
     */
    public double getCurrExtensionDistFromCenter(){
        return (getExtensionPercent() * LENGTH_EXTENDED) + OFFSET_FROM_CENTER;
    }

    /**
     * gets the myCollector location on the field
     */
    public FloatPoint getExtensionPosition(){
        //let's get the location of the myCollector
        double collectorExtensionXPos = (getCurrExtensionDistFromCenter() * Math.cos(myRobot.getAngle_rad()))
                + myRobot.getXPos();
        double collectorExtensionYPos = getCurrExtensionDistFromCenter() * Math.sin(myRobot.getAngle_rad())
                + myRobot.getYPos();
        return new FloatPoint(collectorExtensionXPos,collectorExtensionYPos);
    }
}
