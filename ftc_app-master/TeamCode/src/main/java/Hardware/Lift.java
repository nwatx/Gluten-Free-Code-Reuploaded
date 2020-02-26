package Hardware;

import android.os.SystemClock;
import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import HelperClasses.Robot;
import ReturnTypes.FloatPoint;
import RobotUtilities.MyMath;

/**
 * This class is used to control the lift extension
 */
public class Lift extends Extension{
    /////////HARDWARE//////////
    private Servo dumperServo;
    private Servo releaseServo;
    ///////////////////////////



    public static double liftExtensionMasterOffset = 0;//the offset we have


    //this is the distance we are expected to slip per (percent per second)
    public double SLIP_PERCENT_PER_SPEED_PERCENT = 0.1;


    //PIDs: 140,50,255


    /////////////////////////CONSTANTS AND STUFF////////////////////////////

    private final double FEEDER_EXTENSION_MAX = 650;
    private final double FEEDER_EXTENSION_MIN = -7;
    private final double FEEDER_EXTENSION_DECELERATION = 0.9;//90% deceleration

    public double DUMP_SERVO_ACTIVATED = 0.72;
    public double DUMP_SERVO_GOING_UP = 0.32;
    /** Starts in 18 inches like a good little robot */
    public double DUMPER_SERVO_18INCHES = 0.38;

    /** Where we go when at the bottom of our range */
    public double DUMP_SERVO_ZERO = 0.142;
    private double dumperServoMasterPosition = DUMP_SERVO_GOING_UP;
    //this is the last position of the dumper servo so we avoid setting it's position more than we have to
    private double dumperServoMasterPositionLast = -1;//make sure to set this

    private final double secondsPredictionTime = 0.1;
    ////////////////////////////////////////////////////////////////////////


    /**
     * Gets the master position of the servo.
     * @return a double that is the position
     */
    public double getDumperServoMasterPosition(){
        return dumperServoMasterPosition;
    }






    /**Our current state*/
    private myStates myState = myStates.manualPowerControl;


    /**These are all our states.*/
    public enum myStates{
        manualPowerControl,
        movingUpLift,
        resettingLift,
    }


    /** current dumper state */
    public dumperStates dumperState = dumperStates.manualControl;

    /**
     * All the possible dumper states
     */
    public enum dumperStates{
        initAuto,//starts in 18 inches
        manualControl,
        dumpingAdvanced
    }


    /**
     * Initializes a new lift object
     * @param myRobot link to robot
     * @param extensionMotor the extension motor
     * @param encoderMotor the encoder motor
     * @param dumperServo
     * @param releaseServo
     * @param startIn18Inches if we are starting in 18 inches like in auto
     */
    public Lift(Robot myRobot, RevMotor extensionMotor, RevMotor encoderMotor,
                Servo dumperServo, Servo releaseServo,boolean startIn18Inches){

        //extension length is negative since it goes backwards
        //offset is positive since it is forwards from the origin of the robot
        super(myRobot,extensionMotor,encoderMotor,-85,4);
        setRange(FEEDER_EXTENSION_MIN,FEEDER_EXTENSION_MAX);
        setPercentDeceleration(FEEDER_EXTENSION_DECELERATION);


        extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //this will dump the stuff
        this.dumperServo = dumperServo;
        this.releaseServo = releaseServo;
        //if we are starting in 18 inches, unrelease the minerals
        if(startIn18Inches){
            unreleaseMinerals();
            makeDumperFit();
        }else{
            //if we don't care, just release
            releaseMinerals();
            unDump();//start retracted
        }



        //set the prediction time of the lift to a bit more than the collector
        setEXTENSION_PREDICTION_TIME_SECONDS(secondsPredictionTime);
        myState = myStates.manualPowerControl;//we will be in manual power control to start


        //set the masterOffsetTicks based on the saved value
        masterOffsetTicks = liftExtensionMasterOffset;

        //reset the error sum for pid control
        liftErrorSum = 0;

        setExtensionPowerRaw(0);

    }



    //the target position the release servo will go to when called to release
    public double RELEASE_SERVO_RELEASED = 0.9;
    //the target position the release servo will go to when called to unrelease
    public double RELEASE_SERVO_UNRELEASED = 0.1;



    //the last position of the releaseServo
    private double releaseServoLastPosition = -1.0;

    //the current position of the releaseServo
    private double releaseServoCurrentPosition = RELEASE_SERVO_RELEASED;




    /**
     * Activates the release servo to release the minerals
     */
    public void releaseMinerals(){
        releaseServoCurrentPosition = RELEASE_SERVO_RELEASED;
    }

    /**
     * Retracts the release servo
     */
    public void unreleaseMinerals(){
        releaseServoCurrentPosition = RELEASE_SERVO_UNRELEASED;
    }


    /**
     * Call this at the beginning of autonomous to make sure we are starting within
     * the size requirement
     */
    public void makeDumperFit(){
        dumperState = dumperStates.initAuto;
        dumperServoMasterPosition = DUMPER_SERVO_18INCHES;
    }


    /**The last time we updated*/
    private long lastUpdateTime= 0;
    /**Our current time */
    private long currTimeMillis=0;
    private long elapsedTimeThisUpdate = 0;
    /**
     * Call this every update
     */
    public void update(){
        //get the current time
        currTimeMillis = SystemClock.uptimeMillis();
        //get the elapsed time in millis for this update
        elapsedTimeThisUpdate = currTimeMillis - lastUpdateTime;
        //save the last update time
        lastUpdateTime = currTimeMillis;



        //first we have to worry about the spikies
        avoidSpikesIfNecessary();

        //update our state machine
        updateStateMachine();

        //make sure we are always in power control
        setRunToPositionMode(RunToPositionModes.powerControl);

        //we are an extension so update that
        super.update();




        //we might need to update the dumping advanced
        if(dumperState == dumperStates.dumpingAdvanced){
            updateDumpAdvanced();
        }




        //only change the servo position if it has changed
        if(Math.abs(dumperServoMasterPositionLast-dumperServoMasterPosition) > 0.005){
            dumperServo.setPosition(1.0-dumperServoMasterPosition);
            dumperServoMasterPositionLast = dumperServoMasterPosition;
        }


        //only set the release servo if the position has changed
        if(Math.abs(releaseServoLastPosition-releaseServoCurrentPosition) > 0.005){
            releaseServo.setPosition(releaseServoCurrentPosition);
            releaseServoLastPosition = releaseServoCurrentPosition;
        }






        //if we didn't set the nice power this update
        if(!updatedSetExtensionSpeedNiceThisUpdate){
            //make sure we are only intending to go where we currently are
            targetExtensionPercent = getExtensionPercent();
        }
        //reset the error sum when we enter or exit this
        if(updatedSetExtensionSpeedNiceThisUpdate != isUpdatedSetExtensionSpeedNiceLastUpdate){
            liftErrorSum = 0;
            targetExtensionPercent = getExtensionPercent();
        }





        isUpdatedSetExtensionSpeedNiceLastUpdate = updatedSetExtensionSpeedNiceThisUpdate;
        updatedSetExtensionSpeedNiceThisUpdate = false;
    }


    /**
     * Updates our internal state machine
     */
    private void updateStateMachine() {
        //enforce that we are in power control mode if we are in manualPowerControl
        if(myState == myStates.manualPowerControl){

        }
        //if we are moving up the lift we need to update that
        if(myState == myStates.movingUpLift){
            updateMoveUpLift();
        }
        //handles if we are resetting the lift
        if(myState == myStates.resettingLift){
            updateResetLift();
        }
    }









    //the current target extension percent
    private double targetExtensionPercent = -1;//flag that it's uninitialized


    /**
     * USE THIS FOR LIFT
     * makes sure not to move the target percent too fast
     * @param targetExtensionPercent
     */
    public void setTargetExtensionPercent(double targetExtensionPercent){

        this.targetExtensionPercent = targetExtensionPercent;
    }

    /**
     * gets where we are trying to go
     * @return
     */
    public double getTargetExtensionPercent(){
        return targetExtensionPercent;
    }


    /**
     * if we have updated the extension position this update
     */
    private boolean updatedSetExtensionSpeedNiceThisUpdate = false;
    private boolean isUpdatedSetExtensionSpeedNiceLastUpdate = true;

    //the last error of the lift
    private double lastLiftError = 0;
    /**
     * Controls the lift target position like a speed control algorithm XD
     * @param targetSpeed the speed you want in extensions per second
     */
    public void setExtensionSpeedNice(double targetSpeed){

        //return if not in the right state
//        if(myState != myStates.manualPowerControl){
//            return;
//        }
        //just set to 0 if target speed is 0
        if(Math.abs(targetSpeed) < 0.00001){
            setExtensionPowerRaw(0);
            return;
        }

        updatedSetExtensionSpeedNiceThisUpdate = true;
        //get the elapsed seconds
        double elapsedSeconds = elapsedTimeThisUpdate/1000.0;

        targetSpeed = (targetSpeed >= 0 ? 1 : -1) * Math.pow(Math.abs(targetSpeed),1.5);
        //move the target extension percent
        setTargetExtensionPercent(targetExtensionPercent + (targetSpeed * elapsedSeconds/1.5));

        //calculates when to go to position mode, cutoff earlier when moving faster
        double predictedLiftPosition = getExtensionPercent() +
                getExtensionCurrentSpeedPercent() * (SLIP_PERCENT_PER_SPEED_PERCENT * 0.2);

        double currError = (targetExtensionPercent - predictedLiftPosition);
        //calculate the delta
        double delta = currError * 0.8;
        liftErrorSum += currError * (elapsedTimeThisUpdate/1500.0);
        delta += liftErrorSum;
        double differential = ((liftErrorSum - lastLiftError)/elapsedSeconds) * 0.0001;
        delta += differential;

//        myRobot.telemetry.addLine("differential: " + differential);
        lastLiftError = currError;



        double outputPower = Range.clip(delta / 0.10,-1,1) * 1.0;


        //if we are near the top or bottom, cap the output power
        if(getExtensionPercent() < 0.2 || getExtensionPercent() > 0.8){
            outputPower = Range.clip(outputPower,-0.7,0.7);
        }
        super.setExtensionPowerRaw(outputPower);
        myRobot.telemetry.addLine("ExtensionCurrPower: " + extensionMotorPower);
    }

    /**
     * Use this to reset the lift encoder
     */
    public void resetEncoder(){
        super.resetEncoder();
        setTargetExtensionPercent(getExtensionPercent());
    }


    /**
     * Sets the current position of the lift
     */
    public void setCurrentPositionTicks(double ticks){
        super.setCurrentPositionTicks(ticks);
        setTargetExtensionPercent(getExtensionPercent());
        liftExtensionMasterOffset = masterOffsetTicks;//save this
    }




    /**
     * ACTIVATES THE DUMPER SERVO
     */
    public void dump(){
        dumperState = dumperStates.manualControl;
        dumperServoMasterPosition = DUMP_SERVO_ACTIVATED;
    }

    /**
     * RETRACTS THE DUMPER SERVO
     */
    public void unDump(){
        dumperState = dumperStates.manualControl;
        dumperServoMasterPosition = DUMP_SERVO_GOING_UP;
    }



    /**
     * Once we are in the dump advanced this will calculate what we need to do
     * This will not just dump, but will calculate the tilt we should do
     */
    private void updateDumpAdvanced(){
        //get the current extension position
        FloatPoint extensionCurrPosition = getExtensionPosition();
        FloatPoint feedTarget = new FloatPoint(Robot.feedTargetX,Robot.feedTargetY);
        double distanceFromFeedTarget = Math.hypot(extensionCurrPosition.x-feedTarget.x,
                extensionCurrPosition.y - feedTarget.y);

        //the z of the lift extension when at 0 percent extended (the offset)
        double heightAt0 = 14;
        //this is the amount of z distance change when at 100% extension vs 0%
        double extensionHeightMax = 93.3-heightAt0;
        //now we can calculate our current height
        double extensionCurrentHeight = (getExtensionPercent() * extensionHeightMax) + heightAt0;


        //the z position you want to dump at
        double targetZPosFeed = 78-5;


        /**
         * At this point we have every 3d point we need to calculate the angle to dump
         * now comes how to calculate that
         */

        /*
         First calculate the angle along the ground towards the feeding target
         this is the X AXIS of the coordinate plane we are looking at.

         (0,0) will be the robot's location and 0 elevation
         */
        //so now we have to transform all the points to be on this plane
        double angleFromRobotToFeed = Math.atan2(feedTarget.y - myRobot.getYPos(),
                feedTarget.x - myRobot.getXPos());


        //we need the angle perpendicular so that we can transform 2d points into 1d points
        //via intersections at this angle
        double perpAngle = angleFromRobotToFeed + Math.toRadians(90);



        /*
        So essentially it's this:

                *
              f.
            *    .
          *        F
        R

        where R is the robot location, F is the feeder location,
        f is the intersection (the feeder position on the line)

         */
        FloatPoint intersectionFeeder =
                MyMath.lineIntersecion(new FloatPoint(myRobot.getXPos(),myRobot.getYPos()),
                        Math.tan(angleFromRobotToFeed),extensionCurrPosition,perpAngle);

        //get the distance the f point is from the robot (this is our weird x)
        double distFeederTransformFromRobot = Math.hypot(intersectionFeeder.x-myRobot.getXPos(),
                intersectionFeeder.y-myRobot.getYPos());



        /*
        NOW WE DO THE SAME THING FOR THE TARGET POSITION
         */

        FloatPoint intersectionTarget =
                MyMath.lineIntersecion(new FloatPoint(myRobot.getXPos(),myRobot.getYPos()),
                        Math.tan(angleFromRobotToFeed),feedTarget,perpAngle);
        double distTargetTransformFromRobot = Math.hypot(intersectionTarget.x-myRobot.getXPos(),
                intersectionTarget.y-myRobot.getYPos());



        //this angle with the ground the dumper needs to make
        double finalAngleToTarget =
                Math.atan2(targetZPosFeed-extensionCurrentHeight,
                        distTargetTransformFromRobot-distFeederTransformFromRobot);

        //ok now all that's left is to make the dump servo cover that distance

        //first some constants

        //the angle the feeder makes with the ground when the dump servo is at 0 percent
        double angleFeeder0Percent = Math.toRadians(37);
        //the angle the feeder makes with the ground when the dump servo is at 100 percent
        //(negative)
        double angleFeeder100Percent = Math.toRadians(-35);


        setTiltPercent((finalAngleToTarget-angleFeeder0Percent)
                /(angleFeeder100Percent-angleFeeder0Percent));
    }


    /**
     * Starts the dump advanced state, to always point the dumper correctly
     */
    public void startDumpAdvanced() {
        dumperState = dumperStates.dumpingAdvanced;
    }









    /**
     * Retracts the dumper all the way back to zero
     */
    public void retractToZero() {
        dumperServoMasterPosition = DUMP_SERVO_ZERO;
    }


    /**
     * Returns the percent activated the dumper is
     */
    public double getTiltPercent(){
        return (dumperServoMasterPosition - DUMP_SERVO_GOING_UP)/((DUMP_SERVO_ACTIVATED- DUMP_SERVO_GOING_UP));
    }


    /**
     * Sets the tilt position as a percent
     */
    public void setTiltPercent(double percent){
        percent = Range.clip(percent, 0,1);
        dumperServoMasterPosition = (percent * (DUMP_SERVO_ACTIVATED - DUMP_SERVO_GOING_UP)) + DUMP_SERVO_GOING_UP;
    }



    /**The time of the last time we move the dumper out of the
     * way when going up to avoid getting caught on the spikes*/
    private long lastGetDumperOutOfWayTime = 0;
    private boolean intendingToGoUpwardsLast = false;//if we were intending to go upwards the last update

    /**
     * If you are going upwards and haven't activated the dumper yet, move dat
     */
    private void avoidSpikesIfNecessary() {
        //we need to first figure out if we intend to go upwards right now
        //this is simply if we are applying positive power when not in position mode (speed or power)
        //otherwise we see if we have a non zero power and are targeted about the range of 2%
        boolean intendingToGoUpwards =
                (myState == myStates.manualPowerControl && extensionMotorPower > 0) ||
                myState == myStates.movingUpLift;

        //if the dumper percent is less than 0 (retracted position)
        //and we are going with a positive power we might need to clear the SPIKES
        if(getTiltPercent() < -0.0001 && intendingToGoUpwards){
            setTiltPercent(0);//go to at least zero percent to clear the spikes

            //if we weren't last time, record the time
            if(!intendingToGoUpwardsLast){
                //record the time so we don't move the extension until this clears
                lastGetDumperOutOfWayTime = SystemClock.uptimeMillis();
            }
        }
        intendingToGoUpwardsLast = intendingToGoUpwards;//record this

        if(getExtensionPercent() < 0.25 && !intendingToGoUpwards
                && dumperState != dumperStates.initAuto){
            retractToZero();
        }

    }








    //if we are going still fill power or not
    private double liftErrorSum = 0;


    /**
     * Resets the lift error sum
     */
    public void resetErrorSum(){
        liftErrorSum = 0;
    }
    /**Gets the current error sum of the lift*/
    public double getCurrLiftErrorSum(){ return liftErrorSum; }



    /**A scalar that slows or speeds up extending or resetting*/
    private double positionModePower = 0;
    /**Gets the maximum allowed power during position mode*/
    public double getPositionModePower() { return positionModePower; }


    /**
     * This will extend the lift at full speed, stop once it gets 70% of the way up
     * then wait to decelerate, mark that position and maintain that position
     * as long as it is greater than 95%
     */
    private void updateMoveUpLift() {
        //calculates when to go to position mode, cutoff earlier when moving faster
        double predictedLiftPosition = getExtensionPercent() +
                getExtensionCurrentSpeedPercent()
                        * (SLIP_PERCENT_PER_SPEED_PERCENT * 0.45);

        double targetLiftPosition = 0.95;
        double delta = targetLiftPosition - predictedLiftPosition;

        if(Math.abs(getExtensionCurrentSpeedPercent()) < 0.4 &&
                getExtensionPercent() > 0.3){
            liftErrorSum += delta * (elapsedTimeThisUpdate/300.0);
        }
        delta += liftErrorSum;


        //we can apply more negative power if we are moving fast but not if slow
        double minVal = getExtensionCurrentSpeedPercent() > 0.5 ? -1.0 : -0.4;
        super.setExtensionPowerRaw(Range.clip(delta / 0.17,minVal,1)
                * positionModePower);
    }






    /**
     * Call this to change the state machine the lift resetting sequence
     * @param power the percent power we will go at
     */
    public void resetLift(double power){
        //see if we need to initialize, otherwise do nothing
        if(myState != myStates.resettingLift){
            myRobot.telemetry.addLine("INITIALIZED RESETTING LIFT");
            myState = myStates.resettingLift;
            liftErrorSum = 0;
            positionModePower = power;
        }
    }

    /**
     * Call this to change the state machine of the lift moving up
     * @param power the percent power we will go at
     */
    public void moveUpLift(double power){
        if(myState != myStates.movingUpLift){
            myState = myStates.movingUpLift;
            liftErrorSum = 0;
            //save the power we are going at
            positionModePower = power;
        }
    }

    /**
     * Resets the lift extension
     */
    private void updateResetLift(){
        //calculates when to go to position mode, cutoff earlier when moving faster
        double predictedLiftPosition = getExtensionPercent() +
                getExtensionCurrentSpeedPercent()
                        * 0.065;

        double targetLiftPosition = 0.0;
        double delta = Range.clip((targetLiftPosition - predictedLiftPosition)
                /0.3,-1,1)*0.2;

        if(Math.abs(getExtensionCurrentSpeedPercent()) < 0.2 &&
                getExtensionPercent() < 0.5){
            liftErrorSum += delta * (elapsedTimeThisUpdate/125.0);
        }else{
            liftErrorSum = 0;
        }
        delta += liftErrorSum;


        //THIS IS A SANITY CHECK!
        //we can't build up too much integral or it will kill the motor
        if(Math.abs(liftErrorSum) > 1.2){
//            setCurrentPositionTicks(0);
            setExtensionPowerRaw(0);
            myState = myStates.manualPowerControl;
            liftErrorSum = 0;
            targetExtensionPercent = getExtensionPercent();
            return;
        }


//        delta *= Range.clip(Math.abs(getExtensionPercent())/0.15,0.2,1);

        //we can apply more POSITIVE (breaking) power if we are moving fast but not if slow
        double max = Math.abs(getExtensionCurrentSpeedPercent()) > 0.5 ? 1.0 : 0.4;
        double min = getExtensionPercent() < 0.02 ? -0.0 : -1.0;
        super.setExtensionPowerRaw(Range.clip(delta / 0.17,min,max) * positionModePower);

    }

    /**
     * Sets the extension power
     */
    public void setExtensionPowerRaw(double power){
        myState = myStates.manualPowerControl;
        super.setExtensionPowerRaw(power);//call the super method now
    }

}