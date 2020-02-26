package Hardware;

import android.os.SystemClock;
import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.openftc.revextensions2.ExpansionHubServo;

import HelperClasses.Robot;
import ReturnTypes.FloatPoint;
import RobotUtilities.MovementVars;
import RobotUtilities.MyMath;
import RobotUtilities.MyPosition;
import RobotUtilities.SpeedOmeter;

/**
 * Collector.java
 * This class extends Extension because it is an extension, but is specific to the collector.
 * We use the rev PIDs, not manual ones
 */
public class Collector extends Extension{
    //link to the main robot
    private Robot myRobot;


    /////////////CONSTANTS///////////////
    private double COLLECTOR_FORWARDS_SPEED = 1.0;
    private double COLLECTOR_BACKWARDS_SPEED = -1.0;


    public static double COLLECTOR_SLIP_DISTANCE_PER_CM_PER_SECOND = 0.03;//how far the myCollector will travel for each cm/s that it moves

    private final double COLLECTOR_EXTENSION_MAX = 1200;//max extension amount
    private final double COLLECTOR_EXTENSION_MIN = 200;//min extension amount (don't go before this)
    private final double COLLECTOR_EXTENSION_DECELERATION = 0.3;//250% deceleration on both ends (70% range is for decelerating)


    private final double DUMPER_ACTIVATION_TIME = 20;//how long the dumper will take to go from retracted to activated
    private final double DUMPER_RESET_TIME = 300;//opposite of above


    /**absolute servo position of the activated position*/
    public double TILT_ACTIVATED_POS = 0.03;
    /**absolute servo position of the retracted position*/
    public double TILT_RETRACTED_POS = 0.86;








    public static double collectorExtensionMasterOffset = 0;//the offset we have

    //the seconds of prediction to decelerate properly at the end of its range
    private static final double secondsPrediction = 0.1;


    //if this is true, the collector will stop itself from smashing walls
    private boolean stopBeforeWalls = false;


    /////////////////////////////////////


    /////////HARDWARE/////////////
    private RevMotor rollerMotor;
    private Servo dumperServo;


    public boolean hasEverSetServo = false;



    public double dumperServoMasterPosition = TILT_RETRACTED_POS;
    private double dumperServoMasterPositionLast = -1;

    /**
     * Initializes a new Collector
     * @param myRobot link to the robot
     * @param rollerMotor the roller motor
     * @param extensionMotor the extension motor
     * @param dumperServo dumper servo
     * @param startActivated
     */
    public Collector(Robot myRobot, RevMotor rollerMotor, RevMotor extensionMotor,
                     ExpansionHubServo dumperServo, boolean startActivated){
        super(myRobot,extensionMotor,69,42);

        //set our extension range
        setRange(COLLECTOR_EXTENSION_MIN,COLLECTOR_EXTENSION_MAX);
        //set the deceleration
        setPercentDeceleration(COLLECTOR_EXTENSION_DECELERATION);


        //reverse the motor while we still have a reference
        extensionMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rollerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);





        //link up our things
        this.myRobot = myRobot;
        this.rollerMotor = rollerMotor;
        this.dumperServo = dumperServo;


        setRollerPower(0);//set initial power to 0





        //set our collector tilt if we are activated or deactivated to avoid twitching
        if(startActivated){
            forceActivateRestart();
            dumperStateStartTime = SystemClock.uptimeMillis() - 100000;//we need to already be done
            setCollectorTilt(1.0);//set the tilt to 1 to activate
        }else{

            forceRetractRestart();
            dumperStateStartTime = SystemClock.uptimeMillis() - 100000;//we need to already be done
            setCollectorTilt(0.0);//set the tilt to 0
        }
        //now apply that
        update();



        //set our offset if it was saved
        masterOffsetTicks = collectorExtensionMasterOffset;

        //set the collector extension prediction time
        setEXTENSION_PREDICTION_TIME_SECONDS(secondsPrediction);
    }



    //use this to get the last power set to the servo
    private double rollerMotorCurrentPower = 0.0;

    /**
     * Gets the current power assigned to the roller
     * @return
     */
    public double getRollerMotorCurrentPower(){
        return rollerMotorCurrentPower;
    }



    /**sets the collector power*/
    public void setRollerPower(double power){
        rollerMotorCurrentPower = power;
    }





    public void setCollectorTilt(double tiltPercent){
        dumperServoMasterPosition = (tiltPercent * (TILT_ACTIVATED_POS - TILT_RETRACTED_POS)) + TILT_RETRACTED_POS;
    }
    /**
     * Returns the percent activated the dumper is
     */
    public double getTiltPercent(){
        return (dumperServoMasterPosition - TILT_RETRACTED_POS)/((TILT_ACTIVATED_POS- TILT_RETRACTED_POS));
    }


    /**
     * This assists activate dumper, since the movement is slow over time we need an enum with states
     */
    public dumperStates dumperState = dumperStates.setMe;





    //the average recent speed of the collector
    private double averageSpeed = 0;
    /**
     * Updates a weighted average of our speed
     */
    public void calcAverageSpeed() {
        //calculate how much time has elapsed
        double percentToCare = myRobot.elapsedMillisThisUpdate/300.0;
        //get the current speed
        double currSpeed = getExtensionCurrentSpeedPercent();
        averageSpeed = (1.0-percentToCare)*averageSpeed + (percentToCare * currSpeed);
    }


    /**
     * Sets the average speed (just do this to reset or to set an initial approximate value)
     * @param averageSpeed the speed to set to
     */
    public void setAverageSpeed(double averageSpeed){
        this.averageSpeed = averageSpeed;
    }


    /**
     * Gets the current average speed of the collector extension
     */
    public double getAverageSpeed(){
        return averageSpeed;
    }


    private enum dumperStates{
        setMe,//used so that init state variables can be called
        activating,
        retracting,
        autoAvoidBalls,
        autoDoMarker
    }

    //if the dumperState is finished (initialization occurs)
    private boolean dumperStateFinished = true;
    //the start time of each state
    private long dumperStateStartTime = 0;





    //this is our current roller state
    public rollerStates rollerState = rollerStates.off;
    public enum rollerStates {
        off,
        forwards,
        reverse,
        stopWhenEncoder,
        manualControl//this does nothing
    }
    /**
    * This is called every update and will set the roller power automatically
    * or not if we are state manual control
    */
    public void HandleRollerStates() {
        if(rollerState == rollerStates.off){
            setRollerPower(0);
        }
        if(rollerState == rollerStates.forwards){
            //if we are down, we can go full speed, otherwise it is slower
            setRollerPower(COLLECTOR_FORWARDS_SPEED);

        }
        if(rollerState == rollerStates.reverse){
            setRollerPower(COLLECTOR_BACKWARDS_SPEED);
        }
    }


    /**
     * Turns on the myCollector forwards
     */
    public void turnOnRoller(){
        rollerState = rollerStates.forwards;
    }

    /**
     * Turns off myCollector, this will keep setting power to 0
     */
    public void turnOffCollector(){
        rollerState = rollerStates.off;
    }

    /**
     * Reverses the myCollector
     */
    public void reverseCollector(){
        rollerState = rollerStates.reverse;
    }

    /**
     * Doesn't set power so that you can call setRollerPower
     */
    public void manualControl() {
        rollerState = rollerStates.manualControl;
    }




    //THESE TRACK THE LAST VALUES OF USB THINGS SO WE DON'T SET THEM UNLESS THERE IS A SIGNIFICANT CHANGE
    private double rollerMotorLastPower = 0;

    private long lastUpdateTime = 0;//time of the last update in millis
    private double extensionCurrSpeed = 0;//this is the speed of the extension in cm/s
    /**
     * CALLS INTERNAL METHODS AND UPDATES POWERS AND STUFF
     */
    public void update(){
        //stop before hitting a wall if necessary
//        if(stopBeforeWalls){
//            stopBeforeWalls();
//            stopBeforeWalls = false;
//        }

        super.update();//we are an extension so update that
        myRobot.telemetry.addLine("MY STUPID CURRENT POWER: " + extensionMotorPower);

        HandleRollerStates();
        MeasureExtensionSpeed();
        UpdateDumper();





        //SEE IF THE DUMPER SERVO POSITION HAS CHANGED SIGNIFICANTLY
        if(Math.abs(dumperServoMasterPosition-dumperServoMasterPositionLast) > 0.0005){
            /**Now apply the powers */
            dumperServo.setPosition(dumperServoMasterPosition);
            hasEverSetServo = true;
            dumperServoMasterPositionLast = dumperServoMasterPosition;
        }

        rollerMotor.setPower(rollerMotorCurrentPower);

    }




    /**
     * Call this on the init of a dumper state
     */
    private void initDumperStateVariables(){
        dumperStateFinished = false;
        dumperStateStartTime = SystemClock.uptimeMillis();

    }


    /**
     * Call this when you change the dumper state
     */
    private void nextDumperStage(){
        dumperStateFinished = true;
    }



    /**
     * ACTIVATES DUMPER (away from robot), starts the state and records time
     */
    public void activateDumper(){
        if(dumperState != dumperStates.activating){
            dumperState = dumperStates.activating;
            nextDumperStage();
        }
    }

    /**
     * RETRACTS DUMPER (Towards robot), starts the state and records time
     */
    public void retractDumper(){
        if(dumperState != dumperStates.retracting){
            dumperState = dumperStates.retracting;
            nextDumperStage();
        }
    }



    /**
     * This is called by init only to make sure the state is properly initialized
     */
    private void forceActivateRestart(){
        dumperState = dumperStates.activating;
        nextDumperStage();
    }
    /**
     * This is called by init only to make sure the state is properly initialized
     */
    private void forceRetractRestart(){
        dumperState = dumperStates.retracting;
        nextDumperStage();
    }



    /**
     * This is used in autonomous to avoid hitting the double sample white ball when retracting
     */
    public void autoAvoidBalls(){
        if(dumperState != dumperStates.autoAvoidBalls){
            dumperState = dumperStates.autoAvoidBalls;
            nextDumperStage();
        }
    }


    /**
     * This puts the collector down almost all the way but not all the way to avoid hitting
     * the ground while deploying the team marker
     */
    public void deployMarker(){
        if(dumperState != dumperStates.autoDoMarker){
            dumperState = dumperStates.autoDoMarker;
            nextDumperStage();
        }
    }




    //this is the last dumper update time
    private long lastUpdateDumperTime = 0;

    /**
     * Handles dumper states and linear interpolation between them
     */
    private void UpdateDumper() {
        long currTime = SystemClock.uptimeMillis();
        long elapsedMillis = currTime - lastUpdateDumperTime;
        if(dumperState == dumperStates.activating || dumperState == dumperStates.autoDoMarker){
            if(dumperStateFinished){
                initDumperStateVariables();
            }
            //go from 0 to 1 in DUMPER_ACTIVATION_TIME, but clip at end to be 0 to 1
            double percent = (double) (currTime-dumperStateStartTime)/DUMPER_ACTIVATION_TIME;

            //we won't go all the way down if we are deploying for autonomous
            double maxPercent = dumperState == dumperStates.activating ? 1.0 : 0.65;
            percent = Range.clip(percent,0,maxPercent);
            if(percent > getTiltPercent()){
                setCollectorTilt(percent);
            }
        }

        /**
         * NOW TO DEAL WITH RETRACTING
         */
        if(dumperState == dumperStates.retracting || dumperState == dumperStates.autoAvoidBalls){
            if(dumperStateFinished){
                initDumperStateVariables();
            }

            double currPercent = getTiltPercent();
            double targetPercent = 0;

            //go towards the target position
            double deltaPercent = (currPercent < targetPercent ? 1.0 : -1.0)*
                    ((double) elapsedMillis/DUMPER_RESET_TIME);

            //add deceleration
            deltaPercent *= Range.clip(Math.abs(currPercent - targetPercent)/0.3,
                    0,1);




//            myRobot.telemetry.addLine("\n targetPercent: " + targetPercent);
//            myRobot.telemetry.addLine("currPercent: " + currPercent);
//            myRobot.telemetry.addLine("deltaPercent: " + deltaPercent + "\n");



            //now we can set the collector tilt
            setCollectorTilt(Range.clip(currPercent + deltaPercent,0,1));
//            myRobot.telemetry.addLine("CollectorDumperPercent: " + getTiltPercent());

        }


        lastUpdateDumperTime = currTime;
    }

    /**
     * This method returns if were are in a second range
     * @param encoderPosition
     * @return
     */
    private boolean inTollerantRange(double encoderPosition) {
        //0.506, 0.693
        //0.931, 0.232
        boolean inRange1 = encoderPosition > 0.506 && encoderPosition < 0.693;
        boolean inRange2 = encoderPosition > 0.931 || encoderPosition < 0.232;//this one spans the wrap around
        return inRange1 || inRange2;
    }



    private double extensionLastPosition = 0;//the last position of the collector extension
    /**
     * This keeps the speed of the extension to be used however (cm/s)
     */
    private void MeasureExtensionSpeed() {
        long currTime = SystemClock.uptimeMillis();
        //measure the extension speed
        double elapsedTime = (currTime - lastUpdateTime)/1000.0;
        if(elapsedTime > 0.02){
            double extensionCurrPosition = getCurrExtensionDistFromCenter();
            //speed is distance/time, so do exactly that
            extensionCurrSpeed = (extensionCurrPosition - extensionLastPosition)/elapsedTime;

            //save these variables for next update
            extensionLastPosition = extensionCurrPosition;
            lastUpdateTime = currTime;
        }
    }
    /**gets the myCollector current speed in cm/s */
    public double getExtensionCurrSpeed(){
        return extensionCurrSpeed;
    }



    /** This will go as fast as physically possible to any location with braking */
    public void extensionGoFullSpeedToPercent(double percent){
        //oscillate around the target percent
        extensionMotorPower = getExtensionPercent() < percent ? 1.0 : -1.0;



        //if extended is less than min, invert everything
        extensionMotorPower  *= COLLECTOR_EXTENSION_MAX > COLLECTOR_EXTENSION_MIN ? 1.0 : -1.0;
    }





    /**
     * Activates position mode and goes to a target distance away from the robot
     * @param length: target distance in cm away from the robot the collector should be
     */
    public void setExtensionTargetLength(double length){
        double percent = (length - OFFSET_FROM_CENTER)/(LENGTH_EXTENDED);
        percent = Range.clip(percent,0,1);
        setExtensionTargetPercent(percent);
    }


    /**
     * Returns the length of the extension if it were fully extended
     */
    public double getFullLengthCollector(){
        return LENGTH_EXTENDED + OFFSET_FROM_CENTER;
    }


    /** USE THIS TO CALIBRATE THE SLIP PREDICTED DISTANCE DEPENDING ON SPEED */
    public void calibrateSlipDistance(double changeInLength, double initialSpeedCm_p_S) {
        COLLECTOR_SLIP_DISTANCE_PER_CM_PER_SECOND = changeInLength/initialSpeedCm_p_S;
    }




    /*
    The approximate time in seconds it takes to deploy the collector. This is used to anticipate
    when to deploy the collector in deployWhenOverCrater. Since the robot could be moving
     */
    private static double TIME_TO_DEPLOY_COLLECTOR = 0.05;

    /**
     * This will deploy the collector dumper automatically when over the crater
     * @return if it has been deployed
     */
    public boolean deployWhenOverCrater() {
        /*

        -------------------------
        |                       |
        |                       |
        |                       |
        |                       |
        |                       |
        |                       |
        |                       |
        |--                     |
        |  --                   |
        |    --                 |
        ------*------------------
         */
        /* FIRST LINE IS THE CRATER */
        double y1 = 0;
        double x1 = 127;
        double m1 = -(157.0/127);


        /* SECOND LINE DEFINED BY THE ROBOT */
        //slope of the second line in the tangent of the angle
        double m2 = Math.tan(myRobot.getAngle_rad());
        double y2 = myRobot.getYPos();
        double x2 = myRobot.getXPos();


        /*Take the intercept of the two points */
        FloatPoint intercept = MyMath.lineIntersecion(
                new FloatPoint(x1,y1),m1,new FloatPoint(x2,y2),m2);


        //get the distance we are away from the intercept
        double distanceFromRobotToDeploy = Math.hypot(intercept.x-myRobot.getXPos(),
                intercept.y-myRobot.getYPos());

        //get how far away the collector is from the center of the robot
        double collectorCurrLength = getCurrExtensionDistFromCenter();

        //get the delta distance until the deploy
        double distanceUntilDeploy = distanceFromRobotToDeploy - collectorCurrLength;

        myRobot.telemetry.addLine("Intersection x: " + intercept.x +
                " y: " + intercept.y + "dist: " + distanceFromRobotToDeploy);

        //how far the collector will travel is the speed (cm/s) * the time in second (giving cm)
        double howFarTheCollectorWillTravelDuringDeploy =
                (SpeedOmeter.getSpeedY() + getExtensionCurrentSpeedCmpS())
                * TIME_TO_DEPLOY_COLLECTOR;
//        Log.d("ERROR_LOG", "hor far will go deploy: " + howFarTheCollectorWillTravelDuringDeploy);
//
//        Log.d("ERROR_LOG", "speed Y: " + SpeedOmeter.getSpeedY());
//        Log.d("ERROR_LOG", "speed collector: " + getExtensionCurrentSpeedCmpS());

        //now subtract how far it will go during the deploy from the distance to the deploy point
        double distanceUntilDeployWithSpeed = distanceUntilDeploy -
                howFarTheCollectorWillTravelDuringDeploy;




        double targetDeployAddition = 10;//positive makes it deploy earlier, negative is later
        double deltaAngleFromPerpendicular =
                MyPosition.subtractAngles(myRobot.getAngle_rad(),Math.atan(m1)-Math.toRadians(90));
        double additionForAngle = (Math.abs(deltaAngleFromPerpendicular) / Math.toRadians(30)) * 13;
        targetDeployAddition -= additionForAngle;

        myRobot.telemetry.addLine("\n\ndelta angle from perp : " +
                Math.toDegrees(deltaAngleFromPerpendicular));
        myRobot.telemetry.addLine("addition to target deploy : " +
                additionForAngle + "\n\n");

        //if the relative distance with guessing based on how fast we are going is crosses 0, deploy
        boolean deploy = distanceUntilDeployWithSpeed <= targetDeployAddition;//you can tune this number to make it earlier or later

        if(deploy){
            activateDumper();
        }
        return deploy;
    }






    /**
     * Call this with default minimum powers
     * @param power the power you want
     */
    public void setExtensionPowerNice(double power){
        double predictedPosition = getExtensionPercent() +
                getExtensionCurrentSpeedPercent() * 0.4;

        //the amount this will overshoot the end of its range
        double overShootEndRangePrediction = predictedPosition - 1;
        if(predictedPosition > 1 && getExtensionCurrSpeed() > 0.2 && getExtensionPercent() < 1){
            power -= overShootEndRangePrediction * 1.5;
        }

//        //the amount we are expected to overshoot the start of its range
//        double overShootStartRangePrediction = predictedPosition - 0;
//        if(predictedPosition < 0 && getExtensionCurrSpeed() < -0.3 && getExtensionPercent() > 0){
//            power -= overShootStartRangePrediction * 2;
//        }


        setExtensionPowerNice(power, 0.05,0.20);

        //don't allow moving in on the lift
        if(myRobot.myLift.getExtensionPercent() < 0.15 && power < 0 && getExtensionPercent() < 0){
            setExtensionPowerRaw(0);
        }



        myRobot.telemetry.addLine("currPercent: " + getExtensionPercent());

        //since our range is so far, don't allow any power past it's absolute max
        if(getExtensionPercent() >= 1.054 && power > 0){
            setExtensionPowerRaw(0);
        }
    }


    /**
     * This method makes sure the collector doesn't hit any walls and stops before them
     */
    public void stopBeforeWalls(){
        //get where the collector is on the field
        FloatPoint collectorLocation = getExtensionPosition();
        if((collectorLocation.x < 5 || collectorLocation.y < 5) && extensionMotorPower > 0){
            extensionMotorPower = Range.clip(extensionMotorPower,-0.15,0.15);
        }
    }





    /**
     * This override is debugging at 0.8 power
     * @param margin
     */
    @Override
    public void resetExtension(double margin){
        super.resetExtension(margin);
        setExtensionPowerRaw(0.8);
    }



    /**
     * Sets the current position in ticks. So if we were to then get the position it would return
     * what we set it to here. We override the extension method because we want to save the
     * offset in a static variable between autonomous and teleop. If we had masterOffsetTicks
     * (an extension class variable) as static, all extensions would have the same masterOffsetTicks.
     * Therefore we have our own.
     * @param ticks the reading the encoder should read in ticks after this is done
     */
    @Override
    public void setCurrentPositionTicks(double ticks){
        super.setCurrentPositionTicks(ticks);
        collectorExtensionMasterOffset = masterOffsetTicks;
    }


    /**
     * Gets the current speed of the collector in centimeters per second
     * @return speed in cm/s
     */
    public double getExtensionCurrentSpeedCmpS(){
        return getExtensionCurrentSpeedPercent() * LENGTH_EXTENDED;
    }





    //The current location the collector will try to be at
    private double collectorAbsoluteTargetRadius = 0;
    /**
     * This will save the current radius from the robot the collector extension is
     * and will maintain it.
     */
    public void maintainCollectorCurrentRadius(){
        collectorAbsoluteTargetRadius = getCurrExtensionDistFromCenter();
    }

    /**
     * This will adjust the absolute target radius by a distance.
     * @param lengthCM: the amount of centimeters to increase or decrease the radius
     */
    public void adjustAbsoluteTargetRadius(double lengthCM){
        collectorAbsoluteTargetRadius += lengthCM;
    }















    private boolean powerSignLast = false;
    private boolean inOuterLimit = false;


    /**
     * Call this to maintain the current location automatically
     */
    public void updateMaintainCurrentLocation() {
        //subtract the amount the robot has moved in the y dimension to make this absolute
        collectorAbsoluteTargetRadius -= MyPosition.currentTravelYDistance;

        /**
         * This will maintain a target radius away from the robot
         */
        double currentRadius = getCurrExtensionDistFromCenter();
        double prediction = SpeedOmeter.getSpeedY() * 0.20;


        if (Math.abs(MovementVars.movement_y) < 0.05) {
            prediction *= 0.3;
        }
        currentRadius += prediction;
        myRobot.telemetry.addLine("CURRENT MOVEMENT_Y: " + MovementVars.movement_y);
        double deltaRadius = collectorAbsoluteTargetRadius - currentRadius;
        double maxPower = 1.0;
        double decelerationCM = 33;
        double power = Range.clip((deltaRadius / decelerationCM) * maxPower, -maxPower, maxPower);


        double extensionLimitPredictionPercent =
                (getExtensionCurrentSpeedPercent() * EXTENSION_PREDICTION_TIME_SECONDS) + getExtensionPercent();


        boolean powerSign = power > 0;//true if power is positive


        //stop if close to the edges
        if (extensionLimitPredictionPercent < 0.05 && power < 0 || extensionLimitPredictionPercent > 0.95 && power > 0) {
            inOuterLimit = true;
        }
        if (inOuterLimit && powerSign != powerSignLast) {
            inOuterLimit = false;//if the power undergoes a sign change, and we are in an outer limit, turn that off
        }

        if (inOuterLimit) {
            power = 0;
            maintainCollectorCurrentRadius();//don't allow adjusting the radius behind this
        }


        setRunToPositionMode(RunToPositionModes.speedControl);
        setExtensionPowerRaw(power);


        powerSignLast = powerSign;//save the sign for next time
    }


    /**
     * Call this every update if you want the collector to avoid smashing into walls
     */
    public void enableWallSafety(){
        stopBeforeWalls = true;
    }

}


