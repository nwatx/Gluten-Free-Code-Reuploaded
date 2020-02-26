package HelperClasses;

import android.os.SystemClock;
import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.Range;

import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;

import org.firstinspires.ftc.robotcontroller.RobotUtilities.PiecewiseFunction;
import org.firstinspires.ftc.robotcontroller.Vision.FtcRobotControllerVisionActivity;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.RevExtensions2;

import java.text.DecimalFormat;
import java.util.ArrayList;

import Debugging.ComputerDebugging;
import Debugging.TimeProfiler;
import Globals.Globals;
import Hardware.AutoCollector;
import Hardware.AutoFeeder;
import Hardware.Collector;
import Hardware.DriveTrain;
import Hardware.HangMechanism;
import Hardware.Lift;
import Hardware.LoadingSensors;
import Hardware.RevMotor;
import RobotUtilities.MovementEssentials;
import RobotUtilities.MovementEssentials.movementResult;
import RobotUtilities.MyPosition;
import RobotUtilities.SpeedOmeter;
import RobotUtilities.TelemetryAdvanced;
import teamcode.Auto1;

import static RobotUtilities.MovementEssentials.pointAngle;
import static RobotUtilities.MovementVars.movement_turn;
import static RobotUtilities.MovementVars.movement_x;
import static RobotUtilities.MovementVars.movement_y;
import static RobotUtilities.MyPosition.AngleWrap;
import static RobotUtilities.MyPosition.subtractAngles;
import static RobotUtilities.MyPosition.worldAngle_rad;
import static RobotUtilities.MyPosition.worldXPosition;
import static RobotUtilities.MyPosition.worldYPosition;

/**
 * This is the base class for opmodes that use the robot
 */
public class Robot extends TunableOpMode {
    public static boolean usingComputer = false;
    //////////////////////////////////CONSTANTS/////////////////////////
    public final double FIELD_LENGTH = 358.775;//the length of the fieldld in cm
    ////////////////////////////////////////////////////////////////////


    private RevBulkData revExpansionMasterBulkData;
    private RevBulkData revExpansionSlaveBulkData;
    //these are the expansion hub objects
    private ExpansionHubEx revMaster;
    private ExpansionHubEx revSlave;
    //holds all the rev expansion hub motors
    private ArrayList<RevMotor> allMotors = new ArrayList<>();

    ComputerDebugging computerDebugging;

    public Collector myCollector;
    public Lift myLift;
    public AutoFeeder myAutoFeeder;
    public AutoCollector myAutoCollector;
    public HangMechanism hangMechanism;

    //use this to format decimals lol
    public DecimalFormat df = new DecimalFormat("#.00");
    private DriveTrain myDriveTrain;
    public LoadingSensors myLoadingSensors;

//    public PitScannerInterface myPitScannerInterface;


    public static TelemetryAdvanced m_telemetry;


    //this will be a milisecond time
    public long currTimeMillis = 0;


    //set this to change the initial position of the collector dumper
    public static double initialCollectorDumperPosition = 0.0;
    /** If we need to fit within the sizing requirement on init. Set this. */
    private boolean startIn18Inches = false;

    /** Sets if we need to start within 18 inches or not */
    public void setStartIn18Inches(boolean startIn18Inches){ this.startIn18Inches = startIn18Inches; }



    /**
     * Called when the driver pushes the init button
     */
    @Override
    public void init() {
        Auto.masterMotorScale = 1.0;
        currTimeMillis = SystemClock.uptimeMillis();
        //we need to call this to initialize the extra features of rev
        RevExtensions2.init();


        //get the two expansion hubs themselves
        revMaster = hardwareMap.get(ExpansionHubEx.class,"Expansion Hub 4");
        revSlave = hardwareMap.get(ExpansionHubEx.class,"Expansion Hub 2");


        this.msStuckDetectInit = 5000;
        this.msStuckDetectInitLoop = 5000;
        telemetry.setItemSeparator("");
        telemetry.setMsTransmissionInterval(250);
        telemetry.setAutoClear(true);


//        m_telemetry = new TelemetryAdvanced(55,46);//59,53);


        if(usingComputer){
            //use this to debug things
            computerDebugging = new ComputerDebugging();
        }







        ///GET ALL THE DRIVE TRAIN MOTORS AND ADD THEM TO ALL MOTORS///
        RevMotor tl = new RevMotor((ExpansionHubMotor) hardwareMap.get("motorTL"),true);
        RevMotor tr = new RevMotor((ExpansionHubMotor) hardwareMap.get("motorTR"),true);
        RevMotor bl = new RevMotor((ExpansionHubMotor) hardwareMap.get("motorBL"),true);
        RevMotor br = new RevMotor((ExpansionHubMotor) hardwareMap.get("motorBR"),true);
        //add them to all motors
        allMotors.add(tl);
        allMotors.add(tr);
        allMotors.add(bl);
        allMotors.add(br);
        //now we can initialize the myDriveTrain
        myDriveTrain = new DriveTrain(tl, tr, bl, br);


        //initialize our PitScannerInterface
//        myPitScannerInterface = new PitScannerInterface(this);


        RevMotor collectorMotor =
                new RevMotor((ExpansionHubMotor) hardwareMap.get("collector"), false);
        RevMotor collectorExtensionMotor =
                new RevMotor((ExpansionHubMotor) hardwareMap.get("extension"), false);
        allMotors.add(collectorMotor);
        allMotors.add(collectorExtensionMotor);


        myCollector = new Collector(this,collectorMotor,collectorExtensionMotor,
                (ExpansionHubServo) hardwareMap.get("collectorDumperServo"),
                initialCollectorDumperPosition > 0.5);

        myCollector.extensionMotor.myMotor.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION,
                new PIDCoefficients(12,0.19,0));



        ModernRoboticsI2cRangeSensor left =
                (ModernRoboticsI2cRangeSensor) hardwareMap.get("loadingSensorLeft");
        ModernRoboticsI2cRangeSensor right =
                (ModernRoboticsI2cRangeSensor) hardwareMap.get("loadingSensorRight");
        left.setI2cAddress(I2cAddr.create8bit(0x26));
        myLoadingSensors = new LoadingSensors(left,right,this);




        //now we can do the lift
        RevMotor liftMotor =
                new RevMotor((ExpansionHubMotor) hardwareMap.get("liftMotor"), false);
        //the encoder for the lift is in the this drive train motor
        RevMotor encoderMotor = myDriveTrain.bottomRight;
        allMotors.add(liftMotor);
        myLift = new Lift(this,liftMotor,encoderMotor,
                (ExpansionHubServo) hardwareMap.get("liftDumperServo"),
                (ExpansionHubServo) hardwareMap.get("releaseServo"),startIn18Inches);
        myLift.extensionMotor.myMotor.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION,
                new PIDCoefficients(10,0.17,0));
        myLift.update();


        //now the hang mechanism and the motor
        RevMotor hangMotor =
                new RevMotor((ExpansionHubMotor) hardwareMap.get("hangMotor"),false);
        hangMechanism = new HangMechanism(hangMotor);
        allMotors.add(hangMotor);



        myAutoCollector = new AutoCollector(this,myCollector,myLift);

        myAutoFeeder = new AutoFeeder(this,myCollector,myLift,myAutoCollector);
        myAutoFeeder.setRegularDump();//regular dump by default




        //get the bulk data if you want to calibrate stuff
        getRevBulkData();


        //update it once to get a position
        myCollector.update();


        myLift.update();

    }

    @Override
    public void init_loop(){
//        giveCppFieldData();
        currTimeMillis = SystemClock.uptimeMillis();
        getRevBulkData();


        giveButtonPressStuff();
        currTimeMillis = SystemClock.uptimeMillis();


//        myLift.update();
        myCollector.update();


        giveOtherModulePositionData();


        //draw the robot on the computer if we are usingComputer
        if(usingComputer){
            ComputerDebugging.sendRobotLocation(this);
            ComputerDebugging.markEndOfUpdate();
        }
    }


    @Override
    public void start() {
        AutoFeeder.myState = AutoFeeder.myStates.waiting.ordinal();
        //give MyPosition our current positions so that it saves the last positions of the wheels
        //this means we won't teleport when we start the match. Just in case, run this twice
        for(int i = 0; i < 2 ; i ++){
            MyPosition.initialize(myDriveTrain.topRight.getCurrentPosition(),
                    myDriveTrain.topLeft.getCurrentPosition(),
                    myDriveTrain.bottomLeft.getCurrentPosition(),this);
        }
    }


    private TimeProfiler tp1 = new TimeProfiler(1000);
    private TimeProfiler tp2 = new TimeProfiler(1000);
    private TimeProfiler tp3 = new TimeProfiler(1000);
    private TimeProfiler tp4 = new TimeProfiler(1000);
    private TimeProfiler tp5 = new TimeProfiler(1000);
    private TimeProfiler tp6 = new TimeProfiler(1000);
    private TimeProfiler tp7 = new TimeProfiler(1000);
    private TimeProfiler tp8 = new TimeProfiler(1000);


    /**
     * The time of the last loop update in millis
     */
    private long lastLoopTime = 0;
    /**
     * Amount of time elapsed this update in millis
     */
    public int elapsedMillisThisUpdate = 0;


    /**
     * Called every update
     */
    @Override
    public void loop() {
        long timeBefore = SystemClock.uptimeMillis();
        telemetry.addLine("Num hardware writes: " + RevMotor.numHardwareUsesThisUpdate);
        RevMotor.markEndUpdate();//mark the end of an update for the rev motor class
        tp1.markStart();
        //get all the bulk data
        getRevBulkData();
        tp1.markEnd();

        long timeAfter = SystemClock.uptimeMillis();
        telemetry.addData("Bulk data time: ", (timeAfter-timeBefore));

        //update the computer if we are debugging
        updateComputerIfUsingComputer();




        //uses opmode tuner to tune servo positions if enabled
        tuneServoPositionsWithTuner();
//        tuneScalingFactorsWithTuner();
//        tuneRandomThings();
        //get the current time for everyone
        currTimeMillis = SystemClock.uptimeMillis();
        elapsedMillisThisUpdate = (int) (currTimeMillis - lastLoopTime);
        lastLoopTime = currTimeMillis;

        //update myAutoFeeder
        myAutoFeeder.update();
        //update autoCollector
        myAutoCollector.update();
        tp2.markStart();

        //apply the movement to the drivetrain
        myDriveTrain.ApplyMovement();

        tp2.markEnd();
        tp3.markStart();
        //update the collector
        myCollector.update();
        tp3.markEnd();

        tp4.markStart();
        //update the lift
        myLift.update();
        tp4.markEnd();
        //update our hangMechanism
        hangMechanism.update();

        tp5.markStart();
        //finally, update the PitScannerInterface
//        myPitScannerInterface.update();
        tp5.markEnd();


        tp6.markStart();
        updateLoadingSensorsIfNecessary();
        tp6.markEnd();

        //display the readings of the loading sensors for debugging
//        telemetry.addLine("\nLEFT LOADING: " + myLoadingSensors.leftSensorCurrentReading);
//        telemetry.addLine("RIGHT LOADING: " + myLoadingSensors.rightSensorCurrentReading + "\n");



        tp7.markStart();
        //Figures out when all the buttons are pressed for ease of use in other places
        giveButtonPressStuff();

        //this will call MyPosition's update and calculate our position
        MyPosition.giveMePositions(
                myDriveTrain.topRight.getCurrentPosition(),
                myDriveTrain.topLeft.getCurrentPosition(),
                myDriveTrain.bottomLeft.getCurrentPosition());

        tp7.markEnd();

        tp8.markStart();
        //calculate our current speed in every dimension
        SpeedOmeter.update();


        //tell the FtcRobotControllerVisionActivity the robot position
        giveOtherModulePositionData();
        tp8.markEnd();





//        telemetry.addLine("profile 1: " + tp1.getAverageTimePerUpdateMillis());
//        telemetry.addLine("profile 2: " + tp2.getAverageTimePerUpdateMillis());
//        telemetry.addLine("profile 3: " + tp3.getAverageTimePerUpdateMillis());
//        telemetry.addLine("profile 4: " + tp4.getAverageTimePerUpdateMillis());
//        telemetry.addLine("profile 5: " + tp5.getAverageTimePerUpdateMillis());
//        telemetry.addLine("profile 6: " + tp6.getAverageTimePerUpdateMillis());
//        telemetry.addLine("profile 7: " + tp7.getAverageTimePerUpdateMillis());
//        telemetry.addLine("profile 8: " + tp8.getAverageTimePerUpdateMillis());
    }

    /**
     * Sees if we need to update the loading sensors and updates them if so
     */
    private void updateLoadingSensorsIfNecessary() {
        //update the loading sensors only if they could be seeing something
        if(myCollector.rollerState == Collector.rollerStates.forwards &&
                (myCollector.getTiltPercent() > 0.9)){
            if(currTimeMillis - myLoadingSensors.getLastUpdateTimeMillis() > 33){
                myLoadingSensors.update();
            }
        }else{
            //check to see if we are in autonomous
            boolean isAuto = Auto1.programStage != Auto1.progStates.endDoNothing.ordinal();
            //if that isn't the case, we can still update them at a lower rate
            if(isAuto){
                if(currTimeMillis - myLoadingSensors.getLastUpdateTimeMillis() > 200){
                    myLoadingSensors.update();
                }
            }
        }
    }


    /**
     * Use this to tune stuff
     */
    private void tuneRandomThings() {
        AutoFeeder.secondsPredictionRetractCollector = getDouble("prediction");
        AutoFeeder.retractCollectorFastPower = getDouble("fastPower");
        AutoFeeder.collectorPositionModePower = getDouble("slowPower");
    }

    /**
     * This method send's data to the computer for debugging if enabled
     */
    private void updateComputerIfUsingComputer() {
        //only do this if usingComputer = true
        if(usingComputer){
            ComputerDebugging.markEndOfUpdate();
            //send back to the computer the debug info
            ComputerDebugging.sendRobotLocation(this);
//            for(int i = 0; i < FtcRobotControllerVisionActivity.numMinerals; i ++){
//                if(FtcRobotControllerVisionActivity.xPositions[i] > 0 &&
//                        FtcRobotControllerVisionActivity.yPositions[i] > 0){
//
//                    ComputerDebugging.sendKeyPoint(new FloatPoint(FtcRobotControllerVisionActivity.xPositions[i],
//                            FtcRobotControllerVisionActivity.yPositions[i]));
//                }
//            }
        }
    }


    /**
     * Use this to tune random things
     */
    private void tuneServoPositionsWithTuner() {
//        myLift.DUMP_SERVO_ACTIVATED = getDouble("tuneServo1");
//        myLift.DUMP_SERVO_GOING_UP = getDouble("tuneServo2");
//        myLift.DUMP_SERVO_ZERO = getDouble("tuneServo3");
//        myCollector.TILT_RETRACTED_POS = getDouble("Collector Retracted");
//        myCollector.TILT_ACTIVATED_POS = getDouble("Collector Activated");
//
//        AutoFeeder.collectorRetractionPosition = getDouble("extensionRetraction");


//        myCollector.TILT_PERCENT_ALL_THE_WAY_IN = getDouble("tuneServo4");


//        myLift.RELEASE_SERVO_RELEASED = getDouble("releasedPosition");
//        myLift.RELEASE_SERVO_UNRELEASED = getDouble("unreleasedPosition");

        if(gamepad1.dpad_left){
            myLift.unreleaseMinerals();
        }
        if(gamepad1.dpad_right){
            myLift.releaseMinerals();
        }
    }


    public void stop(){

    }


    private void giveButtonPressStuff() {
        ButtonPress.giveMeInputs(gamepad1.a,gamepad1.b,gamepad1.x,gamepad1.y,gamepad1.dpad_up,
                gamepad1.dpad_down,gamepad1.dpad_right,gamepad1.dpad_left,gamepad1.right_bumper,
                gamepad1.left_bumper,gamepad1.left_stick_button,gamepad1.right_stick_button,
                gamepad2.a,gamepad2.b,gamepad2.x,gamepad2.y,gamepad2.dpad_up,
                gamepad2.dpad_down,gamepad2.dpad_right,gamepad2.dpad_left,gamepad2.right_bumper,
                gamepad2.left_bumper,gamepad2.left_stick_button,gamepad2.right_stick_button);
    }


    /**
     * Kills all movement
     */
    public void stopMovement(){
        movement_x = 0;
        movement_y = 0;
        movement_turn = 0;
    }

    public static String movementYVisual =
                    "                   1" +
                    "                    " +
                    "                    " +
                    "                    " +
                    "                    " +
                    "                    " +
                    "                    " +
                    "                    " +
                    "1                   ";
    public static PiecewiseFunction movementYPiecewise = new PiecewiseFunction(movementYVisual);

    public static String movementXVisual =
            "                   1" +
                    "                    " +
                    "                    " +
                    "                    " +
                    "                    " +
                    "                    " +
                    "                    " +
                    "                    " +
                    "1                   ";
    public static PiecewiseFunction movementXPiecewise = new PiecewiseFunction(movementXVisual);

    public static String movementTurnVisual =
            "                   1" +
                    "                    " +
                    "                    " +
                    "                    " +
                    "                    " +
                    "                    " +
                    "                    " +
                    "                    " +
                    "1                   ";
    public static PiecewiseFunction movementTurnPiecewise = new PiecewiseFunction(movementTurnVisual);


    /**
     * Use this to tune the telemetry scaling factors with the FTC opmode tuner
     */
    public void tuneScalingFactorsWithTuner(){
        MyPosition.moveScalingFactor = getDouble("MoveScale");
        MyPosition.turnScalingFactor = getDouble("TurnScale");
        MyPosition.auxScalingFactor = getDouble("AuxScale");
        MyPosition.auxPredictionScalingFactor = getDouble("AuxPrediction");
    }

    /**
     * Accepts user input to control robot
     */
    public void ControlMovement() {
        //go faster with right bumper
        double masterScale = 0.5 + ((gamepad1.right_bumper ? 1 : 0) * (1.0-0.5));
        movement_y = -gamepad1.right_stick_y * masterScale;// * getDouble("y_move_scale");
        movement_x = gamepad1.right_stick_x * masterScale;// * getDouble("x_move_scale");
        movement_turn = -(gamepad1.left_stick_x + gamepad2.right_stick_x) * masterScale;// * getDouble ("rot_move_scale");

        movement_y = (movement_y >= 0 ? 1.0 : -1.0) * movementYPiecewise.getVal(Math.abs(movement_y));
        movement_x = (movement_x >= 0 ? 1.0 : -1.0) * movementXPiecewise.getVal(Math.abs(movement_x));
        movement_turn = (movement_turn >= 0 ? 1.0 : -1.0) * movementTurnPiecewise.getVal(Math.abs(movement_turn));
    }



    //these are the feeding positions from the crater side
    public static double craterAutoFeedDistance = 79.114 + 18-3;
    public static double craterTargetX = 175 + 15 - 1.5;
    public static double craterTargetY = 165.5-2;

    //these are the feeding position for the depot side
    public static double depotAutoFeedDistance = 93;
    public static double depotTargetX = 195;
    public static double depotTargetY = 165.5;



    //how far away from the feed target the robot will go to
    public static double autoFeedDistance = craterAutoFeedDistance;
    //x position of the target feeding location
    public static double feedTargetX = craterTargetX;
    //y position of the target feeding location
    public static double feedTargetY = craterTargetY;

    //use this to orbit while auto feeding
    public static double orbitAmount = 0;


    /**
     * This changes if we are feeding from the crater or from somewhere else (cubes only p much)
     * @param crater a boolean that is true if we are feeding from the crater
     */
    public void setCraterMode(boolean crater){
        if(crater){
            autoFeedDistance = craterAutoFeedDistance;
            feedTargetX = craterTargetX;
            feedTargetY = craterTargetY;
        }else{
            autoFeedDistance = depotAutoFeedDistance;
            feedTargetX = depotTargetX;
            feedTargetY = depotTargetY;
        }
    }



    /**
     * This will orbit the robot to make a target angle with the lander (standard is 30 degrees)
     * this only occurs while driving to auto feed
     * @param angle the target angle the robot should make with the feeding target
     */
    public void orbitToAngleWhileFeeding(double angle){
        //First, this is where we will be orbiting around
        double xCenter = feedTargetX;
        double yCenter = feedTargetY;

        //the absolute angle to the target feeding location (but we want to point 180 degrees from this)
        double angleToTarget = Math.atan2(yCenter-getYPos(),xCenter-getXPos());

        //calculate the delta angleToGo
        double angleToGo = MyPosition.subtractAngles(angleToTarget,angle);
        //straif if we are not at the target angle
        orbitAmount = 0.3 * Range.clip(angleToGo / Math.toRadians(10.0),-1,1);
    }


    /**
     * Returns how far off we are from our target radius in the feeding
     * @return the delta from the target auto feed radius
     */
    public double getDeltaRadiusForFeeding(){
        //calculate how far we currently are from the target
        double currentRadius = Math.hypot(feedTargetX-worldXPosition,
                feedTargetY-worldYPosition);

        //this is the target radius that we want to be
        double targetRadius = autoFeedDistance;

        //so calculate the delta that we have to go
        return targetRadius-currentRadius;
    }

    /**
     * This will change the target based on where we are feeding from ei the cube location
     * @param cube int from 0 to 2 of cube
     * @return
     */
    public boolean driveToAutoFeed(int cube){
        if(cube == 0){
            feedTargetX += 0;
            feedTargetY += 0;
            autoFeedDistance -= 1.5;
        }
        if(cube == 1){
            //feed a little bit closer for the middle cube
            autoFeedDistance -= 1.5;
            feedTargetY -= 0;
        }


        if(cube == 2){
            autoFeedDistance -= 1.5;//feed even closer for the right

            //First, this is where we will be orbiting around
            double xCenter = feedTargetX;
            double yCenter = feedTargetY;

            //the absolute angle to the target feeding location (but we want to point 180 degrees from this)
            double angleToTarget = Math.atan2(yCenter-getYPos(),xCenter-getXPos());

            //we want it to be 45 degrees to the target
            double targetAngleToTarget = Math.toRadians(25);
            //calculate the delta angleToGo
            double angleToGo = MyPosition.subtractAngles(targetAngleToTarget,angleToTarget);

            double deltaRadius = getDeltaRadiusForFeeding();
            if(deltaRadius > -25 && myAutoFeeder.isOkToOrbit()
                    && AutoFeeder.numAutoFeeds > 0){
                //straif if we are not at the 45 degree target angle
                orbitAmount = -0.6 * Range.clip(angleToGo
                        / Math.toRadians(20.0),-1,1);
            }

        }

        boolean done = driveToAutoFeed();
        if(cube == 0){
            feedTargetX -= 0;
            feedTargetY -= 0;
            autoFeedDistance += 1.5;

        }
        if(cube == 1){
            autoFeedDistance += 1.5;
            feedTargetY += 0;
        }
        if(cube == 2){
            autoFeedDistance += 1.5;
        }
        return done;
    }


    public static double targetAutoFeedAngle = Math.toRadians(30);


    /**
     * The I term to not get stuck in turning to target
     */
    private double autoFeedAngleErrorSum = 0;

    /**
     * Call this before auto feeding this resets the error sum
     */
    public void initAutoFeed(){
        autoFeedAngleErrorSum = 0;
    }






    /**
     * This moves the robot to be a fixed radius from the feeder and turns towards it
     * but also straifs to get to an optimal position
     */
    public boolean driveToAutoFeed(){
        //First, this is where we will be orbiting around
        double xCenter = feedTargetX;
        double yCenter = feedTargetY;

        //the absolute angle to the target feeding location (but we want to point 180 degrees from this)
        double angleToTarget = Math.atan2(yCenter-getYPos(),xCenter-getXPos());

        //we want it to be 45 degrees to the target
        double targetAngleToTarget = targetAutoFeedAngle;//Math.toRadians(30);
        //calculate the delta angleToGo
        double angleToGo = MyPosition.subtractAngles(targetAngleToTarget,angleToTarget);
        //straif if we are not at the 45 degree target angle
        double straifPower = 0.3 * Range.clip(angleToGo / Math.toRadians(10.0),-1,1);

        /*
         * CALL ORBIT MODE
         */
        double deltaRadius = orbitModeFeeder(orbitAmount);//orbitModeFeeder(-straifPower);

        boolean wasOrbiting = Math.abs(orbitAmount) > 0.1;
        orbitAmount = 0;//stop ....for next time

//        movement_x = MovementEssentials.minPower(movement_x,MovementEssentials.movement_x_min);
//        movement_y = MovementEssentials.minPower(movement_y,MovementEssentials.movement_y_min);

//        movement_y = MovementEssentials.minPower(movement_y,MovementEssentials.movement_y_min);


        /**
         * Now we have to deal with termination so re-calculate relative point angle
         */

        //subtract 180 to get where we want to point
        double angleToPoint = subtractAngles(angleToTarget,Math.toRadians(180));
        //now that we know what absolute angle to point to, we calculate how close we are to it
        double relativePointAngle = AngleWrap(angleToPoint-worldAngle_rad);





        return Math.abs(deltaRadius) < 5 &&
                Math.abs(relativePointAngle) < Math.toRadians(3)
                && !wasOrbiting;//&&Math.abs(angleToGo) < Math.toRadians(15)
    }




    private CurvePoint startAutoCollect;
    /**NEED TO CALL THIS BEFORE AUTO COLLECT */
    public void initAutoCollect(){
        MovementEssentials.initCurve();

        //create the first curve point which is where the robot was at the start of the curve
        startAutoCollect = new CurvePoint(getXPos(),getYPos(),
                0,0,0,0,0);
    }

    /**
     * DRIVES TO THE NEXT COLLECTING LOCATION
     * @param collectingLocation - holds information about where and how we want to go
     * @param distAwayFromCrater
     * @param approachDistance
     * @return
     */
    public boolean driveToAutoCollect(CollectingLocation collectingLocation, double distAwayFromCrater, double approachDistance) {
        ArrayList<CurvePoint> allCurvePoints = new ArrayList<>();
        allCurvePoints.add(startAutoCollect);



        //the approximate angle we are collecting from relative to the crater
        double angleFrom = (Globals.isCrater() ? -135 : 45);
        /**
         * This is where the final target is, which is slightly outside the crater
         */
        double xFinal = (-Math.cos(Math.toRadians(angleFrom)) * distAwayFromCrater)
                + collectingLocation.xApproach;
        double yFinal = (-Math.sin(Math.toRadians(angleFrom)) * distAwayFromCrater)
                + collectingLocation.yApproach;


        /**
         * Now we want an approaching point so that we can arrive at a better angle.
         * We know the second point is where we want to point to
         */
        //since slope is tan(angle), we can get the angle with atan (just absolute value for now)
        double collectAngle = collectingLocation.getAngle();


        //use negative since and cosine since we want the point to be before the last point
        double xApproach = (-Math.cos(collectAngle) * (approachDistance))+xFinal;
        double yApproach = (-Math.sin(collectAngle) * (approachDistance))+yFinal;




        double xFirst = (startAutoCollect.x + xApproach)/2;
        double yFirst = (startAutoCollect.y + yApproach)/2;
        double xSecond = (xFinal + xApproach)/2;
        double ySecond = (yFinal + yApproach)/2;





        //add the first point, which is half way between the start and the approach
        allCurvePoints.add(new CurvePoint(xFirst,yFirst,
                1.0, 1.0,17,50,
                Math.toRadians(60),0.0));


        //the second is halfway between the final and the approach
        allCurvePoints.add(new CurvePoint(xSecond,ySecond,
                1.0, 1.0,17,50,
                Math.toRadians(60),0.0));


        //this is the final destination, a little bit spaced away from the crater
        allCurvePoints.add(new CurvePoint(xFinal,yFinal,
                1.0,1.0,17,50,
                Math.toRadians(60),0));


        MovementEssentials.followCurve(allCurvePoints,Math.toRadians(90),false);



        double distanceToFinal = Math.hypot(getXPos()-xFinal,getYPos()-yFinal);
        if(distanceToFinal < 60){
            movementResult r = pointAngle(Math.atan2(collectingLocation.pointLocationY-getYPos(),
                    collectingLocation.pointLocationX-getXPos()),1.0,Math.toRadians(40));

//            double scaleMovement = 1.0- Range.clip(Math.abs(r.turnDelta_rad)/Math.toRadians(30),
//                    0,0.6);
//            movement_x *= scaleMovement;
//            movement_y *= scaleMovement;

            return Math.abs(r.turnDelta_rad) < Math.toRadians(10) && distanceToFinal <= 30;

        }
        return false;//we are not done yet
    }




    public double currRelativePointAngleAutoFeed = 0;
    /**
     * This will orbit the robot around the target position of the feeder so that the feeder
     * will end up in the same position but the user can still move the robot in a circle
     * @param xPower
     */
    public double orbitModeFeeder(double xPower){
        //First, this is where we will be orbiting around
        double xCenter = feedTargetX;
        double yCenter = feedTargetY;

        //the absolute angle to the target feeding location (but we want to point 180 degrees from this)
        double angleToTarget = Math.atan2(yCenter-getYPos(),xCenter-getXPos());
        //subtract 180 to get where we want to point
        double angleToPoint = subtractAngles(angleToTarget,Math.toRadians(180));
        //now that we know what absolute angle to point to, we calculate how close we are to it
        double relativePointAngle = AngleWrap(angleToPoint-worldAngle_rad);
        currRelativePointAngleAutoFeed = relativePointAngle;//save this for people

        if(Math.abs(SpeedOmeter.getDegPerSecond()) < 15){
            //increment the error sum
            autoFeedAngleErrorSum += (relativePointAngle/Math.toRadians(2.0))
                    * elapsedMillisThisUpdate/1000.0;
        }else{
            autoFeedAngleErrorSum = 0;
        }




        //so calculate the delta radius that we have to go
        double deltaRadius = getDeltaRadiusForFeeding();

        //NOW let's assume that going forwards will increase our radius
        //(we are pretty close to the target angle)
        if(Math.abs(relativePointAngle) < Math.toRadians(30)){

            //adjust for how fast we are currently moving
            double deltaWithSpeed = deltaRadius - SpeedOmeter.currSlipDistanceY();

            double maxYPower = 0.8;//maximum thrust allowed to use in y dimension
            movement_y = Range.clip((deltaWithSpeed / 14.0)
                    *maxYPower,-maxYPower,maxYPower);
            movement_y = MovementEssentials.minPower(movement_y,MovementEssentials.movement_y_min);
            movement_y *= Range.clip(Math.abs(deltaWithSpeed)/5,0,1);
        }
        //movement_x controlled by the user directly
        movement_x = xPower;


        /** NOW TO POINT USING MOVEMENT_TURN*/
        //how many radians required to go from point_speed to 0
        double decelerationRadians = Math.toRadians(18);
        //max turn speed
        double point_speed = 0.6;

        //minimum turning power allowed
        double minTurnPower = MovementEssentials.movement_turn_min;

        //also multiply the min turning power allowed
//        minTurnPower *= 1.0 + Math.abs(movement_x) * 2.0;
        //also multiply the movement_y
//        movement_y *= 1.0 + Math.abs(movement_x) * 3.0;

        //point ahead of time if we are moving
        relativePointAngle -= (movement_x / 0.3) * Math.toRadians(4);




        //adjust for angular velocity now
        double velocityAdjustedRelativePointAngle =
                AngleWrap(relativePointAngle-SpeedOmeter.currSlipAngle() * 0.7);




        //Scale down the relative angle by decelerationRadians and multiply by point speed
        double turnSpeed = (velocityAdjustedRelativePointAngle/decelerationRadians)*point_speed;
        //now just clip the result to be in range
        movement_turn = Range.clip(turnSpeed,-point_speed,point_speed);

        //min power the turn speed
        movement_turn = MovementEssentials.minPower(movement_turn,minTurnPower);

//        movement_turn += Range.clip(autoFeedAngleErrorSum,-1,1) * 0.2;


        //smooths down the last bit to finally settle on an angle
        movement_turn *= Range.clip(Math.abs(relativePointAngle)/
                Math.toRadians(3),0,1);



        return deltaRadius;
    }











    //Reports the numbers of our location for debugging
    public void reportPositionData(){
        telemetry.addLine("xPos: "+ df.format(worldXPosition) +
                " yPos: " + df.format(worldYPosition) +
                " Rot: " + df.format(Math.toDegrees(MyPosition.worldAngle_rad)));

//        telemetry.addLine("\nAdjusted: \n" +"xPos: "+ df.format(worldXPosition-137) +
//                " yPos: " + df.format(worldYPosition-137) +
//                " Rot: " + df.format(Math.toDegrees(subtractAngles(MyPosition.worldAngle_rad,Math.toRadians(-135)))) + "\n");
    }
    public void DrawVirtualField() {
        for(int i = 0; i < m_telemetry.size_y; i ++){
            telemetry.addLine(m_telemetry.getLine(i)
                    .replace(" ","  "));
//                    .replace("r", "█")
//                    .replace("+", "▓")
//                    .replace("|", "▒")
//                    .replace("-", "░")
//                    .replace(" ","╰"));
//            telemetry.addLine(m_telemetry.getLine(i));
        }
        telemetry.addLine("zoom:" + zoom);
    }


    public double getXPos(){
        return worldXPosition;
    }
    public double getYPos(){
        return worldYPosition;
    }
    public double getAngle_rad(){
        return worldAngle_rad;
    }
    public double getAngle_deg(){
        return Math.toDegrees(worldAngle_rad);
    }




    //VIRTUAL FIELD VARIABLES//
    public static double center_x = 0.5;
    public static double center_y = -0.5;
    public static double zoom = 1;
    long lastZoomVirtualFieldTime = 0;


    public double getZoomVirtualField(){
        return zoom;
    }


    public void zoomInVirtualField(){
        long currTime = SystemClock.uptimeMillis();
        double elapsedSeconds = ((double) currTime- lastZoomVirtualFieldTime)/1000.0;
        lastZoomVirtualFieldTime = currTime;
        //return if this is the first time in a while we have pushed this button
        if(elapsedSeconds > 0.2){return;}

        //it will take 1 second to zoom in 2x
        zoom *= 1 + elapsedSeconds;
    }
    //you can also manually set a zoom
    public void zoomVirtualField(double amount){
        zoom = amount;
    }
    public void zoomOutVirtualField(){
        long currTime = SystemClock.uptimeMillis();

        double elapsedSeconds = ((double) currTime- lastZoomVirtualFieldTime)/1000.0;
        lastZoomVirtualFieldTime = currTime;
        if(elapsedSeconds > 0.2){return;}
        zoom *= 1 - elapsedSeconds;
    }

    public void followRobotVirtualField(){
        center_x = worldXPosition/FIELD_LENGTH;
        center_y = worldYPosition/FIELD_LENGTH;
        center_y -= 1.0;
    }

    /**
     * Pans on the virtual field
     * @param amount the x amount of pan
     */
    public static void panXVirtualField(double amount){ center_x += amount; }
    public static void panYVirtualField(double amount){
        center_y += amount;
    }


    /**
     * Passes the robot location, zoom and center data to the C++ telemetry class for drawing
     */
    public void giveCppFieldData(){
        m_telemetry.clear();
        m_telemetry.drawField(center_x,center_y,zoom);
        m_telemetry.drawRobot(worldXPosition, worldYPosition, MyPosition.worldAngle_rad);
    }




    private long lastUpdateSlaveTime = 0;
    private long lastUpdateMasterTime = 0;

    /**
     * Gets all the data from the expansion hub in one command to increase loop times
     */
    public void getRevBulkData() {
//        boolean needToPollMaster = !AutoFeeder.canPollMasterAtLowerRate ||
//            currTimeMillis-lastUpdateMasterTime > 300;
//        if(needToPollMaster){
        RevBulkData newDataMaster;
        try{
            newDataMaster = revMaster.getBulkInputData();
            if(newDataMaster != null){
                revExpansionMasterBulkData = newDataMaster;
            }
        }catch(Exception e){
            //don't set anything if we get an exception
        }
        lastUpdateMasterTime = currTimeMillis;

//        }



        /*
            We don't always need to poll the slave rev hub if we know the collector and lift
            are not moving
         */
        boolean needToPollSlave = Math.abs(myCollector.getExtensionMotorPower()) > 0 ||
                Math.abs(hangMechanism.getCurrentPower()) > 0 ||
                Math.abs(myCollector.getExtensionCurrentSpeedPercent()) > 0.05 ||
                currTimeMillis - lastUpdateSlaveTime > 400;

        if(needToPollSlave){
            RevBulkData newDataSlave;
            try{
                newDataSlave = revSlave.getBulkInputData();
                if(newDataSlave != null){
                    revExpansionSlaveBulkData = newDataSlave;
                }
            }catch(Exception e){
                //don't set anything if we get an exception
            }
            lastUpdateSlaveTime = currTimeMillis;
        }



        /////NOW WE HAVE THE BULK DATA BUT WE NEED TO SET THE MOTOR POSITIONS/////
        for(RevMotor revMotor : allMotors){
            if(revMotor == null){continue;}
            if(revMotor.isMaster){
                if(revExpansionMasterBulkData != null){
                    revMotor.setEncoderReading(
                            revExpansionMasterBulkData.getMotorCurrentPosition(revMotor.myMotor));
                }
            }else{
                if(revExpansionSlaveBulkData != null){
                    revMotor.setEncoderReading(
                            revExpansionSlaveBulkData.getMotorCurrentPosition(revMotor.myMotor));
                }
            }
        }

    }

    /**
     * This gives the position data of the robot to FtcRobotControllerVisionActivity
     */
    private void giveOtherModulePositionData() {
        //set the FtcRobotControllerVisionActivity's world angle so it can calculate the crater angle
        FtcRobotControllerVisionActivity.worldAngle_rad = getAngle_rad();
        FtcRobotControllerVisionActivity.worldXPosition = getXPos();
        FtcRobotControllerVisionActivity.worldYPosition = getYPos();
    }
}

