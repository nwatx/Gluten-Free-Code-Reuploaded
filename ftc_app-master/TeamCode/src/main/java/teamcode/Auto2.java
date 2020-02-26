package teamcode;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.Vision.FtcRobotControllerVisionActivity;

import java.util.ArrayList;

import Debugging.TimeProfiler;
import Globals.Globals;
import Hardware.AutoFeeder;
import HelperClasses.Auto;
import HelperClasses.CollectingLocation;
import HelperClasses.CurvePoint;
import RobotUtilities.MovementEssentials;
import RobotUtilities.MovementEssentials.movementResult;
import RobotUtilities.MovementVars;
import RobotUtilities.MyPosition;

import static Hardware.Collector.RunToPositionModes;
import static RobotUtilities.MovementEssentials.followCurve;
import static RobotUtilities.MovementEssentials.gunToPosition;
import static RobotUtilities.MovementEssentials.pointAngle;
import static RobotUtilities.MovementVars.movement_y;
import static RobotUtilities.MyPosition.subtractAngles;


/**
 * This is the depot auto
 */
@Autonomous(name = "auto2", group = "auto2")
public class Auto2 extends Auto {



    //new epic
    private double SPEED_SCALE = 2.1;

    public enum progStates{
        landing,
        unlatching,
        turnTowardsMarker,
        extendingToMarker,//extends over and deposits marker
        deployMarker,
        turningToOurCube,//turns to the cube
        getOurCube,
        drivingToDepot,
        goToCollectMore,
        feedCollectedTrip,


        endDoNothing
    }
    /** State Variables */
    //if we have stalled the lift motor and figured out where the bottom of our range is yet
    private boolean hasCalibratedLiftPositionYet = false;
    private long liftCalibrationStartTime = 0;//time when we start to calibrate the lift
    private long depositMarkerTime = 0;//time when we reverse the collector to deposit the marker
    private boolean gotToTargetPlaceDrivingBack = false;//this is when
    private long activateDumperOurBallTime = 0;//the time when we put down the collector to get our ball


    //this is flagged when we have started the parking sequence
    private boolean startedParking = false;


    /** THESE ARE COMPENSATION VALUES FOR AVERAGES IN TELEMETRY ERROR */
    //these first two change our position after drive to marker
    private static final double compensateXDrivingToMarker = 0;
    private static final double compensateYDrivingToMarker = 0;

//    //these change our position after driving back
//    private static final double compensateXBackHome = 0;//-3 - compensateXDrivingToMarker;
//    private static final double compensateYBackHome = 0;//-6 - compensateYDrivingToMarker;

    private boolean compensate1Yet = false;//flags when we compensate for the first 1
//    private boolean compensate2Yet = false;//flags when we compensate for the second 1








    /**
     * When the driver presses init
     */
    @Override
    public void init(){
        //change the auto mode to auto2
        Globals.autoMode = Globals.AutoModes.auto2;

        initialCollectorDumperPosition = 0;
        super.init();
        //reset the hang mechanism encoder to 0
        hangMechanism.ResetEncoder();

        //We start the collector in an unusually far in position so calibrate to that
        myCollector.setCurrentPositionTicks(0);



        //this will set the sample locations in approximate locations
        FtcRobotControllerVisionActivity.initializeSampleLocations();


        //enable vision to scan samples
        FtcRobotControllerVisionActivity.linkToInstance.enableView();
    }



    //this is false when we are entering the collecting points and true when we are
    //moving the sample location in init of auto
//    private boolean movingSamples = false;


    @Override
    public void init_loop(){
        super.init_loop();
        telemetry.addLine("cubeLocation: " +
                FtcRobotControllerVisionActivity.getCubeLocation());

//        if(movingSamples){
        ControlSampleLocations();
//        }else{
//            //handles all the stuff to do with the user entering points in initialization of autonomous
//            InitAutoPointsHandler.update(gamepad1.left_stick_x,gamepad1.left_stick_y,getZoomVirtualField());
//        }


        //allow the gamepad to control where the virtual field is
//        controlVirtualFieldPosition();
//        DrawVirtualField();

        telemetry.addLine("collector percent: " + myCollector.getExtensionPercent());
        telemetry.addLine("collector position: " + myCollector.getExtensionCurrPos());
        telemetry.addLine("collector offset: " + myCollector.masterOffsetTicks);
    }




    private static int cubeLocation = 0;

    @Override
    public void start(){
        //first save the cubeLocation DON'T MOVE AFTER SUPER.START()!!!
        cubeLocation = FtcRobotControllerVisionActivity.getCubeLocation();

        super.start();//call the super method
        myAutoFeeder.setUseCubeLocationInAutoFeed(true);


        boolean isHomeCrater = false;//we are collecting from the other crater

        //WE WILL ALWAYS FEED FROM THE RIGHT//
        double craterPercent = 0.8;

        /**
         * Add a bunch of collecting locations with specifying where the robot needs to be
         * and just vary the actual point location
         */
        for(int i = 0; i < 180; i ++){
            CollectingLocation thisLocation = new CollectingLocation(craterPercent,isHomeCrater);
            myAutoCollector.addCollectingLocation(thisLocation);

            //the remainder will tell us the best location
            int remainder = i%4;
            switch (remainder){
                case 0:
                    break;
                case 1:
                    thisLocation.pointLocationX = FIELD_LENGTH-(-10);
                    break;
                case 2:
                    thisLocation.pointLocationX = FIELD_LENGTH-15;
                    break;
                case 3:
                    thisLocation.pointLocationX = FIELD_LENGTH-30;
                    break;
                default:
                    break;
            }
        }



        /*
         * Use this to use debugging mode
         */
        ///////////////////////////////////////////////////
        startDebugging();
        //////////////////////////////////////////////////

//        nextStage(progStates.drivingBack.ordinal());//GOING HOME
        nextStage(progStates.unlatching.ordinal());//just start from here for debugging

        //testing auto collect
//        nextStage(progStates.goToCollectMore.ordinal());
//        AutoFeeder.numAutoFeeds = 1;
//        MyPosition.setPosition(147.1, 143.2,Math.toRadians(-45));


//        programStage = progStates.landing.ordinal();//start from hang



        hasCalibratedLiftPositionYet = false;//mark that we haven't calibrated the lift
        startedParking = false;//we haven't started parking yet


        FtcRobotControllerVisionActivity.linkToInstance.disableView();

    }





    private static ArrayList<TimeProfiler> allTimeProfilers = new ArrayList<>();

    static {
        for(int i = 0; i < 10; i ++){
            allTimeProfilers.add(new TimeProfiler(500));
        }
    }


    @Override
    public void loop(){

        allTimeProfilers.get(3).markEnd();
        allTimeProfilers.get(3).markStart();
        telemetry.addLine("UPS: " + 1000.0/allTimeProfilers.get(3).getAverageTimePerUpdateMillis());



        telemetry.addLine("hasCalibratedLiftPositionYet? " + hasCalibratedLiftPositionYet);
        telemetry.addLine("cube location: " + cubeLocation);
        telemetry.addData("PROGRAM STAGE", programStage);


        allTimeProfilers.get(0).markStart();
        super.loop();
        allTimeProfilers.get(0).markEnd();


        allTimeProfilers.get(1).markStart();


//        followRobotVirtualField();
        //Display our location
        reportPositionData();
        allTimeProfilers.get(1).markEnd();


        //you can still use the virtual field for debugging
//        controlVirtualFieldPosition();

        //draw this first so we can draw over it
//        DrawVirtualField();


        allTimeProfilers.get(2).markStart();

        CalibrateLiftPosition();
        allTimeProfilers.get(2).markEnd();

        //extend into the crater at the end of auto
        if(!debugging){
            //if we are already in endDoNothing, we have already extended into the crater
            //so don't run the extend into crater when ready
            if(programStage != progStates.endDoNothing.ordinal() || startedParking){
                ExtendIntoCraterWhenReady();
            }
        }

        telemetry.addLine("average time profilier 0: " + allTimeProfilers.get(0).getAverageTimePerUpdateMillis());
        telemetry.addLine("average time profilier 1: " + allTimeProfilers.get(1).getAverageTimePerUpdateMillis());
        telemetry.addLine("average time profilier 2: " + allTimeProfilers.get(2).getAverageTimePerUpdateMillis());
        telemetry.addLine("average time profilier 3: " + allTimeProfilers.get(3).getAverageTimePerUpdateMillis());
    }

    /**
     * This extends into the crater if it the end of auto
     */
    private void ExtendIntoCraterWhenReady() {
        //get the elapsed time since the start of the program
        long elapsedSinceStart = currTimeMillis-programStartTime;//time since start of program
        long timeTillEnd = 30000-elapsedSinceStart;//get the time until the end of the program



        //this is how much time we want to allow to abort auto feed
        long retractAndAbortTime = 800;
        long parkingTime = 400;//time to allow the collector to extend into the crater





        //extend into the crater with 1 second left
        if(timeTillEnd < retractAndAbortTime){
            startedParking = true;
            //if it is not time to park yet
            if(timeTillEnd > parkingTime){
                //this will go from 1 to 0 and we will decelerate using it
                double decelerationPercent = Range.clip(timeTillEnd-parkingTime/
                        (retractAndAbortTime-parkingTime),0,1);

                //multiply the lift power by this constant
                myLift.setExtensionPowerRaw(myLift.getExtensionMotorPower() * decelerationPercent);

                stopMovement();
                nextStage(progStates.endDoNothing.ordinal());//stop doing stuff
                myCollector.retractDumper();
            }else{
                myAutoFeeder.abortAutoFeed();
                myAutoCollector.abortAutoCollect();
                myCollector.setExtensionTargetPercent(0.5);//extend the collector into the pit
            }
        }
    }


    //this will be recorded when we start the calibration
    private double liftStartingPositionTicks = 0;
    /**
     * Moves the lift down when appropriate to the bottom and calibrates itself
     */
    private void CalibrateLiftPosition() {
        if(hasCalibratedLiftPositionYet){return;}//return if we are done

        //only do this if we have gotten way off the lander
        //or we have already started
        if(myCollector.getExtensionPercent() > 0.05
                || liftCalibrationStartTime != 0){
            //flag the first update we do this
            if(liftCalibrationStartTime == 0){
                liftCalibrationStartTime = SystemClock.uptimeMillis();
                liftStartingPositionTicks = myLift.getExtensionCurrPos();
            }

            //for this we will use power control
            myLift.setRunToPositionMode(RunToPositionModes.powerControl);
            //TODO: TUNE CALIBRATION OF LIFT ON POWER CONTROL
            //now slowly move the lift down
            myLift.setExtensionPowerRaw(-0.45);


            //get the lift speed in ticks per second
            double liftCurrentSpeed = myLift.getExtensionCurrentSpeedTicks();

            boolean liftMoving = Math.abs(liftCurrentSpeed) > 2;

            double liftTravelTicks = Math.abs(myLift.getExtensionCurrPos() - liftStartingPositionTicks);
            //if we are going rediculously slow (less than 5 ticks per second) and last update we were moving
            //or we were doing this for more than 3 seconds, calirate
            if((!liftMoving && liftTravelTicks > 40)
                    || currTimeMillis - liftCalibrationStartTime > 3000){//also timeout just in case

                hasCalibratedLiftPositionYet = true;//flag we have calibrated
                myLift.setCurrentPositionTicks(0);
//                myLift.setCurrentPositionTicks(0);//calibrate the lift
                myLift.setExtensionPowerRaw(0);
            }
            telemetry.addLine("lift travel ticks: " + liftTravelTicks);
        }
        telemetry.addLine("lift position!! : " + myLift.getExtensionCurrPos());
    }


    /**
     * Called every update
     */
    @Override
    public void MainStateMachine() {
        super.MainStateMachine();


        ////////////////////////////////////////////////////////////////////////////////////////////
        /**
         * Cube is our sample location
         */
        double cubeX = 269;
        double cubeY = 89;
        if(cubeLocation == 0){
            cubeX = 295;
            cubeY = 115;
        }
        if(cubeLocation == 2){
            cubeX = 242;
            cubeY = 63;
        }
        ////////////////////////////////////////////////////////////////////////////////////////////








        if(programStage == progStates.landing.ordinal()){
            if(stageFinished){
                initializeStateVariables();
            }

            //go half power for the first 30% of the range
            double power = 1.0;//hangMechanism.getCurrentPercent() < 0.3 ? 0.5 : 1.0;
            hangMechanism.setPower(power);

            if(hangMechanism.getCurrentPercent() > 0.99 || isTimedOut(5000)){
                nextStage();
                hangMechanism.setPower(0.0);//stop moving the hang mechanism
            }
        }

        /**
         * This will first turn to unlatch and unhook the mechanism
         */
        if(programStage == progStates.unlatching.ordinal()){
            if(stageFinished){
                MyPosition.setPosition(216.7,142,Math.toRadians(45));
                initializeStateVariables();
            }
            //turn
            MovementVars.movement_turn = -0.3 * SPEED_SCALE;
            //once we turn more than 45 degrees
            if(Math.abs(subtractAngles(blockStartingAngle_rad,getAngle_rad())) > Math.toRadians(60)
                    || isTimedOut(3000)){
                MovementVars.movement_turn = 0;//stop
                nextStage();
            }
        }


        /**
         * Turns towards the team marker (doesn't depend on the cube since we go over) the ball
         */
        if(programStage == progStates.turnTowardsMarker.ordinal()){
            if(stageFinished){
                initializeStateVariables();
            }
            movementResult r = MovementEssentials.pointAngle
                    (Math.toRadians(-45),0.4 * SPEED_SCALE,Math.toRadians(30));

            if(Math.abs(r.turnDelta_rad) < Math.toRadians(3)){
                nextStage();
                stopMovement();
            }
        }

        /**
         * This state drives to and deposits the team marker
         */
        if(programStage == progStates.extendingToMarker.ordinal()){
            if(stageFinished){
                initializeStateVariables();
            }

            double currPercentCollector = myCollector.getExtensionPercent();
            double decelerationPercent = 0.3;

            //where we will extend to
            double targetPercentCollector = 0.7;
            double speedScale = Range.clip((targetPercentCollector-
                    currPercentCollector)/decelerationPercent,0.2,1);


            gunToPosition(FIELD_LENGTH-115,115,
                    Math.toRadians(-45),0.3*speedScale,0.3*speedScale,
                    Math.toRadians(30),0.5,false);



            myCollector.setExtensionPowerRaw(0.4 * speedScale);
            if(myCollector.getExtensionPercent() > targetPercentCollector || isTimedOut(4000)){
                nextStage();
                myCollector.setExtensionPowerRaw(0);
            }
        }


        /**
         * This state deploys the team marker and waits
         */
        if(programStage == progStates.deployMarker.ordinal()){
            if(stageFinished){
                initializeStateVariables();
            }
            myCollector.activateDumper();

            int deployMarkerTime = 400;//the amount of time we will wait in this state

            if(currTimeMillis - stateStartTime > deployMarkerTime){
                nextStage();
            }
        }


        //calculate the distance to the ball
        double distanceToBallFromRobot = Math.hypot(cubeX-getXPos(),cubeY-getYPos());

        /**
         * Turns towards our cube and moves the collector to 15 cm in front of it
         */
        if(programStage == progStates.turningToOurCube.ordinal()){
            if(stageFinished){
                initializeStateVariables();
            }

            //point towards the cube
            double pointAngle = Math.atan2(cubeY - getYPos(),cubeX - getXPos());

            //actually point
            movementResult r = MovementEssentials.pointAngle(pointAngle,
                    0.2 * SPEED_SCALE, Math.toRadians(30));

            myCollector.setExtensionTargetLength(distanceToBallFromRobot + 15);


            //once we are turned towards the cube we can go to the next stage
            if(Math.abs(r.turnDelta_rad) < Math.toRadians(4)){
                nextStage();
            }
        }

        /**
         * Retracts the collector to get the cube
         */
        if(programStage == progStates.getOurCube.ordinal()){
            if(stageFinished){
                initializeStateVariables();
            }
            //back up kinda slowly i guess
            myCollector.setRunToPositionMode(RunToPositionModes.speedControl);
            myCollector.setExtensionPowerRaw(-0.4);


            //if we have gotten the cube we are done
            if(myCollector.getCurrExtensionDistFromCenter() < distanceToBallFromRobot - 10){
                nextStage();
                //stop the collector extension
                myCollector.setExtensionPowerRaw(0);
            }
        }


        /**
         * Drives towards the opponent's crater
         */
        if(programStage == progStates.drivingToDepot.ordinal()){
            if(stageFinished){
                initializeStateVariables();
            }


            //reset the collector extension now that the dumper has cleared
            myCollector.resetExtension(0.00);
            myCollector.setExtensionPowerRaw(0.4);



            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            //add our initial location, parameters don't matter since this only defines the shape
            //of the first line
            allPoints.add(new CurvePoint(blockStartingX,blockStartingY,
                    0,0,0,0,0));

            allPoints.add(new CurvePoint(FIELD_LENGTH-50,FIELD_LENGTH*0.4,
                    0.3*SPEED_SCALE,0.3*SPEED_SCALE,50,50,
                    Math.toRadians(60),0.6));

            allPoints.add(new CurvePoint(FIELD_LENGTH-30,FIELD_LENGTH*0.53,
                    0.3*SPEED_SCALE,0.3*SPEED_SCALE,50,50,
                    Math.toRadians(60),0.6));



            //follow curve will return true when done
            boolean done = followCurve(allPoints,Math.toRadians(90),false);

            if(done){
                stopMovement();
                nextStage();
            }
        }

        //this state goes for more
        if(programStage == progStates.goToCollectMore.ordinal()){
            if(stageFinished){
                initializeStateVariables();
                myAutoCollector.autoCollect();
                myCollector.turnOnRoller();
            }

            //if we are done we can go on to the next stage
            if(myAutoCollector.isDoneAutoCollect()){
                //if we have already done 4 trips go to end for now
                if(AutoFeeder.numAutoFeeds >= 4) {
                    nextStage(progStates.endDoNothing.ordinal());
                    myCollector.turnOffCollector();//turn off the collector
                    myLift.setExtensionPowerRaw(0);//turn off the lift extension
                    myCollector.setExtensionPowerRaw(0);//turn off the collector extension
                    stopMovement();//stop any movement that may be left over
                }else{
                    nextStage(progStates.feedCollectedTrip.ordinal());//just go right to feeding
                }
            }
        }




        //this state yeets the trip we already have
        if(programStage == progStates.feedCollectedTrip.ordinal()){
            if(stageFinished){
                initializeStateVariables();
                //only call auto feed if this is to get the first cube
                //because otherwise it will have already been called by the loading sensors
                if(AutoFeeder.numAutoFeeds == 0){
                    //yeah that's all it takes folks
                    myAutoFeeder.autoFeed(true);
                }
            }

            if(AutoFeeder.numAutoFeeds == 0){
                myAutoFeeder.earlyFinishDump();//early finish the first one
            }else{
                myAutoFeeder.setRegularDump();//regular everything else
            }
            if(myAutoFeeder.isDoneAutoFeed()){
                nextStage();
                stopMovement();
//                doneStraifingFeed = false;//reset this for the next time
            }
        }


        telemetry.addData("programStage: " , programStage);
    }


    /**
     * This will deploy the team marker
     */
    private boolean poopMarker() {

        //we will reverse once the collector is a bit extended
        boolean canReverseYet = myCollector.getExtensionPercent() > 0.05;
        if(canReverseYet){
            //activate the dumper
            myCollector.deployMarker();

            if(myCollector.getTiltPercent() < 0.4){
                myCollector.manualControl();
                myCollector.setRollerPower(0.15);//holds marker in
            }else{
                //reverse the collector to poop
                myCollector.manualControl();
                myCollector.setRollerPower(-0.25);
            }


            //record the depositMarkerTime if it hasn't been set yet (0)
            if(depositMarkerTime == 0){
                depositMarkerTime = currTimeMillis;//record the time we started reversing
            }
        }


        //if we have reversed for a while, we are done (this is so long because of the lift)
        return canReverseYet;// && currTimeMillis - depositMarkerTime > 750;
    }


    /** Handles controls for the ASCII field */
    public void controlVirtualFieldPosition(){
        if(gamepad1.a){zoomInVirtualField(); }
        if(gamepad1.y){zoomOutVirtualField(); }
    }
}