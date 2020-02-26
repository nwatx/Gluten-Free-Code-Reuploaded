package teamcode;

import android.os.SystemClock;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.Vision.FtcRobotControllerVisionActivity;

import java.util.ArrayList;

import Debugging.TimeProfiler;
import Globals.Globals;
import Hardware.AutoCollector;
import Hardware.AutoFeeder;
import HelperClasses.Auto;
import HelperClasses.ButtonPress;
import HelperClasses.CollectingLocation;
import HelperClasses.CurvePoint;
import RobotUtilities.InitAutoPointsHandler;
import RobotUtilities.MovementEssentials;
import RobotUtilities.MovementEssentials.movementResult;
import RobotUtilities.MovementVars;
import RobotUtilities.MyPosition;
import RobotUtilities.SpeedOmeter;

import static Hardware.Collector.RunToPositionModes;
import static RobotUtilities.MovementEssentials.followCurve;
import static RobotUtilities.MovementEssentials.gunToPosition;
import static RobotUtilities.MovementEssentials.pointAngle;
import static RobotUtilities.MovementVars.movement_turn;
import static RobotUtilities.MovementVars.movement_y;
import static RobotUtilities.MyPosition.subtractAngles;
import static RobotUtilities.MyPosition.worldAngle_rad;


@Autonomous(name = "auto1", group = "auto1")
public class Auto1 extends Auto {

    /**This enables double sample mode YAY*/
    private boolean doubleSample = false;

    //new epic
    private double SPEED_SCALE = 2.1;


    //todo: delete mo
    private int loggedYet = 0;




    //TODO:deleteme
    private boolean hasGoneToFeed = false;

    public enum progStates{
        landing,
        unlatching,
        drivingToMarker,//can go to backingUpForFirstCube, or eatingPartnersCube, or drivingBack
        backingUpForFirstCube,
        approachingPartnersCube,


        eatingPartnersCube,//this is just the part where it goes forwards actually eating the double sample


        drivingBack,
        turnToBall,
        backupBeforeBall,
        getOurBall,


        /**NOW THE REKING BEGINS-*/
        feedCollectedTrip,
        goToCollectMore,
        /**-----------------------*/

        inWayOfOpponent,


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

    //these change our position after driving back
    private static final double compensateXBackHome = -0.905 - compensateXDrivingToMarker;
    private static final double compensateYBackHome = -1.47 - compensateYDrivingToMarker;//2.605

    private boolean compensate1Yet = false;//flags when we compensate for the first 1
    private boolean compensate2Yet = false;//flags when we compensate for the second 1








    @Override
    public void init(){
        Globals.autoMode = Globals.AutoModes.auto1;

        initialCollectorDumperPosition = 0;
        super.init();
        //reset the hang mechanism encoder to 0
        hangMechanism.ResetEncoder();

        //We start the collector in an unusually far in position so calibrate to that
        myCollector.setCurrentPositionTicks(0);

        //set our starting position to the lander but this will be overridden when we get down
        setStartingPosition(137,137,Math.toRadians(-135));



        InitAutoPointsHandler.initializeAutoKeyPoints();




        //this will set the sample locations in approximate locations
        FtcRobotControllerVisionActivity.initializeSampleLocations();


        //enable vision because we will need it
        FtcRobotControllerVisionActivity.linkToInstance.enableView();

    }



    //this is false when we are entering the collecting points and true when we are
    //moving the sample location in init of auto
//    private boolean movingSamples = false;




    //after this many cycles we will get in the opponents way
    private int tripsUntilGetInOpponentsWay = 10;


    @Override
    public void init_loop(){
        super.init_loop();
        telemetry.addLine("cubeLocation: " + FtcRobotControllerVisionActivity.getCubeLocation());



        telemetry.addLine("\nTRIPS UNTIL GET IN WAY: " + tripsUntilGetInOpponentsWay + "\n");

        if(ButtonPress.isGamepad1_y_pressed()){
            tripsUntilGetInOpponentsWay ++;
        }

        if(ButtonPress.isGamepad1_a_pressed()){
            tripsUntilGetInOpponentsWay --;
        }



//        telemetry.addLine("DOUBLE SAMPLE? " + doubleSample);
        //switch the double sample if you press gamepad1.a
//        if(ButtonPress.isGamepad1_a_pressed()){
//            doubleSample = !doubleSample;
//        }




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




    public static int cubeLocation = 0;

    @Override
    public void start(){
        //first save the cubeLocation DON'T MOVE AFTER SUPER.START()!!!
        cubeLocation = FtcRobotControllerVisionActivity.getCubeLocation();

        super.start();//call the super method
        myAutoFeeder.setUseCubeLocationInAutoFeed(true);


        boolean homeCrater = true;//we are doing our local crater

        /*
        These are the percentages along the crater (bottom right to top left)
        that we will approach from
         */
        double middleCubePercent = 0.45;
        double leftCubePercent = 0.45 - 0.23;
        double rightCubePercent = 0.45 + 0.21;

        /**
         * Add a bunch of collecting locations with only specifying where the robot needs to be
         */
        for(int i = 0; i < 60; i ++){
            double collectingLocation = middleCubePercent;
            if(cubeLocation == 0){ collectingLocation = leftCubePercent; }
            if(cubeLocation == 2){ collectingLocation = rightCubePercent; }

            myAutoCollector.addCollectingLocation(collectingLocation,homeCrater);
            myAutoCollector.addCollectingLocation(collectingLocation,homeCrater);
            myAutoCollector.addCollectingLocation(collectingLocation,homeCrater);
        }

        if(cubeLocation == 2){
            for(int i = 0; i < 60-2; i += 3){
                myAutoCollector.allCollectingLocations.get(i).pointLocationX = -30;

                myAutoCollector.allCollectingLocations.get(i+1).pointLocationX = 10;
                //for this one we will move more to the right to avoid hitting the ball
                myAutoCollector.allCollectingLocations.get(i+1)
                        .setPercentApproach(0.45+0.23,homeCrater);

                myAutoCollector.allCollectingLocations.get(i+2).pointLocationX = 30;
                myAutoCollector.allCollectingLocations.get(i+2)
                        .setPercentApproach(0.45+0.23,homeCrater);
            }
        }

        if(cubeLocation == 1){
            for(int i = 0; i < 60-2; i += 3){
                //we will leave the first one at 0,0
                myAutoCollector.allCollectingLocations.get(i).pointLocationY = 0;

                //extend less on the first trip
                myAutoCollector.allCollectingLocations.get(i+1).pointLocationY = 55;

                myAutoCollector.allCollectingLocations.get(i+2).pointLocationX = 55;

            }
        }

        if(cubeLocation == 0){
            for(int i = 0; i < 60-2; i += 4){
                //extend less on the first trip
                myAutoCollector.allCollectingLocations.get(i).pointLocationY = -35;

                myAutoCollector.allCollectingLocations.get(i+1).pointLocationY = 0;

                myAutoCollector.allCollectingLocations.get(i+2).pointLocationY = 40;

                //this is a last attempt to get cubes on the remaining side
                myAutoCollector.allCollectingLocations.get(i+3).pointLocationX = 60;
                myAutoCollector.allCollectingLocations.get(i+3).pointLocationY = -5;
            }
        }





        /**
         * Use this to use debugging mode
         */
        ///////////////////////////////////////////////////
//        startDebugging();
        //////////////////////////////////////////////////

//        nextStage(progStates.drivingBack.ordinal());//GOING HOME
//        nextStage(progStates.unlatching.ordinal());//just start from here for debugging

        //testing auto collect
//        nextStage(progStates.goToCollectMore.ordinal());
//        AutoFeeder.numAutoFeeds = 1;
//        MyPosition.setPosition(147.67, 140.573, Math.toRadians(-45-0.55));

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


        telemetry.addLine("\nCollector raw encoder: " + myCollector.getExtensionPosRAW());
        telemetry.addLine("Collector offset: " + myCollector.masterOffsetTicks);

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



        //if it is time to get in our opponent's way, go do that.
        getInOpponentsWayIfNecessary();


        telemetry.addLine("average time profilier 0: " + allTimeProfilers.get(0).getAverageTimePerUpdateMillis());
        telemetry.addLine("average time profilier 1: " + allTimeProfilers.get(1).getAverageTimePerUpdateMillis());
        telemetry.addLine("average time profilier 2: " + allTimeProfilers.get(2).getAverageTimePerUpdateMillis());
        telemetry.addLine("average time profilier 3: " + allTimeProfilers.get(3).getAverageTimePerUpdateMillis());
    }


    /**
     * Moves to get in our opponent's way after a certain number of trips
     */
    private void getInOpponentsWayIfNecessary() {
        if(cubeLocation == 2 && AutoFeeder.numAutoFeeds >= tripsUntilGetInOpponentsWay){
            nextStage(progStates.inWayOfOpponent.ordinal());
        }
    }

    /**
     * This extends into the crater if it the end of auto
     */
    private void ExtendIntoCraterWhenReady() {
        //get the elapsed time since the start of the program
        long elapsedSinceStart = currTimeMillis-programStartTime;//time since start of program
        long timeTillEnd = 30000-elapsedSinceStart;//get the time until the end of the program




        //this is how much time we want to allow to abort auto feed
        //if our lift is up, we need a lot of time so that we don't dump
        //otherwise don't worry
        long retractAndAbortTime = myLift.getExtensionPercent() > 0.3 ? 800 : 450;


        //time to allow the collector to extend into the crater
        //if the collector is already extended, we don't need any parking time
        //else we better save some time
        long parkingTime = myCollector.getExtensionPercent() > 0.5 ? 0 : 400;





        //extend into the crater with 1 second left
        if(timeTillEnd < retractAndAbortTime){
            startedParking = true;
            //if it is not time to park yet
            if(timeTillEnd > parkingTime){
                //this will go from 1 to 0 and we will decelerate using it
                double decelerationPercent = Range.clip(timeTillEnd-parkingTime/
                        ((double) retractAndAbortTime-parkingTime),0,1);

                //multiply the lift power by this constant
                myLift.setExtensionPowerRaw(myLift.getExtensionMotorPower() * decelerationPercent);

                //stop the movement
                stopMovement();
                nextStage(progStates.endDoNothing.ordinal());//stop doing stuff
            }else{
                myAutoFeeder.abortAutoFeed();
                myCollector.setExtensionTargetPercent(0.65);//extend the collector into the pit
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
            //make sure the lift is un-dumped
            myLift.unDump();
            //make sure the release servo is out of the way
            myLift.releaseMinerals();

            //for this we will use power control
            myLift.setRunToPositionMode(RunToPositionModes.powerControl);
            //now slowly move the lift down
            myLift.setExtensionPowerRaw(-0.52);


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
         * Cube is the partner's sample, ball is our sample
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


        double ballX = 90;
        double ballY = 90;
        if(cubeLocation == 0){
            ballX = 116;
            ballY = 63;
        }
        if(cubeLocation == 1){
            ballX = 90;
            ballY = 90;
        }
        if(cubeLocation == 2){
            ballX = 63;
            ballY = 117;
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
                MyPosition.setPosition(147.67, 140.573,
                        Math.toRadians(-45-0.55));//( - 0.0525)
                initializeStateVariables();
            }
            //turn
            MovementVars.movement_turn = -0.3 * SPEED_SCALE;
            //once we turn more than 45 degrees
            if(Math.abs(subtractAngles(blockStartingAngle_rad,getAngle_rad())) > Math.toRadians(40)
                    || isTimedOut(3000)){
                MovementVars.movement_turn = 0;//stop
                nextStage();
            }
        }

        /**
         * This state drives to and deposits the team marker
         */
        if(programStage == progStates.drivingToMarker.ordinal()){
            if(stageFinished){
                initializeStateVariables();
            }

            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            //add our initial location, parameters don't matter since this only defines the shape
            //of the first line
            allPoints.add(new CurvePoint(blockStartingX,blockStartingY,
                    0,0,0,0,0));

            allPoints.add(new CurvePoint(140,110,
                    0.5*SPEED_SCALE,0.5*SPEED_SCALE,50,50,
                    Math.toRadians(60),0.6));

            allPoints.add(new CurvePoint(155,65,
                    0.4*SPEED_SCALE, 0.5*SPEED_SCALE,50,65,
                    Math.toRadians(60),0.6));



            //get the current x location of the collector on the field
            double currentCollectorX = myCollector.getExtensionPosition().x;

            //where the collector will be when we stop and deploy the marker
            double deployXCollector = 310;



            //slow down once we get towards the end
            double scaleDownLastMove = (1.0* Range.clip((deployXCollector-currentCollectorX)
                    /100.0,0.05,1.0));


            allPoints.add(new CurvePoint(190, 30.5,
                    0.5*SPEED_SCALE * scaleDownLastMove,0.5*SPEED_SCALE,50,65,
                    Math.toRadians(60),0.6));



            allPoints.add(new CurvePoint(285, 30.5,
                    0.4 * scaleDownLastMove*SPEED_SCALE,0.6*SPEED_SCALE,50,65,
                    Math.toRadians(60),0.6));



            //follow curve will return true when done
            followCurve(allPoints,Math.toRadians(90),false);




            /** NOW WE WILL COMPENSATE FOR TELEMETRY ERROR ONCE WE CROSS 250*/
            if(getXPos() > 250 && !compensate1Yet){
                //the compensation constants are the top of the program
                MyPosition.setPosition(getXPos() + compensateXDrivingToMarker,
                        getYPos() + compensateYDrivingToMarker, getAngle_rad());
                compensate1Yet = true;
            }


            //NOW TO DO THE MARKER
            boolean doneMarker = false;
            //since we have the wall rider we can extend really early
            if(Math.abs(subtractAngles(0,worldAngle_rad)) < Math.toRadians(32) &&
                    myCollector.getExtensionPosition().x > 210){

                //the max amount we are allowed to extend the collector
                double maxExtension = 0.4;

                //min percent of the slow down (lower means more deceleration)
                double minSlowDown = 0.1;
                //slow down the more you extend
                double slowDownAmount = 1.0
                        - Range.clip(myCollector.getExtensionPercent()
                        /maxExtension,0,1.0-minSlowDown);

                if(myCollector.getExtensionPercent() < maxExtension){
                    myCollector.setExtensionPowerRaw(0.55 * scaleDownLastMove);
                }else{
                    myCollector.setExtensionPowerRaw(0);
                }
                doneMarker = poopMarker();
            }

            //If the collector x position is past deployXCollector, we are done moving and can stop the
            //collector
            boolean doneMovement = currentCollectorX > deployXCollector;
            //stop moving if the collector is at the target drop zone
            if(doneMovement){
                stopMovement();
                //stop the collector if we weren't moving
                myCollector.setExtensionPowerRaw(0);
            }






            //TODO: fix the marker deployment exit condition
            //if we have done the marker and are done moving, advance
            if(doneMovement || isTimedOut(8000)){
                //if we are double sampling go to the next stage
                if(doubleSample){
                    nextStage();
                }else{
                    //IF WE AREN'T DOUBLE SAMPLING: drive back, skip the next few states
                    nextStage(progStates.drivingBack.ordinal());
                }


                //now we can go back to 0
                myCollector.setExtensionTargetPercent(0.0);
                myCollector.setExtensionPowerRaw(0.5);//go a bit slower
            }
        }


        //TODO: turn off roller after double sample, turn back on when down and ready to eat

        //if we are going to double sample, first extend this many centimeters before the ball
        double amountUndershootBall = 7;

        /**
         * This state backs up before getting the cube
         */
        if(programStage == progStates.backingUpForFirstCube.ordinal()){
            if(stageFinished){
                initializeStateVariables();
                myCollector.maintainCollectorCurrentRadius();//record the current radius to maintain
            }

            //YES I'M USING FOLLOW CURVE
            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            //add our initial location, parameters don't matter since this only defines the shape
            //of the first line
            allPoints.add(new CurvePoint(blockStartingX,blockStartingY,
                    0,0,0,0,0));

            //first add a point close to the wall to avoid the silver mineral
            allPoints.add(new CurvePoint(230,31,
                    0.5*SPEED_SCALE,0.4*SPEED_SCALE,20,20,
                    Math.toRadians(60),0.5));


            //we should back up more if it's a close cube
            double x = cubeLocation == 2 ? 190 : 210;
            allPoints.add(new CurvePoint(x,40,
                    0.5*SPEED_SCALE,0.4*SPEED_SCALE,20,20,
                    Math.toRadians(60),0.0));


            //drive to back up
            if(followCurve(allPoints,Math.toRadians(-90),false)){
                gotToTargetPlaceDrivingBack = true;
            }



            //calculate the distance to the ball
            double distanceToBall = Math.hypot(cubeX-getXPos(),cubeY-getYPos())-amountUndershootBall;//undershoot a bit
            //make sure notodesn to go beyond the collector's range by clipping it to it's limits
            double min = myCollector.OFFSET_FROM_CENTER;
            double max = myCollector.OFFSET_FROM_CENTER + myCollector.LENGTH_EXTENDED;
            //clip this so that it don't't go over the limits
            double currentTargetPosition =  Range.clip(distanceToBall,min,max);


            boolean isExtendingCollector = false;
            //wait until we are done driving
            if(gotToTargetPlaceDrivingBack){
                movementResult r = pointAngle(0,0.3,Math.toRadians(30));
                if(Math.abs(r.turnDelta_rad) < Math.toRadians(3)){
                    //set the target distance of the collector extension
                    myCollector.setExtensionTargetLength(currentTargetPosition);
                    //go a bit slower to the position
                    myCollector.setExtensionPowerRaw(0.6);
                    isExtendingCollector = true;
                }
            }


            if(!isExtendingCollector){
                //if we are not extended the distance to ball, maintain current location
                if(myCollector.getCurrExtensionDistFromCenter() < distanceToBall){
                    myCollector.updateMaintainCurrentLocation();//maintain the current collector location (override if necessary)
                }else{
                    myCollector.setExtensionPowerRaw(0);//otherwise stop
                }
            }


            //next stage if we are extended (within 5 cm) and done driving
            double delta = myCollector.getCurrExtensionDistFromCenter() - currentTargetPosition;
            if(gotToTargetPlaceDrivingBack && Math.abs(delta) < 5){
                nextStage();
                stopMovement();
            }
        }






        //This state turns towards the cube and extends the collector accordingly
        if(programStage == progStates.approachingPartnersCube.ordinal()){
            if(stageFinished){
                initializeStateVariables();
                myCollector.turnOnRoller();
            }

            //calculate the distance to the ball
            double distanceToApproachBall = Math.hypot(cubeX-getXPos(),cubeY-getYPos())
                    -amountUndershootBall;//make sure we are before the ball

            //make sure not to go beyond the collector's range by clipping it to it's limits
            double min = myCollector.OFFSET_FROM_CENTER;
            double max = myCollector.OFFSET_FROM_CENTER + myCollector.LENGTH_EXTENDED;
            distanceToApproachBall = Range.clip(distanceToApproachBall,min,max);




            //Now that we're safe, set the target distance of the collector extension
            myCollector.setExtensionTargetLength(distanceToApproachBall);
            //go a bit slower to the position
            myCollector.setExtensionPowerRaw(0.45);

            //calculate the angle to point towards our target ball
            double pointAngle = Math.atan2(cubeY - getYPos(),cubeX - getXPos());
            //point towards the ball
            movementResult r = pointAngle(pointAngle,0.40*SPEED_SCALE,
                    Math.toRadians(50));






            //If we are not turning, like stalled, up the power
            if(Math.abs(SpeedOmeter.getDegPerSecond()) < 5 && Math.abs(r.turnDelta_rad) > Math.toRadians(5)){
                movement_turn = MovementEssentials.minPower(movement_turn,0.2);
            }



            //now take delta from our extension distance to the distance to the ball
            double distanceToExtend = distanceToApproachBall- myCollector.getCurrExtensionDistFromCenter();





            //if we are close to the target distance and angle, move on
            if(Math.abs(distanceToExtend) < 2.0 && Math.abs(r.turnDelta_rad) < Math.toRadians(3) &&
                    Math.abs(SpeedOmeter.getDegPerSecond()) < 10){
                nextStage();
                stopMovement();
            }
        }


        /**
         * This eats our partner's cube
         */
        if(programStage == progStates.eatingPartnersCube.ordinal()){
            if(stageFinished){
                initializeStateVariables();
                //activate collector and dumper
                myCollector.turnOnRoller();
                myCollector.activateDumper();
            }


            //calculate the distance to the cube
            double rawDistanceToCube = Math.hypot(cubeX-getXPos(),cubeY-getYPos());
            //the start distance is what we will extend to at the beginning of the state
            double startDistance = rawDistanceToCube - amountUndershootBall;
            //the final distance is at the end what distance we will extend to (distance + 13cm)
            double finalDistance = rawDistanceToCube + 13;
            //calculate the distance to the ball, but add some because we want to extend a bit past it
            double currTargetDistance = ((currTimeMillis - stateStartTime)/750.0) *  (finalDistance - startDistance) + startDistance;


            //set to the current target distance
            myCollector.setExtensionTargetLength(currTargetDistance);
            //go with 70% power
            myCollector.setExtensionPowerRaw(0.7);

            //if we are above 97% allow driving to get that last bit of distance
            if(myCollector.getExtensionPercent() > 0.97){
                movement_y = 0.1 * SPEED_SCALE;//move slowly forwards
            }


            //get the delta distance for comparisons and stuff
            double distanceToExtend = finalDistance - myCollector.getCurrExtensionDistFromCenter();



            //Still point towards the ball

            //calculate the angle to point towards our target ball
            double pointAngle = Math.atan2(cubeY - getYPos(),cubeX - getXPos());
            //point towards the ball
            pointAngle(pointAngle,0.45 * SPEED_SCALE,Math.toRadians(50));



            //move on once we have eaten
            if(distanceToExtend < 5 || isTimedOut(3000)){
                nextStage();
                stopMovement();
            }
        }


        /**
         * This state drives back from the crater and ends near the target cube
         */
        if(programStage == progStates.drivingBack.ordinal()){
            if(stageFinished){
//                MyPosition.setPosition(300,30,0);//GOING HOME
                initializeStateVariables();
//                myLift.resetEncoder();//GOING HOME
//                myCollector.setCurrentPositionTicks(210);//GOING HOME


                /** COMPENSATE FOR TELEMETRY ERROR */
                if(!compensate2Yet){
                    MyPosition.setPosition(getXPos() + compensateXBackHome,
                            getYPos() + compensateYBackHome, getAngle_rad());
                    compensate2Yet = true;
                }
            }


            boolean okToMove = false;
            //first retract the dumper, then retract the collector, then move
            if(myCollector.getTiltPercent() < 0.35){
                //reset the collector extension now that the dumper has cleared
                myCollector.resetExtension(0.00);
                myCollector.setExtensionPowerRaw(0.4);


                //once we have retracted, reverse the collector
                if(myCollector.getExtensionPercent() < 0.02){
                    okToMove = true;
                    myCollector.retractDumper();//no more auto override
                }
            }


            myCollector.autoAvoidBalls();//we want to avoid hitting the white ball

            //turn off the collector if we have tilted to avoid jamming
            if(myCollector.getTiltPercent() < 0 + 0.1){
                myCollector.turnOffCollector();
            }







            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            //add our initial location, parameters don't matter since this only defines the shape
            //of the first line
            allPoints.add(new CurvePoint(blockStartingX,blockStartingY,
                    0,0,0,0,0));


            allPoints.add(new CurvePoint(210, 31,
                    0.45*SPEED_SCALE, 0.45*SPEED_SCALE,50,65,
                    Math.toRadians(60),0.4));


            allPoints.add(new CurvePoint(170,50,
                    0.45*SPEED_SCALE,0.45*SPEED_SCALE,50,65,
                    Math.toRadians(60),0.6));


            double x = 115;
            double y = 135;
            if(cubeLocation == 0){
                x = 141 + 7;
                y = 111 - 7;
            }
            if(cubeLocation == 1){
                x = 128 + 6;
                y = 128 - 6;
            }
            if(cubeLocation == 2){
                x = 124+3;
                y = 133;
            }

            //go slower for the approach of the left cube
            double moveSpeed = cubeLocation == 0 ? 0.4 : 0.45;
            double turnSpeed = moveSpeed;
            allPoints.add(new CurvePoint(x, y,moveSpeed*SPEED_SCALE,
                    turnSpeed*SPEED_SCALE,
                    50,65, Math.toRadians(60),0.0));




            /*
                This will move a little bit back before we start following the curve
             */
            boolean done = false;
            if(okToMove){
                //these we will to backwards on
                done = followCurve(allPoints,Math.toRadians(-90),false);
            }else{
                //if we haven't gone 10 cm yet, make sure we are just backing up
                if(Math.hypot(getXPos()-blockStartingX,getYPos()-blockStartingY) < 10){
                    stopMovement();
                    movement_y = -0.1 * SPEED_SCALE;
                }
                stopMovement();
            }


            //if we are done go to next stage
            if(done){
                stopMovement();
                nextStage();
            }
        }

        /**
         * This state turns towards the cube
         */
        if(programStage == progStates.turnToBall.ordinal()){
            if(stageFinished){
                initializeStateVariables();
                myCollector.activateDumper();
                myCollector.turnOnRoller();
            }

            movementResult r = pointAngle(Math.atan2(ballY-getYPos(), ballX-getXPos()),
                    0.35 * SPEED_SCALE,Math.toRadians(30));



            if(Math.abs(r.turnDelta_rad) < Math.toRadians(10)){
                int stage = progStates.getOurBall.ordinal();//cubeLocation == 0 ? progStates.backupBeforeBall.ordinal() :
                        //progStates.getOurBall.ordinal();
                nextStage(stage);
                stopMovement();
            }
        }

        /**
         * Backs up before we deploy the collector (only used for cubeLocation == 0)
         * not even used
         */
        if(programStage == progStates.backupBeforeBall.ordinal()){
            if(stageFinished){
                initializeStateVariables();
            }

            //continue pointing towards the ball
            pointAngle(Math.atan2(ballY-getYPos(), ballX-getXPos()),
                    0.35 * SPEED_SCALE,Math.toRadians(60));

            movement_y = -0.15 * SPEED_SCALE;
            if(Math.hypot(getXPos()-blockStartingX,getYPos()-blockStartingY) > 7){
                nextStage();
                stopMovement();
            }
        }






        /**
         * This actually gets our ball (moving forwards)
         */
        if(programStage == progStates.getOurBall.ordinal()){
            if(stageFinished){
                initializeStateVariables();
            }

            double angleToBall = Math.atan2(ballY-getYPos(), ballX-getXPos());

            //this is the distance away the center of the robot is to the ball
            double approachDist = 33;
            //get where the robot should be when we get the ball
            double approachX = (-Math.cos(angleToBall) * approachDist)+ballX;
            double approachY = (-Math.sin(angleToBall) * approachDist)+ballY;


            //point towards the ball and save what power the algorithm wanted
            movementResult r = pointAngle(angleToBall,
                    0.35 * SPEED_SCALE,Math.toRadians(35));
            //save this because it will be overridden
            double movementTurnSaved = MovementVars.movement_turn;

            //calculate the movement speed inversely proportional to the amount we need to turn
            double speed = 0.3 * Range.clip(1.0-(Math.abs(r.turnDelta_rad)/
                    Math.toRadians(20)),0,1);

            //wait for the collector to deploy before driving
            if(myCollector.getTiltPercent() < 0.9){
                speed = 0;
            }
            //go to the approach location
            gunToPosition(approachX,approachY,Math.toRadians(90),speed * SPEED_SCALE,
                    0,0.01,0,false);

            //re-apply the saved turn
            MovementVars.movement_turn = movementTurnSaved;


            //put the dumper down when we are at the target angle
            if(Math.abs(r.turnDelta_rad) < Math.toRadians(6)){
                myCollector.activateDumper();
                if(activateDumperOurBallTime == 0){
                    activateDumperOurBallTime = currTimeMillis;
                }
                if(currTimeMillis - activateDumperOurBallTime > 400){
                    myCollector.turnOnRoller();
                }
            }


            if(Math.hypot(getXPos()-approachX,getYPos()-approachY) < 5){
                stopMovement();
                nextStage();
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

            //once we're done feeding we need to move on with the state
            if(myAutoFeeder.isDoneAutoFeed()){
                nextStage();
                stopMovement();
            }
        }
        //this state goes for more
        if(programStage == progStates.goToCollectMore.ordinal()){
            if(stageFinished){
                initializeStateVariables();
                myAutoCollector.autoCollect();
                myCollector.turnOnRoller();
            }
            int maxTrips = 5;
            if(AutoFeeder.numAutoFeeds >= maxTrips) {
                myLoadingSensors.disable();
            }


            telemetry.addLine("\n\nin go to collect more");

            //if we are done we can go on to the next stage
            if(myAutoCollector.isDoneAutoCollect()){
                telemetry.addLine("ABORTING");
                telemetry.addLine("num auto feeds: " + AutoFeeder.numAutoFeeds);
                telemetry.addLine("autoCollectState: " + AutoCollector.myState);
                telemetry.addLine("autoFeederState: " + AutoFeeder.myState);
                telemetry.addLine("has gone to feed: " + hasGoneToFeed);




                //if we have already done 1 trips go to end for now
                if(AutoFeeder.numAutoFeeds >= maxTrips) {
                    myAutoFeeder.abortAutoFeed();
                    myAutoCollector.abortAutoCollect();
                    hasGoneToFeed = true;
                    myCollector.turnOffCollector();//turn off the collector
                    myLift.setExtensionPowerRaw(0);//turn off the lift extension
                    myCollector.setExtensionPowerRaw(0);//turn off the collector extension
                    stopMovement();//stop any movement that may be left over
                }else{
                    hasGoneToFeed = true;
                    nextStage(progStates.feedCollectedTrip.ordinal());//just go right to feeding
                }
            }
            telemetry.addLine("\n");
        }


        /**
         * This state stops everything we are doing and parks in the way of our opponent
         */
        if(programStage == progStates.inWayOfOpponent.ordinal()){
            if(stageFinished){
                initializeStateVariables();
                //first stop everything
                myAutoCollector.abortAutoCollect();
                initAutoCollect();
            }

            if(myLift.getExtensionPercent() < 0.2){
                //we are going to use the drive to auto collect method to park at the right end
                //of the crater to prevent the opponent from parking
                driveToAutoCollect(new CollectingLocation(0.92,true),
                        20, 20);
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


    /**
     * This is called when auto is done (because of the timer)
     */
    public void stop(){
        //save the current tilt of the collector
        initialCollectorDumperPosition = myCollector.getTiltPercent();
    }
}