package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import HelperClasses.Auto;
import HelperClasses.ButtonPress;
import RobotUtilities.SpeedOmeter;

import static RobotUtilities.MovementVars.movement_turn;
import static RobotUtilities.MovementVars.movement_x;
import static RobotUtilities.MovementVars.movement_y;
import static RobotUtilities.MyPosition.AngleWrap;

/**
 * Measure Slip:
 * Takes measurments for how far the robot slips given different speeds.
 */

@Autonomous(name = "MeasureSlipDT", group = "auto1")
public class MeasureSlipDT extends Auto {

    //what power should we apply to the collector when accelerating it
    private static final double COLLECTOR_POWER = 1.0;
    //speed the robot will go before the slide
    private static double GOING_FORWARDS_SPEED = 0.3;
    private static double GOING_SIDEWAYS_SPEED = 0.3;
    private static double TURNING_SPEED = 0.4;


    //how long the robot will accelerate for before performing the slip
    private static final int ACCELERATION_TIME = 1000;
    //how long the robot will wait until taking the measurement
    private static final int SLIP_TIME = 1000;



    //how fast (meters per second) we were going at end of movement_y
    private double movement_speed_y = 0.0;
    private double movement_speed_x = 0.0;
    private double turn_speed = 0.0;
    //how fast the collector was moving cm/s just before curring the power
    private double collector_speed = 0.0;
    private double collector_length_before_slip = 0.0;//the length of the collector before cutting power
    private double collector_percent_before_slip = 0.0;//the percent length of the collector before cutting power



    public enum progStates{

        allowingUserControl1,
        goingForwards,//goes fast forwards
        measuringForwardsSlip,//stops, measuring the slip distance
        allowingUserControl2,//allows user to control robot to get ready
        goingSideways,//goes fast sideways
        measuringSidewaysSlip,//stops, measuring the slip distance
        allowingUserControl3,//allows user to control robot to get ready
        turning,//turns fast
        measuringTurningSlip,//stops, measures radians turned
        endProgram//does nothing, so values are still displayed
    }

    @Override
    public void init(){
        //don't worry about this
        setStartIn18Inches(false);
        super.init();
        //start
        programStage = progStates.allowingUserControl1.ordinal();

        //since we are measuring, start at 0,0,0
        setStartingPosition(0,0,Math.toRadians(0));
    }

    @Override
    public void init_loop(){
        super.init_loop();
//        DrawVirtualField();
    }

    @Override
    public void start(){
        super.start();
    }

    @Override
    public void loop(){
        super.loop();

        telemetry.addLine("Y power: " + GOING_FORWARDS_SPEED);
        telemetry.addLine("X power: " + GOING_SIDEWAYS_SPEED);
        telemetry.addLine("Turn power: " + TURNING_SPEED);


        if(ButtonPress.isGamepad1_dpad_up_pressed()){
            GOING_FORWARDS_SPEED += 0.02;
        }
        if(ButtonPress.isGamepad1_dpad_down_pressed()){
            GOING_FORWARDS_SPEED -= 0.02;
        }

        if(ButtonPress.isGamepad1_dpad_right_pressed()){
            GOING_SIDEWAYS_SPEED += 0.02;
        }
        if(ButtonPress.isGamepad1_dpad_left_pressed()){
            GOING_SIDEWAYS_SPEED -= 0.02;
        }
        if(ButtonPress.isGamepad1_y_pressed()){
            TURNING_SPEED += 0.02;
        }
        if(ButtonPress.isGamepad1_a_pressed()){
            TURNING_SPEED -= 0.02;
        }


        telemetry.addLine("y speed: " + movement_speed_y);
        telemetry.addLine("x speed: " + movement_speed_x);
        telemetry.addLine("turn speed: " + turn_speed);



        //Display our location
        reportPositionData();
    }



    @Override
    public void MainStateMachine() {
        super.MainStateMachine();
        //display the values that were calibrated
        telemetry.addData("COLLECTOR_SLIP_DISTANCE_PER_CM_PER_SECOND",myCollector.COLLECTOR_SLIP_DISTANCE_PER_CM_PER_SECOND);
        telemetry.addData("collector slip distance",myCollector.getCurrExtensionDistFromCenter()-collector_length_before_slip);
        telemetry.addData("ySlipDistanceFor1CMPS",SpeedOmeter.ySlipDistanceFor1CMPS);
        telemetry.addData("xSlipDistanceFor1CMPS",SpeedOmeter.xSlipDistanceFor1CMPS);
        telemetry.addData("turnSlipAmountFor1RPS",SpeedOmeter.turnSlipAmountFor1RPS);

        if(programStage == progStates.allowingUserControl1.ordinal() ||
                programStage == progStates.allowingUserControl2.ordinal() ||
                programStage == progStates.allowingUserControl3.ordinal()){
            ControlMovement();
            telemetry.addLine("PRESS A TO CONTINUE");
            if(ButtonPress.isGamepad1_a_pressed()){
                stopMovement();
                nextStage();
            }
        }



//        if(programStage == progStates.extendingCollectorExtension.ordinal()){
//            if(stageFinished){
//                initializeStateVariables();
//            }
//            myCollector.setExtensionPowerRaw(COLLECTOR_POWER);
//            if(myCollector.getExtensionPercent() > 0.6){
//                nextStage();
//            }
//        }
//
//        if(programStage == progStates.measuringCollectorExtensionSlip.ordinal()){
//            if(stageFinished){
//                collector_speed = myCollector.getExtensionCurrSpeed();
//                collector_length_before_slip = myCollector.getCurrExtensionDistFromCenter();
//                collector_percent_before_slip = myCollector.getTiltPercent();
//                initializeStateVariables();
//            }
//
//
//            //this will apply active braking to stop the collector if it's still going forwards
//            if(myCollector.getExtensionCurrSpeed() > 0){
//                myCollector.setExtensionPowerRaw(-1.0);
//            }else{
//                myCollector.setExtensionPowerRaw(0.0);
//            }
//
//            if(currTimeMillis-stateStartTime > SLIP_TIME){
//                myCollector.calibrateSlipDistance(myCollector.getCurrExtensionDistFromCenter() - collector_length_before_slip,collector_speed);
//                nextStage();
//            }
//
//        }


        if(programStage == progStates.goingForwards.ordinal()){
            if(stageFinished){
                initializeStateVariables();
            }
            movement_y = GOING_FORWARDS_SPEED;
            movement_x = 0.0;
            movement_turn = 0.0;
            if(currTimeMillis - stateStartTime > ACCELERATION_TIME){
                movement_speed_y = SpeedOmeter.getSpeedY();
                stopMovement();
                nextStage();
            }
        }
        if(programStage == progStates.measuringForwardsSlip.ordinal()){
            if(stageFinished){
                initializeStateVariables();
            }


            double distance = Math.sqrt(Math.pow(getXPos()-blockStartingX,2) + Math.pow(getYPos()-blockStartingY,2));


            if(currTimeMillis-stateStartTime > SLIP_TIME){
                SpeedOmeter.ySlipDistanceFor1CMPS = distance/movement_speed_y;
                stopMovement();
                nextStage();
            }
        }

        if(programStage == progStates.goingSideways.ordinal()){
            if(stageFinished){
                initializeStateVariables();
            }
            movement_y = 0.0;
            movement_x = GOING_SIDEWAYS_SPEED;
            movement_turn = 0.0;

            if(currTimeMillis - stateStartTime > ACCELERATION_TIME){
                movement_speed_x = SpeedOmeter.getSpeedX();
                stopMovement();
                nextStage();
            }
        }
        if(programStage == progStates.measuringSidewaysSlip.ordinal()){
            if(stageFinished){
                initializeStateVariables();
            }

            double distance = Math.sqrt(Math.pow(getXPos()-blockStartingX,2) + Math.pow(getYPos()-blockStartingY,2));

            if(currTimeMillis-stateStartTime > SLIP_TIME){
                SpeedOmeter.xSlipDistanceFor1CMPS = distance/movement_speed_x;
                nextStage();
            }
        }
        if(programStage == progStates.turning.ordinal()){
            if(stageFinished){
                initializeStateVariables();
            }
            movement_y = 0.0;
            movement_x = 0.0;
            movement_turn = TURNING_SPEED;

            if(currTimeMillis - stateStartTime > ACCELERATION_TIME){
                turn_speed = SpeedOmeter.getRadPerSecond();
                stopMovement();
                nextStage();
            }
        }
        if(programStage == progStates.measuringTurningSlip.ordinal()){
            if(stageFinished){
                initializeStateVariables();
            }

            double radsTurned = AngleWrap(getAngle_rad()-blockStartingAngle_rad);
            telemetry.addData("degreesTurned: ",Math.toDegrees(radsTurned));
            telemetry.addData("Turn speed deg:" ,Math.toDegrees(turn_speed));
            if(currTimeMillis-stateStartTime > SLIP_TIME){
                SpeedOmeter.turnSlipAmountFor1RPS = radsTurned/turn_speed;
                nextStage(0);
            }
        }
    }


}

