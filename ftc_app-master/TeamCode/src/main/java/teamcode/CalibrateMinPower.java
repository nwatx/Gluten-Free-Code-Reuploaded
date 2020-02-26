package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import HelperClasses.Auto;
import RobotUtilities.MovementEssentials;

import static RobotUtilities.MovementVars.movement_turn;
import static RobotUtilities.MovementVars.movement_x;
import static RobotUtilities.MovementVars.movement_y;
import static RobotUtilities.MyPosition.AngleWrap;
import static RobotUtilities.MyPosition.worldAngle_rad;


@Autonomous(name = "MinPowerCalibration", group = "auto1")
public class CalibrateMinPower extends Auto {


    public enum progStates{
        rampingForwards,
        measurnigForwardsSlip,
        rampingSideways,
        measuringSidewaysSlip,
        rampingTurning,
        measuringTurningSlip
    }

    @Override
    public void init(){
        //don't worry about that
        setStartIn18Inches(false);

        super.init();
        //start
        programStage = progStates.rampingForwards.ordinal();


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

        //Display our location
        reportPositionData();
    }



    @Override
    public void MainStateMachine() {

        super.MainStateMachine();


        telemetry.addData("MIN POWER Y: ", MovementEssentials.movement_y_min);
        telemetry.addData("MIN POWER X: ", MovementEssentials.movement_x_min);
        telemetry.addData("MIN POWER Turn: ", MovementEssentials.movement_turn_min);


        if(programStage == progStates.rampingForwards.ordinal()){
            if(stageFinished){
                initializeStateVariables();
            }
            //get the current time
            long elapsedMillis = currTimeMillis - stateStartTime;
            double elapsedSeconds = (double) elapsedMillis/1000.0;

            //it will take five seconds of measuring to ramp the power to the approximate minimum
            double approxSecondsToMin = 5.0;
            //use the current value as a good guess
            double estimatedMin = MovementEssentials.movement_y_min;
            movement_y = elapsedSeconds * estimatedMin / approxSecondsToMin;

            double distance = Math.sqrt(Math.pow(getXPos()-blockStartingX,2) + Math.pow(getYPos()-blockStartingY,2));
            if(distance >= 2){
                MovementEssentials.movement_y_min = movement_y;
                stopMovement();
                nextStage();
            }
        }
        if(programStage == progStates.measurnigForwardsSlip.ordinal()){
            if(stageFinished){
                initializeStateVariables();
            }
            if(currTimeMillis - stateStartTime > 1000){
                nextStage();
            }
        }



        if(programStage == progStates.rampingSideways.ordinal()){
            if(stageFinished){
                initializeStateVariables();
            }
            //get the current time
            long elapsedMillis = currTimeMillis - stateStartTime;
            double elapsedSeconds = (double) elapsedMillis/1000.0;

            //it will take five seconds of measuring to ramp the power to the approximate minimum
            double approxSecondsToMin = 5.0;
            //use the current value as a good guess
            double estimatedMin = MovementEssentials.movement_x_min;
            movement_x = elapsedSeconds * estimatedMin / approxSecondsToMin;

            double distance = Math.sqrt(Math.pow(getXPos()-blockStartingX,2) + Math.pow(getYPos()-blockStartingY,2));
            if(distance >= 2){
                MovementEssentials.movement_x_min = movement_x;
                stopMovement();
                nextStage();
            }
        }
        if(programStage == progStates.measuringSidewaysSlip.ordinal()){
            if(stageFinished){
                initializeStateVariables();
            }
            if(currTimeMillis - stateStartTime > 1000){
                nextStage();
            }
        }


        if(programStage == progStates.rampingTurning.ordinal()){
            if(stageFinished){
                initializeStateVariables();
            }
            //get the current time
            long elapsedMillis = currTimeMillis - stateStartTime;
            double elapsedSeconds = (double) elapsedMillis/1000.0;

            //it will take five seconds of measuring to ramp the power to the approximate minimum
            double approxSecondsToMin = 5.0;
            //use the current value as a good guess
            double estimatedMin = MovementEssentials.movement_turn_min;
            movement_turn = elapsedSeconds * estimatedMin / approxSecondsToMin;

            double angle = AngleWrap(worldAngle_rad - blockStartingAngle_rad);
            if(Math.abs(angle) >= Math.toRadians(3)){
                MovementEssentials.movement_turn_min = movement_turn;
                stopMovement();
                nextStage();
            }
        }
        if(programStage == progStates.measuringTurningSlip.ordinal()){
            if(stageFinished){
                initializeStateVariables();
            }
            if(currTimeMillis - stateStartTime > 1000){
                nextStage();
            }
        }

    }



}

