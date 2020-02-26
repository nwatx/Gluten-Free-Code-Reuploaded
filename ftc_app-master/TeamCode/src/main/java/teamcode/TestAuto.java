package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

import java.util.Random;

import Debugging.TimeProfiler;
import HelperClasses.Auto;
import RobotUtilities.MovementEssentials;

import static RobotUtilities.MovementVars.movement_y;

/**
 * Test Auto:
 * used to test movement functions
 */

@Autonomous(name = "TestAuto", group = "auto1")
public class TestAuto extends Auto {

    public enum progStates{

        allowingUserControl1,
        goingForwards,//goes fast forwards
    }

    @Override
    public void init(){
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

        //Display our location
        reportPositionData();
    }



    private TimeProfiler timeProfiler = new TimeProfiler(60);
    @Override
    public void MainStateMachine() {
        super.MainStateMachine();
        timeProfiler.markEnd();
        timeProfiler.markStart();
        telemetry.addLine("UPS: " + 1000.0/timeProfiler.getAverageTimePerUpdateMillis());

        if(programStage == progStates.allowingUserControl1.ordinal()){
            ControlMovement();
            telemetry.addLine("PRESS A TO CONTINUE");
            if(gamepad1.a){
                stopMovement();
                nextStage();
            }
        }



        if(programStage == progStates.goingForwards.ordinal()) {
            if (stageFinished) {
                initializeStateVariables();
            }
            double dist = Math.hypot(getXPos() - (blockStartingX + 50), getYPos() - blockStartingY);
            telemetry.addLine("dist: " + dist);

            double power = 0.7;
//            double power = Range.clip(dist / 7.0, 0, 1);
            telemetry.addLine("power: " + power);

            MovementEssentials.gunToPosition(blockStartingX + 50,blockStartingY,
                    Math.toRadians(90),power,0.7,Math.toRadians(30),
                    0.5,false);

            if(currTimeMillis - lastUpdateTime > 1000/10000){
                numUpdates ++;//increment numUpdates

                lastUpdateTime = currTimeMillis;
//                if(numUpdates%2 == 0){
//                    movement_y = 0.1;
//                }else{
//                    movement_y = -0.1;
//                }
            }

//
//            movement_y = (currTimeMillis -stateStartTime)/200000.0;
        }
    }
    private long numUpdates = 0;
    private long lastUpdateTime = 0;


}

