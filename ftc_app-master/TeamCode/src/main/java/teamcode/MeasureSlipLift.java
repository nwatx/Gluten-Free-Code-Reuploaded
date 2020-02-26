package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import HelperClasses.Auto;
import HelperClasses.ButtonPress;


/**
 * This opmode accelerates the lift to high powers and stops, measuring the time it takes to stop
 */

@Autonomous(name = "MeasureSlipLift", group = "auto1")
public class MeasureSlipLift extends Auto {

    //what power should we apply to the lift when accelerating it
    private static double LIFT_POWER = 0.4;


    //how long the lift will accelerate for before performing the slip
    private static final int ACCELERATION_TIME = 500;

    //how long the robot will wait until taking the measurement
    private static final int SLIP_TIME = 1000;


    //how fast (meters per second) we were going at end of movement_y
    private double liftSpeedBeforeStop = 0.0;

    private double liftPercentBeforeStop = 0.0;//the length of the collector before cutting power
    private double liftPercentAfterStop = 0.0;//the percent length of the collector before cutting power


    public enum progStates {

        allowingUserControl1,
        movingLift,//goes fast forwards
        measuringSlip,//stops, measuring the slip distance
    }

    @Override
    public void init() {
        setStartIn18Inches(false);

        super.init();
        //start
        programStage = progStates.allowingUserControl1.ordinal();

        //since we are measuring, start at 0,0,0
        setStartingPosition(0, 0, Math.toRadians(0));
    }

    @Override
    public void init_loop() {
        super.init_loop();
//        DrawVirtualField();
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        super.loop();

        //display the power
        telemetry.addLine("Power: " + LIFT_POWER);


        if (ButtonPress.isGamepad1_dpad_up_pressed()) {
            LIFT_POWER += 0.02;
        }
        if (ButtonPress.isGamepad1_dpad_down_pressed()) {
            LIFT_POWER -= 0.02;
        }


        //display the speed before the stop
        telemetry.addLine("lift speed: " + liftSpeedBeforeStop);


        //Display our location
        reportPositionData();
    }


    @Override
    public void MainStateMachine() {
        super.MainStateMachine();
        //display the values that were calibrated
        telemetry.addData("Lift slip time", myLift.SLIP_PERCENT_PER_SPEED_PERCENT);


        if (programStage == progStates.allowingUserControl1.ordinal()) {
            ControlMovement();
            telemetry.addLine("PRESS A TO CONTINUE AND CALIBRATE LIFT");
            if (ButtonPress.isGamepad1_a_pressed()) {
                myLift.setCurrentPositionTicks(0);
                stopMovement();
                nextStage();
            }
        }


        //moves up the lift before measuring speed
        if(programStage == progStates.movingLift.ordinal()){
            if(stageFinished){
                initializeStateVariables();
            }
            myLift.setExtensionPowerRaw(LIFT_POWER);
            if(myLift.getExtensionPercent() > 0.45){
                nextStage();
            }
        }

        //waits for lift to stop
        if(programStage == progStates.measuringSlip.ordinal()){
            if(stageFinished){
                liftSpeedBeforeStop = myLift.getExtensionCurrentSpeedPercent();
                liftPercentBeforeStop = myLift.getExtensionPercent();
                initializeStateVariables();
            }

            //stop
            myLift.setExtensionPowerRaw(0);

            if(currTimeMillis-stateStartTime > SLIP_TIME ||
                    myLift.getExtensionCurrentSpeedPercent() < 0.05){
                liftPercentAfterStop = myLift.getExtensionPercent();
                //calculate distance traveled
                double distTraveled = liftPercentAfterStop-liftPercentBeforeStop;


                //this look at the units, they cancel to give us time it took to stop
                //well not really, since the speed isn't constant over the deceleration
                myLift.SLIP_PERCENT_PER_SPEED_PERCENT = distTraveled/liftSpeedBeforeStop;
                nextStage();
            }

        }
    }
}
