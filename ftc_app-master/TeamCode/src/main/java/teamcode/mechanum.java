package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcontroller.Vision.FtcRobotControllerVisionActivity;

import Debugging.TimeProfiler;
import Hardware.AutoFeeder;
import Hardware.Collector;
import HelperClasses.ButtonPress;
import HelperClasses.Robot;
import RobotUtilities.MovementVars;
import RobotUtilities.MyPosition;

/**
 * This is the main OpMode for teleop control
 */
@TeleOp(name = "TeleopMain", group = "TeleopMain")
public class mechanum extends Robot {

    //If we are on the alternate button layout for gamepad2,
    //so that there are more buttons
    private static boolean gamepad2AlternateControls = false;


    @Override
    public void init() {
        setStartIn18Inches(false);
        FtcRobotControllerVisionActivity.linkToInstance.disableView();

        super.init();
        //we start the collector in the down position
//        myCollector.activateDumper();
        //update so we don't twitch
//        myCollector.update();
//        myCollector.setCollectorTilt(1.0);

//        FtcRobotControllerVisionActivity.killOpenCV();
//        dumperServo.setPosition(getDouble("DUMPER_DISENGAGED_POS"));

    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
        myAutoFeeder.setUseCubeLocationInAutoFeed(false);
        Auto1.programStage = Auto1.progStates.endDoNothing.ordinal();//flags we aren't in auto

//        MyPosition.setPosition(137,137,Math.toRadians(-135));
//        MyPosition.setPosition(0,0,0);
        zoomVirtualField(2.5);
    }





    private TimeProfiler timeProfiler = new TimeProfiler(1000);
    private TimeProfiler tp1 = new TimeProfiler(1000);
    private TimeProfiler tp2 = new TimeProfiler(1000);
    private TimeProfiler tp3 = new TimeProfiler(1000);
    private TimeProfiler tp4 = new TimeProfiler(1000);
    private TimeProfiler tp5 = new TimeProfiler(1000);
    private TimeProfiler tp6 = new TimeProfiler(1000);
    private TimeProfiler tp7 = new TimeProfiler(1000);
    private TimeProfiler tp8 = new TimeProfiler(1000);
    @Override
    public void loop() {
        timeProfiler.markEnd();
        telemetry.addLine("UPS: " +
                1000.0/timeProfiler.getAverageTimePerUpdateMillis());
        timeProfiler.markStart();


//        telemetry.addLine("Speed Y : " + df.format(SpeedOmeter.getSpeedY()) +
//                " Speed X : " + df.format(SpeedOmeter.getSpeedX()) + " Speed Angle: " +
//                df.format(SpeedOmeter.getDegPerSecond()));


        telemetry.addLine("\nCollector reading: " + myCollector.getExtensionCurrPos());
        telemetry.addLine("\nLift reading: " + myLift.getExtensionCurrPos());






//        tunePIDS();


//        myPitScannerInterface.startScanning();
//        telemetry.addLine("ENCODERS:\nLeft: " + MyPosition.currPos_l +
//                " Right: " + MyPosition.currPos_r +
//                " Straif: " + MyPosition.currPos_a);

        if(gamepad2AlternateControls && gamepad2.a){
            getRevBulkData();
            myLift.setCurrentPositionTicks(0);
        }

        if(gamepad2AlternateControls && gamepad2.b){
            getRevBulkData();
            myCollector.setCurrentPositionTicks(0);
        }



//        telemetry.addData("Collector current position: ", myCollector.getExtensionCurrPos());
//        telemetry.addLine("hangMotorPosition: " + hangMechanism.getCurrentPosition());//display

//        telemetry.addLine("auxReading: " + MyPosition.currPos_a);
//        telemetry.addData("liftExtensionPosition", myLift.getExtensionCurrPos());
//        tuneScalingFactorsWithTuner();

        tp5.markStart();
        super.loop();
        tp5.markEnd();



//        followRobotVirtualField();
        tp1.markStart();

        HandleAlternateControlMode();
        ControlMovement();
//        debugTelemetry();

        //send the telemetry position back to the driver station for debugging
        reportPositionData();
        tp1.markEnd();

        tp2.markStart();
        AutoFeed();

        ControlFeeder();
        ControlCollector();

        tp2.markEnd();
        tp3.markStart();

        ResetMyPositionWhenGamepad1X();

        ControlManualCorrection(2.5);


        ControlAutoCollect();

        controlHangMechanism();
        tp3.markEnd();

        tp4.markStart();

        skipExchangeWithGamepad1();
        skipDumpingIfGamepad();


        tp4.markEnd();


//        telemetry.addLine("profile 1: " + tp1.getAverageTimePerUpdateMillis());
//        telemetry.addLine("profile 2: " + tp2.getAverageTimePerUpdateMillis());
//        telemetry.addLine("profile 3: " + tp3.getAverageTimePerUpdateMillis());
//        telemetry.addLine("profile 4: " + tp4.getAverageTimePerUpdateMillis());
//        telemetry.addLine("profile 5: " + tp5.getAverageTimePerUpdateMillis());
//        telemetry.addLine("profile 6: " + tp6.getAverageTimePerUpdateMillis());
//        telemetry.addLine("profile 7: " + tp7.getAverageTimePerUpdateMillis());
//        telemetry.addLine("profile 8: " + tp8.getAverageTimePerUpdateMillis());



//        TelemetryStuff();

    }



    /**
     * This allows gamepad1 to finish the exchange before the timeout
     */
    private void skipExchangeWithGamepad1() {
        if((gamepad1.a || (gamepad2.dpad_left && !gamepad2AlternateControls))
                && (AutoFeeder.myState == AutoFeeder.myStates.waitingForExchange.ordinal()
                || (AutoFeeder.myState == AutoFeeder.myStates.retractingCollectorAndMoveInLift.ordinal()))){
            //if we have activated the roller we can go right on to moving up lift
            if(myLift.getExtensionPercent() < 0.02){
                myAutoFeeder.nextStage();
                AutoFeeder.myState = AutoFeeder.myStates.movingUpLift.ordinal();
            }else{
                /*
                If we haven't even activated the roller, we are having
                the issue where the lift can't reset
                due to esd. We will recalibrate the lift encoder
                */
//                myLift.setCurrentPositionTicks(0);
            }
        }
    }


    double currMovementTurn = 0;
    double currMovementY = 0;
    private void debugTelemetry() {
        if(ButtonPress.isGamepad1_dpad_up_pressed()){
            currMovementY += 0.01;
        }
        if(ButtonPress.isGamepad1_dpad_down_pressed()){
            currMovementY -= 0.01;
        }
        if(ButtonPress.isGamepad1_dpad_right_pressed()){
            currMovementTurn += 0.01;
        }
        if(ButtonPress.isGamepad1_dpad_left_pressed()){
            currMovementTurn -= 0.01;
        }
        telemetry.addLine("currMovementY: " + currMovementY);
        telemetry.addLine("currMovementTurn: " + currMovementTurn);


        //if gamepad1.a, turn
        if(gamepad1.a){
            MovementVars.movement_y = currMovementY;
            MovementVars.movement_turn = currMovementTurn;
        }

    }


    /**
     * Controls the hangMechanism with gamepad1 triggers
     */
    private void controlHangMechanism() {
        hangMechanism.setPower(gamepad1.right_trigger-gamepad1.left_trigger);//go down
    }


    /**
     * If the gamepad1 presses a then move on from the dumper state
     */
    private void skipDumpingIfGamepad() {
        if((gamepad1.a || (gamepad2.dpad_left && !gamepad2AlternateControls))
                && AutoFeeder.myState == AutoFeeder.myStates.dumping.ordinal()){
            myAutoFeeder.nextStage();
            stopMovement();//stop just in case we were moving
            AutoFeeder.numAutoFeeds ++;
        }
    }

    /**
     * This method yeets the collectiong
     */
    private void ControlAutoCollect() {
        if(ButtonPress.isGamepad2_left_stick_button_pressed()){
//            myAutoFeeder.allCollectingLocations.clear();
//            myAutoFeeder.addCollectingLocation(0.5 + (gamepad2.left_stick_x/5.0));
            if(myAutoFeeder.isDoneAutoFeed()){
                myAutoCollector.autoCollect();
            }else{
                myAutoFeeder.collectAfterFeed();
            }
        }
    }


    /**
     * Switches to alternate control schemes when the user so desires
     */
    private void HandleAlternateControlMode() {
        //go into alternate controls while holding left bumper
        gamepad2AlternateControls = gamepad2.right_bumper;
    }


    /**
     * Controls the manual correction of the robot's position just in case.
     * However we want to visually move the target point so invert the direction
     * @param incrementResolution
     */
    private void ControlManualCorrection(double incrementResolution) {

        if(ButtonPress.isGamepad1_dpad_down_pressed()){
            MyPosition.setPosition(getXPos(),getYPos() + incrementResolution, getAngle_rad());
        }
        if(ButtonPress.isGamepad1_dpad_up_pressed()){
            MyPosition.setPosition(getXPos(),getYPos() - incrementResolution, getAngle_rad());
        }
        if(ButtonPress.isGamepad1_dpad_right_pressed()){
            MyPosition.setPosition(getXPos() - incrementResolution,getYPos(), getAngle_rad());
        }
        if(ButtonPress.isGamepad1_dpad_left_pressed()){
            MyPosition.setPosition(getXPos() + incrementResolution,getYPos(), getAngle_rad());
        }
    }

    /**
     * Resets the telemetry position when gamepad1 presses x or b (for depot)
      */
    private void ResetMyPositionWhenGamepad1X() {
        if(gamepad1.x){
            MyPosition.setPosition(143.63,141,Math.toRadians(-135));
//            MyPosition.setPosition(130,120,Math.toRadians(-90));

//            MyPosition.setPosition(0,0,Math.toRadians(0));
        }
        if(gamepad1.y){
            MyPosition.setPosition(216.7,142,Math.toRadians(-45));
        }
    }


    /**
     * Drives to the auto feed location
     */
    private void AutoFeed() {
        if(ButtonPress.isGamepad1_left_stick_button_pressed()){
            targetAutoFeedAngle = Math.abs(targetAutoFeedAngle-Math.toRadians(30))<0.0001 ?
                    Math.toRadians(50) : Math.toRadians(30);
        }
        if(gamepad1.left_stick_button) {
            driveToAutoFeed();
        }


        if(gamepad1.right_stick_button){
//            driveToAutoFeed();
            orbitModeFeeder(gamepad1.right_stick_x);
        }
     }

     //if in the last update we were in collector absolute position control mode
     boolean inCollectorAbsolutePositionModeLast = false;
     /**
     * This handles all user controls of the collector and it's extension
     */
    private void ControlCollector() {
        /**
         * First control the roller
         */
        if(ButtonPress.isGamepad2_a_pressed() && !gamepad2AlternateControls){
            if(myCollector.rollerState != Collector.rollerStates.forwards){
                myCollector.turnOnRoller();
            }else{
                myCollector.turnOffCollector();
            }
        }
        if(ButtonPress.isGamepad2_b_pressed() && !gamepad2AlternateControls){
            if(myCollector.rollerState != Collector.rollerStates.reverse){
                myCollector.reverseCollector();
            }else{
                myCollector.turnOffCollector();
            }
        }


        /**
         * Now for controlling the extension.
         * There are two modes:
         * 1. Speed/power control which is when the user has direct control over
         * the collector power.
         *
         * 2. Absolute position control which is when the user controls the absolute distance
         * away the collector should be from the robot
         */
        //control the extension with gamepad2.right_stick_y
        double scaleDownPower = 1;
        double collectorPower = -gamepad2.right_stick_y * scaleDownPower;

        //if gamepad1.left_bumper then we are in absolute position mode and collector orbit mode
        boolean inCollectorAbsolutePositionMode = gamepad1.left_bumper;//Math.abs(SpeedOmeter.getSpeedY()) * 100.0 > 3;
        /**
         * Save the current radius in the transition between the two modes
         */
        if(inCollectorAbsolutePositionMode && !inCollectorAbsolutePositionModeLast){
            myCollector.maintainCollectorCurrentRadius();
        }
        inCollectorAbsolutePositionModeLast = inCollectorAbsolutePositionMode;

        if(inCollectorAbsolutePositionMode){
            //adjust the target radius
            myAutoFeeder.adjustCollectorTargetRadius(collectorPower);

        }else{
            myAutoFeeder.setCollectorExtensionPowerNice(collectorPower);
        }
















        /**DUMPER CONTROLS */
        if(ButtonPress.isGamepad2_dpad_up_pressed()){
            myCollector.activateDumper();
        }
        if(ButtonPress.isGamepad2_dpad_down_pressed()){
            myCollector.retractDumper();
        }
    }

    private void ControlFeeder() {
        //control the feeder with the triggers (gamepad2)
        double liftPower = gamepad2.right_trigger - gamepad2.left_trigger;
        myAutoFeeder.setLiftExtensionPowerNice(liftPower);





        //automatic feeding with right bumper
        if(ButtonPress.isGamepad2_dpad_right_pressed()){
            //use telemetry if we are not on alternate controls (default) else don't
            boolean allowedToDrive = !gamepad2AlternateControls;

            myAutoFeeder.autoFeed(allowedToDrive);
        }

        /**LEFT BUMPER ABORTS AUTO FEED */
        if(gamepad2.left_bumper){
            myAutoFeeder.abortAutoFeed();
            myAutoCollector.abortAutoCollect();
        }
        if(gamepad2AlternateControls && ButtonPress.isGamepad2_x_pressed()){
            myAutoFeeder.extendLiftToTop();
        }

//        if(gamepad2AlternateControls && ButtonPress.isGamepad2_b_pressed()){
//            myAutoFeeder.resetLiftToBottom();
//        }


        orbitAmount = gamepad1.right_stick_x * 0.6;//set orbit amount to movement_x




        /** DUMPING AND UNDUMPING CONTROLS */
        if(!gamepad2AlternateControls && ButtonPress.isGamepad2_x_pressed()){
            myLift.startDumpAdvanced();
        }
        if(!gamepad2AlternateControls && ButtonPress.isGamepad2_y_pressed()) {
            myLift.unDump();
        }
        /** /////////////////////////////// */
    }



    private void TelemetryStuff() {


//        if(SystemClock.uptimeMillis() - lastUpdateTime < 20){return;}
//        lastUpdateTime = SystemClock.uptimeMillis();


//        DrawVirtualField();

        //telemetry.addData("auxWheel: ", MyPosition.currPos_a );


//
//        percentElectron -= 0.05;
//        if(percentElectron <= 0){
//            percentElectron = 1;
//            electronVert --;
//            if(electronVert < 0){electronVert =5;}
//        }




        /*
        double radius = 33;
        double vertices = 6;
        int x =  m_telemetry.size_x/2;
        int y = m_telemetry.size_y/2;
        for(int v = 0; v < vertices; v++){
            double a1 = phaseShift + (Math.toRadians(360/vertices)*v);
            double a2 = phaseShift + (Math.toRadians(360/vertices)*(v+1));

            double yRatio = 0.53;
            int x1 = (int) (Math.cos(a1)*radius) + x;
            int y1 = (int) (Math.sin(a1)*radius * yRatio) + y;
            int x2 = (int) (Math.cos(a2)*radius) + x;
            int y2 = (int) (Math.sin(a2)*radius * yRatio) + y;

            m_telemetry.drawLine(x1,y1,x2,y2,'c');


            if(electronVert == v){
                int e_x = (int) (percentElectron * (double) (x2-x1))+ x1;
                int e_y = (int) (percentElectron * (double) (y2-y1))+ y1;
                m_telemetry.putChar(e_x,e_y,'e');
                m_telemetry.putChar(e_x,e_y+1,'e');
                m_telemetry.putChar(e_x,e_y-1,'e');
                m_telemetry.putChar(e_x+1,e_y,'e');
                m_telemetry.putChar(e_x-1,e_y,'e');
            }
        }



        radius = 23;
        vertices = 6;
        x =  m_telemetry.size_x/2;
        y = m_telemetry.size_y/2;
        for(int v = 0; v < vertices; v++){
            double a1 = -phaseShift * 1.5 + (Math.toRadians(360/vertices)*v);
            double a2 = -phaseShift * 1.5+ (Math.toRadians(360/vertices)*(v+1));

            double yRatio = 0.53;
            int x1 = (int) (Math.cos(a1)*radius) + x;
            int y1 = (int) (Math.sin(a1)*radius * yRatio) + y;
            int x2 = (int) (Math.cos(a2)*radius) + x;
            int y2 = (int) (Math.sin(a2)*radius * yRatio) + y;

            m_telemetry.drawLine(x1,y1,x2,y2,'c');
        }


        m_telemetry.putChar(x - 5, y, 'L');
        m_telemetry.putChar(x - 4, y, 'o');
        m_telemetry.putChar(x - 3, y, 'a');
        m_telemetry.putChar(x - 2, y, 'd');
        m_telemetry.putChar(x - 1, y, 'i');
        m_telemetry.putChar(x - 0, y, 'n');
        m_telemetry.putChar(x + 1, y, 'g');

        if(loading < 1){

        }else{
            if(loading < 2){
                m_telemetry.putChar(x + 2, y, '.');
            }else{
                if(loading < 3){
                    m_telemetry.putChar(x + 2, y, '.');
                    m_telemetry.putChar(x + 3, y, '.');
                }else{
                    if(loading < 4){
                        m_telemetry.putChar(x + 2, y, '.');
                        m_telemetry.putChar(x + 3, y, '.');
                        m_telemetry.putChar(x + 4, y, '.');
                    }
                }
            }
        }

        loading += 0.013;
        if(loading >= 4){loading = 0;}
        phaseShift += Math.toRadians(0.3);

        */


        /*
        //robot movement test
        if(gamepad1.dpad_left){
            robotX -= 0.03;
        }
        if(gamepad1.dpad_right){
            robotX += 0.03;
        }
        if(gamepad1.dpad_up){
            robotY += 0.03;
        }
        if(gamepad1.dpad_down){
            robotY -= 0.03;
        }
        robotAng -= Math.toRadians(3) * gamepad1.right_stick_x;

        */
        //center_x += gamepad1.left_stick_x * 0.03/zoom;
        //center_y -= gamepad1.left_stick_y * 0.03/zoom;






//        telemetry.addLine("<font face='monospace'>");



//        if(FtcRobotControllerVisionActivity.updateFPV ){return;}
//        for(int i = 0; i < m_telemetry.size_y; i ++){
//            //
//            telemetry.addLine(m_telemetry.getLine(i));//.replace(" ", "."));
//        }

//        FtcRobotControllerVisionActivity.updateFPV = true;


//        telemetry.addLine("</font face='monospace'>");

    }



    double phaseShift = 0;
    double percentElectron =1;
    int electronVert = 0;
    double loading = 0;


    /**
     * Called when driver presses stop
     * makes the next teleop start down
     */
    public void stop(){
        //set the initial dumper position of the collector to activated in case we are
        //rerunning teleop
        initialCollectorDumperPosition = 0.0;
    }
}

