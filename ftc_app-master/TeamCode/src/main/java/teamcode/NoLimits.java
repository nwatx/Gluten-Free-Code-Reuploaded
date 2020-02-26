package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import HelperClasses.Auto;
import HelperClasses.Robot;


@Autonomous(name = "NoLimits", group = "NoLimits")
public class NoLimits extends Robot {

    @Override
    public void init() {
        //don't need to worry about starting in the sizing
        setStartIn18Inches(false);
        super.init();
    }

    @Override
    public void init_loop(){
        super.init_loop();
    }

    @Override
    public void start(){
        super.start();
    }

    @Override
    public void loop(){
        super.loop();
        hangMechanism.setPowerNoLimits(gamepad1.right_trigger-gamepad1.left_trigger);
        ControlMovement();
        hangMechanism.ResetEncoder();
    }


}

