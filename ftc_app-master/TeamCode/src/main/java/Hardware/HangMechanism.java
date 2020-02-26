package Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import HelperClasses.Robot;

public class HangMechanism {
    public static final int MAX_RANGE = 3400;
    public static final int MARGIN_ENDS = 25;

    private RevMotor hangMotor;

    /**
     * initializes hangMotor
     * @param hangMotor DcMotor of the hang mechanism
     */
    public HangMechanism(RevMotor hangMotor){
        hangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hangMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.hangMotor = hangMotor;
    }


    //current reading of the hang encoder
    private double hangMotorCurrentReading = 0;
    //current power of the hang motor
    private double hangMotorPower = 0;
    //last power set to the motor
    private double hangMotorPowerLast = 0;
    public void update(){
        //get the current motor reading
        hangMotorCurrentReading = hangMotor.getCurrentPosition();
        //set the power
        hangMotor.setPower(hangMotorPower);
    }

    /**
     * Resets the encoder
     */
    public void ResetEncoder(){
        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    /**
     * Allows you to set the power of
     * @param power
     */
    public void setPower(double power){
        if((hangMotorCurrentReading >= MAX_RANGE - MARGIN_ENDS && power > 0) ||
                (hangMotorCurrentReading <= MARGIN_ENDS && power < 0.0)){
            power = 0;
        }

        if(getCurrentPercent() > 0.9 && power > 0){ power *= 0.3; }
        if(getCurrentPercent() < 0.1 && power < 0){ power *= 0.3; }

        hangMotorPower = power;
    }

    public int getCurrentPosition() {
        return (int) hangMotorCurrentReading;
    }

    /**
     * Returns percentage wise how extended the hanger is
     * @return
     */
    public double getCurrentPercent(){
        return (hangMotorCurrentReading-MARGIN_ENDS) / (MAX_RANGE-MARGIN_ENDS);
    }


    /**
     * Sets the motor power without the limits on the end
     * Dream big and you can achieve anything
     * @param power
     */
    public void setPowerNoLimits(double power) {
        hangMotorPower=power;
    }


    /**
     * Gets the current power of the hang motor
     * @return the current power from -1 to 1
     */
    public double getCurrentPower(){
        return hangMotorPower;
    }


}
