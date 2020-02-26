package Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.openftc.revextensions2.ExpansionHubMotor;

import HelperClasses.Auto;

/**
 * This class holds a motor object and encoder readings of a motor
 */
public class RevMotor {

    public ExpansionHubMotor myMotor;

    //this is if we are on the master or the slave hub (true if master)
    public boolean isMaster;
    //current position of the motor in encoder ticks
    private int currPosition = 0;


    //this is a tally of how many motor power sets we are doing each update
    public static int numHardwareUsesThisUpdate = 0;


    /**
     * Initializes a RevMotor
     * @param motor the motor
     * @param master true if this is on the master expansion hub
     */
    public RevMotor(ExpansionHubMotor motor, boolean master){
        myMotor = motor;
        isMaster = master;
    }




    //the last power the motor was set to
    private double lastPower = -1.0;

    /**
     * Sets the power of the motor
     * @param power the power you want to go
     */
    public void setPower(double power){
        double powerToApply = power * Auto.masterMotorScale;

        if(Math.abs(powerToApply - lastPower) > 0.005 ||
                (powerToApply == 0 && lastPower != 0)){
            myMotor.setPower(powerToApply);//set the power of the motor

            numHardwareUsesThisUpdate ++;//this is a hardware use
            lastPower = powerToApply;
        }

    }


    /**
     * Setsthe current position of the motor
     */
    public void setEncoderReading(int position){ currPosition = position; }

    /**
     * Gets the current position of the motor
     * @return the current position of the motor
     */
    public int getCurrentPosition() {
        return currPosition;
    }

    /**
     * Sets the runmode of the motor
     */
    public void setMode(DcMotor.RunMode runMode) {
        myMotor.setMode(runMode);
        numHardwareUsesThisUpdate ++;//this is a hardware use
    }

    /**
     * Sets the direction of the motor
     * @param direction the direction
     */
    public void setDirection(DcMotorSimple.Direction direction) {
        myMotor.setDirection(direction);
        numHardwareUsesThisUpdate ++;//this is a hardware use
    }

    /**
     * Sets the ZeroPowerBehavior of the motor
     * @param zeroPowerBehavior behaviour
     */
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        myMotor.setZeroPowerBehavior(zeroPowerBehavior);
        numHardwareUsesThisUpdate ++;//this is a hardware use
    }

    /**
     * Sets the target position of the motor
     * @param targetPosition the target position
     */
    public void setTargetPosition(int targetPosition) {
        myMotor.setTargetPosition(targetPosition);
        numHardwareUsesThisUpdate ++;//this is a hardware use
    }

    /**
     * Gets the runmode of us
     * @return the runmode
     */
    public DcMotor.RunMode getMode() {
        numHardwareUsesThisUpdate ++;//this is a hardware use
        return myMotor.getMode();
    }



    /**
     * This is used mostly for debugging, call this once per update.
     * Knowing where updates start allows us to keep track of how many motors we are setting
     * powers to and stuff at a time
     */
    public static void markEndUpdate(){
        numHardwareUsesThisUpdate = 0;
    }
}
