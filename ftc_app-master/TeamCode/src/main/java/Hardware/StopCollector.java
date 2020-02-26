package Hardware;

import android.os.SystemClock;

import com.qualcomm.robotcore.hardware.AnalogInput;

/**
 * Stops the collector automatically when pulling up the collector dumper
 * This is similar to encoder controller (which currently isn't used)
 *
 */
class StopCollector {
    //tracks the collector encoder continuously
    private Tracker myTracker;

    //current velocity of the encoder in rotations per second
    private double currVelocity = 0;

    /**
     * Constructor
     * @param encoder the analog encoder (AnalogInput)
     */
    public StopCollector(AnalogInput encoder){
        myTracker = new Tracker(encoder);//initialize our tracker (turns analog to continuous)
    }



    //last update time in milliseconds
    private long lastUpdateTime = 0;

    private final int milisecondsPerUpdate = 50;//maximum rate at which the speed will be sampled
    /**
     * Call this often (every update)
     *
     */
    public void update(){
        //update the tracker even if 50 milliseconds haven't past
        myTracker.update();

        //get the current time
        long currTime = SystemClock.uptimeMillis();
        //calculate the elapsedTime since the last update
        long elapsedTime = currTime - lastUpdateTime;
        if(elapsedTime > milisecondsPerUpdate){
            lastUpdateTime = currTime;//save the time of this update for next time
            calculateSpeed(elapsedTime);//if we need to know our speed

        }
    }


    /**
     * Sees if now is a good time to activate the collector
     * @return
     */
//    public boolean seeIfTimeToGo(){
//
//        return myTracker.getPos() % 1
//    }






    //last position of the encoder
    private double lastEncoderPosition = 0;

    /**
     * Called every 50 milliseconds
     * @param elapsedTimeMillis the amount of time that has passed since the last update
     */
    private void calculateSpeed(long elapsedTimeMillis) {
        //get the current position of the encoder
        double currPosition = myTracker.getPos();

        //get how many rotations have occurred since the last update
        double elapsedRotations = currPosition-lastEncoderPosition;
        //calculate how many seconds have elapsed
        double elapsedSeconds = elapsedTimeMillis/1000.0;

        //now we can calculate the rotations per second of the encoder
        currVelocity = elapsedRotations/elapsedSeconds;

        //remember the last position -> the currPosition
        lastEncoderPosition = currPosition;
    }



}

