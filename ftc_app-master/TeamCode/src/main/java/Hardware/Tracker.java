package Hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;

/**
 * This class converts an analog encoder into an incremental one by looking for
 * wrap arounds in the signal
 */
public class Tracker {
    AnalogInput m_encoder;
    public static final double WRAP_AROUND_THRESHOLD = 0.45;
    public static final double limit_top = 1;
    public static final double limit_bot = 0;

    private double resetReading = 0.0f;
    private double currentReading = 0.0f;
    private double lastReading = 0.0f;
    private double lastValidReading = 0.0f;

    int num_of_wraparounds = 0;

    boolean wrapingAround = false;




    public Tracker(AnalogInput encoder){
        m_encoder = encoder;
        reset();
    }

    public void reset(){
        num_of_wraparounds = 0;
        resetReading = 0;//getCurrentReading();
        currentReading = getCurrentReading();
        lastReading = getCurrentReading();
        lastValidReading = getCurrentReading();
    }

    private double getCurrentReading(){
        return Math.abs(m_encoder.getVoltage()/ m_encoder.getMaxVoltage());
    }

    public void update(){
        currentReading = getCurrentReading();
        double delta_temp = currentReading-lastReading;
        if(Math.abs(delta_temp) > WRAP_AROUND_THRESHOLD){
            wrapingAround = true;
        }else{
            if(wrapingAround){
                wrapingAround = false;
                if(currentReading-lastValidReading >0){
                    num_of_wraparounds --;
                }else{
                    num_of_wraparounds ++;
                }
            }

            lastValidReading = currentReading;
        }

        lastReading = currentReading;
    }
    public double getPos(){
        return (num_of_wraparounds+((lastValidReading-resetReading)-limit_bot)/(limit_top-limit_bot));
    }
    public double getRAW(){
        return currentReading;
    }
}