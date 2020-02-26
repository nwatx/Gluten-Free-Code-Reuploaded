package Globals;

/**
 * Contains static variables about the details of our current run
 */
public class Globals {
    /** What auto mode we are on */
    public static AutoModes autoMode = AutoModes.auto1;
    public enum AutoModes{
        auto1,
        auto2
    }


    /**
     * @return true if collecting from the crater in auto
     */
    public static boolean isCrater(){
        return autoMode == AutoModes.auto1;
    }
}
