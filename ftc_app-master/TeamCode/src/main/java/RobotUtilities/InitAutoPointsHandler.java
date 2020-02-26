package RobotUtilities;

import android.os.SystemClock;

import java.util.ArrayList;

import HelperClasses.ButtonPress;
import HelperClasses.CollectingLocation;
import HelperClasses.Robot;

import static HelperClasses.Robot.m_telemetry;

//this handles all the auto points stuff, everything is static so you don't need an instance
public class InitAutoPointsHandler {

    public static ArrayList<CollectingLocation> allAutoKeyPoints;
    public static void initializeAutoKeyPoints() {
        //nothing in it so far, maybe change this initialization to have presets like last year
        allAutoKeyPoints = new ArrayList<>();

        //start with entering the first point
        enterPointState = enterPointStates.firstPoint;
    }


    public static long lastInitLoopTime = 0;
    public static double selectX = 180;
    public static double selectY = 180;



    private static enterPointStates enterPointState = enterPointStates.firstPoint;
    private enum enterPointStates{
        firstPoint,
        secondPoint
    }


    //called during init loop of autonomous, deals with input
    public static void update(double gamepad1LeftStickX, double gamepad1LeftStickY,
                              double currZoom) {
        /**HANDLES PANNING */
        long currTime = SystemClock.uptimeMillis();
        double elapsed = (currTime-lastInitLoopTime)/1000.0;//elapsed time in seconds

        //pans according to elapsed time, and 1/zoom so when we zoom in (bigger zoom) less movement
        Robot.panXVirtualField(gamepad1LeftStickX * elapsed * (1.0/currZoom));
        Robot.panYVirtualField(-gamepad1LeftStickY * elapsed * (1.0/currZoom));

        lastInitLoopTime = currTime;
        /**----------------*/


        //allows the user to move a pointer around, this is according to the zoom, so when you
        //zoom in it will move less
        selectX += ButtonPress.isGamepad1_dpad_right_pressed() ? 10.0 * (1.0/currZoom) : 0;
        selectX += ButtonPress.isGamepad1_dpad_left_pressed() ? -10.0 * (1.0/currZoom): 0;
        selectY += ButtonPress.isGamepad1_dpad_up_pressed() ? 10.0 * (1.0/currZoom) : 0;
        selectY += ButtonPress.isGamepad1_dpad_down_pressed() ? -10.0 * (1.0/currZoom) : 0;

        //draw the cross hair to show where you are
        m_telemetry.putCharField(selectX,selectY,'+');



        if(ButtonPress.isGamepad1_left_stick_button_pressed()){

            if(enterPointState == enterPointStates.firstPoint){
                allAutoKeyPoints.add(new CollectingLocation(selectX,selectY));
            }
            if(enterPointState == enterPointStates.secondPoint){
                allAutoKeyPoints.get(allAutoKeyPoints.size()-1).pointLocationX = selectX;
                allAutoKeyPoints.get(allAutoKeyPoints.size()-1).pointLocationY = selectY;
            }

            //now switch the state (if first point, now we enter second and vice versa)
            if(enterPointState == enterPointStates.firstPoint)
            {enterPointState = enterPointStates.secondPoint;}
            else{enterPointState = enterPointStates.firstPoint;}
        }

        //Now we draw all the points
        for(int i = 0; i < allAutoKeyPoints.size(); i ++){
            //so this line might not be fully defined. if it is, great draw the line otherwise
            //draw a line between the first point and the cursor
            if(allAutoKeyPoints.get(i).pointLocationX != -1){
                m_telemetry.drawLine(allAutoKeyPoints.get(i).xApproach,allAutoKeyPoints.get(i).yApproach,
                        allAutoKeyPoints.get(i).pointLocationX,allAutoKeyPoints.get(i).pointLocationY,
                        255,255,255,(char)('0' + (i+1)));
            }else{
                m_telemetry.drawLine(allAutoKeyPoints.get(i).xApproach,allAutoKeyPoints.get(i).yApproach,
                        selectX,selectY,255,255,255,(char)('0' + (i+1)));
            }

        }
    }





}
