package org.firstinspires.ftc.robotcontroller.Vision;

import org.firstinspires.ftc.robotcontroller.RobotUtilities.CurvePoint;
import org.firstinspires.ftc.robotcontroller.RobotUtilities.PiecewiseFunction;

import java.util.ArrayList;

public class Vision {
    public native static void firstPerson(long addrRgba);


    public static double bestBallX = 0;
    public static double bestBallY = 0;
    public native static int readBallPattern(long addrRgba,double sample1X,double sample2X,
                                             double sample3X,double sample1Y,double sample2Y, double sample3Y);


    public static double getRelativeBallLocation(){
        ArrayList<CurvePoint> ballLocationFunctionPoints = new ArrayList<>();
        ballLocationFunctionPoints.add(new CurvePoint(0.168,24*2.54));
        ballLocationFunctionPoints.add(new CurvePoint(0.275,36*2.54));
        ballLocationFunctionPoints.add(new CurvePoint(0.322,48*2.54));
        ballLocationFunctionPoints.add(new CurvePoint(0.356,64*2.54));
        ballLocationFunctionPoints.add(new CurvePoint(0.379,84*2.54));
        ballLocationFunctionPoints.add(new CurvePoint(0.397,123*2.54));



        //now that we have the points we can initialize our piecewise function by passing them in
        PiecewiseFunction ballLocationFunction = new PiecewiseFunction(ballLocationFunctionPoints);




        return ballLocationFunction.getVal(bestBallY)-(5*2.54);
    }
}
