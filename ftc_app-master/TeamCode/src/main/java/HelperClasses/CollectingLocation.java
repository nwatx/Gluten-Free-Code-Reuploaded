package HelperClasses;

import FieldStats.Field;
import Globals.Globals;

/**
 * This class is used to mark a collecting location
 */
public class CollectingLocation {

    public double xApproach;
    public double yApproach;
    //by default point to the corner
    public double pointLocationX = 0;
    public double pointLocationY = 0;


    //how far (percentage) the collector will extend
    public double percentCollector = 0.8;



    //the way you specify a collecting location is with a percentage
    //this dictates from left to right (0-1.0 accordingly) where the robot should attack
    //this constructor will assume you want to point to the corner only, nothing advanced
    public CollectingLocation(double percentLeftToRightCrater,boolean homeCrater){

        setPercentApproach(percentLeftToRightCrater,homeCrater);

        percentCollector = 0.8;
    }

    public CollectingLocation(double x, double y){
        this.xApproach = x;
        this.yApproach = y;
        pointLocationX = 0;
        pointLocationX = 0;
    }
    //initialize with 2 points, the location and the point location
    public CollectingLocation(double x, double y,double pointX, double pointY){
        this.xApproach = x;
        this.yApproach = y;
        this.pointLocationX = pointX;
        this.pointLocationY = pointY;
    }



    //get's the angle you should point during the collection
    public double getAngle(){
        return Math.atan2(pointLocationY - yApproach,pointLocationX- xApproach);
    }


    /**
     * Converts us to a string
     * @return a string representation of the object
     */
    public String toString(){
        return "X: " + xApproach + " Y: " + yApproach + " pointX: " + pointLocationX + " pointY" + pointLocationY;
    }

    /**
     * Use this to re-specify the percent along the crater you want to collect from
     * @param percent the percent from bottom right to top left
     */
    public void setPercentApproach(double percent, boolean homeCrater) {
        if(homeCrater){
            //so if we have a percent, we can just interpolate each of the components accordingly
            xApproach = percent * (Field.CRATER1_TOP_LEFT_X - Field.CRATER1_BOTTOM_RIGHT_X);
            xApproach += Field.CRATER1_BOTTOM_RIGHT_X;

            yApproach = percent * (Field.CRATER1_TOP_LEFT_Y - Field.CRATER1_BOTTOM_RIGHT_Y);
            yApproach += Field.CRATER1_BOTTOM_RIGHT_Y;
        }else{
            //so if we have a percent, we can just interpolate each of the components accordingly
            xApproach = percent * (Field.CRATER2_TOP_LEFT_X - Field.CRATER2_BOTTOM_RIGHT_X);
            xApproach += Field.CRATER2_BOTTOM_RIGHT_X;

            yApproach = percent * (Field.CRATER2_TOP_LEFT_Y - Field.CRATER2_BOTTOM_RIGHT_Y);
            yApproach += Field.CRATER2_BOTTOM_RIGHT_Y;

        }
    }
}
