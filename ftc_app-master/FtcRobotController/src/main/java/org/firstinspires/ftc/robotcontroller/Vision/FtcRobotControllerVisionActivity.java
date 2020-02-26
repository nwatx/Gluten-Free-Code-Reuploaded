package org.firstinspires.ftc.robotcontroller.Vision;

import android.os.Bundle;
import android.view.View;

import com.qualcomm.ftcrobotcontroller.R;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCameraView;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

/**
 * This class is a custom FtcRobotControllerActivity that implements a Camera Listener.
 * It handles all the vision stuff
 */
public class FtcRobotControllerVisionActivity extends FtcRobotControllerActivity
        implements CameraBridgeViewBase.CvCameraViewListener2
{
    /**
     * Use this to access the instance data because it is set in on create
     */
    public static FtcRobotControllerVisionActivity linkToInstance;
    Mat mRgba;

    private static String mTAG = "visionTAG";
    JavaCameraView javaCameraView;

    boolean loadedVision = false;

    BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this)
    {
        @Override
        public void onManagerConnected(int status)
        {
            switch (status)
            {
                case BaseLoaderCallback.SUCCESS:
                    javaCameraView.enableView();

                    //only load this our native lib once
                    if(!loadedVision){
                        System.loadLibrary("native-lib");
                        loadedVision = true;
                    }
                    break;
                default:
                    super.onManagerConnected(status);
            }
        }
    };

    public void onCameraViewStarted(int width, int height)
    {
        mRgba = new Mat(height, width, CvType.CV_8UC4);
    }

    public void onCameraViewStopped()
    {
        mRgba.release();
    }



    public static boolean updateFPV = false;


    /**
     * We need to know the robot's angle to filter some vision things
     * This will be set by the teamcode module
     */
    public static double worldAngle_rad = Math.toRadians(-135);
    public static double worldXPosition = 138;
    public static double worldYPosition = 138;




    /**
     * This uses the robot angle to calculate the crater angle on the screen for the vision
     * @return angle in radians of the crater
     */
    private double calculateCraterAngle(double robotAngle) {
        //when we are at -142 degrees the angle should be paralle
        //However, let's multiply by 0.7 because of the angle of the phone.
        //This is a big approximation but kinda works so sue me.
        return (robotAngle - Math.toRadians(-142))*0.5;
    }


    //holds the position data about the minerals
    public static double[] xPositions;
    public static double[] yPositions;
    public static int numMinerals = 0;




    //if true, we will block out the collector in the image to encourage choosing from the left
    //or right
    public static boolean isSecondScan = false;




    /**
     * This runs all our vision processing code and is called by the opencv camera listener
     * @param inputFrame mat (image) from the camera
     * @return the modified mat to display
     */
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame)
    {
        //get the inputFrame data
        mRgba = inputFrame.rgba();




//        Mat rotatedImage = new Mat(mRgba.cols(),mRgba.rows(), CvType.CV_8UC4);
//        Core.rotate(mRgba, rotatedImage, Core.ROTATE_90_CLOCKWISE); //ROTATE_180 or ROTATE_90_COUNTERCLOCKWISE


//        rotatedImage.copyTo(mRgba.rowRange(0,mRgba.rows()-1).colRange(0,mRgba.cols()-1));



        imageSizeX = mRgba.cols();//remember the size for future reference
//        if(updateFPV){
//            Vision.firstPerson(mRgba.getNativeObjAddr());
//        }
//        updateFPV = false;




        //these will be populated with the mineral locations
        double[] tempXPositions = new double[mRgba.cols()/4 * mRgba.rows()/4];
        double[] tempYPositions = new double[mRgba.cols()/4 * mRgba.rows()/4];

        //This will scan the pit using the native method and convert the minerals to field coordinates
//        PitScanner.scanPit(mRgba.getNativeObjAddr(),calculateCraterAngle(worldAngle_rad),
//                worldXPosition,worldYPosition,worldAngle_rad,tempXPositions,tempYPositions,
//                isSecondScan);

        xPositions = tempXPositions;
        yPositions = tempYPositions;


        //do a debug count of all the minerals
        int countMinerals = 0;
//        for(int i = 0; i < xPositions.length; i ++){
//            countMinerals ++;
//
////            if(xPositions[i] != 0){
////            }else{
////                break;
////            }
//        }

        numMinerals = countMinerals;


        cubeLocation = Vision.readBallPattern(mRgba.getNativeObjAddr(),sample1X, sample2X, sample3X,sample1Y,sample2Y,sample3Y);



//        if(killOpenCV){
//            if(!openCVKilled){
//                javaCameraView.disableView();
//                Imgproc.rectangle(mRgba,new Point(0,0),new Point(mRgba.width(),mRgba.height()),new Scalar(0,0,0));
//                openCVKilled = true;
//            }
//        }else{
//            if(openCVKilled){
//                OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION,this, mLoaderCallback);
//                openCVKilled = false;
//            }
//        }

        return mRgba;
    }

    public static int imageSizeX = 0;

    /**
     * These are the sample locations (where the robot will look for the particles)
     */
    private static double sample1X = 0;
    private static double sample2X = 0;
    private static double sample3X = 0;
    private static double sample1Y = 240;
    private static double sample2Y = 240;
    private static double sample3Y = 240;

    //these are how you set teh sample
    public static double getSample1X(){ return sample1X; }
    public static double getSample2X(){ return sample2X; }
    public static double getSample3X(){ return sample3X; }
    public static double getSample1Y() { return sample1Y; }
    public static double getSample2Y() { return sample2Y; }
    public static double getSample3Y() { return sample3Y; }

    public static void setSample1X(double x){ sample1X = x; }
    public static void setSample2X(double x){ sample2X = x; }
    public static void setSample3X(double x){ sample3X = x; }
    public static void setSample1Y(double y) {sample1Y = y;}
    public static void setSample2Y(double y) {sample2Y = y;}
    public static void setSample3Y(double y) {sample3Y = y;}





    private static int cubeLocation = 0;
    public static int getCubeLocation(){return cubeLocation;}//gets the cube location (0,1,2)




    public static void initializeSampleLocations() {

        //s5 defaults
//        setSample1X(222.8);
//        setSample2X(892);
//        setSample3X(1121);
//        setSample1Y(150);

        //g2 defaults
        setSample1X(122.8);
        setSample2X(552);
        setSample3X(691);
        setSample1Y(370);

    }



    @Override
    public void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
        linkToInstance = this;//if you want an instance

        javaCameraView = (JavaCameraView) findViewById (R.id.java_camera_view);
        javaCameraView.setVisibility(View.VISIBLE);
        //we want to use the front cam for seeing jewels
//        javaCameraView.setCameraIndex(1);
        javaCameraView.setCvCameraViewListener(this);
//        javaCameraView.disableView();
        killOpenCV = false;
    }

    @Override
    public void onResume()
    {
        super.onResume();
        OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION,this, mLoaderCallback);
    }

    @Override
    public void onPause()
    {
        super.onPause();

        if(javaCameraView!=null){
            javaCameraView.disableView();
        }
    }

    @Override
    public void onDestroy()
    {
        super.onDestroy();

        if(javaCameraView!=null){
            javaCameraView.disableView();
        }
    }


    private static boolean killOpenCV = false;
    private static boolean openCVKilled = false;
    public static void killOpenCV(){
        killOpenCV = true;
    }
    public static void reviveOpenCV(){
        killOpenCV = false;
    }

    /**
     * Use this to disable the camera view
     */
    public void disableView() {
        if(javaCameraView != null){
            javaCameraView.disableView();
        }
    }

    /**
     * Re-enables the java camera view if it was shut down
     */
    public void enableView(){
        if(javaCameraView != null){
            javaCameraView.enableView();
        }
    }
}
