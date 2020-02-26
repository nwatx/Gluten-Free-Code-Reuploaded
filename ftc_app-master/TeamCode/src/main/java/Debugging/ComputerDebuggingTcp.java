package Debugging;


import HelperClasses.Robot;
import ReturnTypes.FloatPoint;

public class ComputerDebuggingTcp {
    private static TcpServer tcpServer;


    /**
     * Initializes udp server and starts it's thread
     */
    public ComputerDebuggingTcp(){
        TcpServer.kill = false;
        tcpServer = new TcpServer(11115);
        Thread runner = new Thread(tcpServer);
        runner.start();//go go go
    }



    /**
     * Sends the robot location to the debug computer
     */
    public void sendRobotLocation(Robot robot){
        tcpServer.addMessage("ROBOT," + robot.getXPos() + "," + robot.getYPos() + "," + robot.getAngle_rad());

        //also draw the collector as a line for now
        sendLine(new FloatPoint(robot.getXPos(),robot.getYPos()),new FloatPoint(
                robot.getXPos() + (Math.cos(robot.getAngle_rad()) * robot.myCollector.getCurrExtensionDistFromCenter()),
                robot.getYPos() + (Math.sin(robot.getAngle_rad()) * robot.myCollector.getCurrExtensionDistFromCenter())));
    }

    /**
     * Sends the location of any other point you would like to send
     * @param floatPoint
     * @throws InterruptedException
     */
    public static void sendKeyPoint(FloatPoint floatPoint) {
        tcpServer.addMessage("POINT," + floatPoint.x + "," + floatPoint.y);
    }

    /**
     * Used for debugging lines
     * @param point1
     * @param point2
     */
    public static void sendLine(FloatPoint point1, FloatPoint point2){
        tcpServer.addMessage("LINE," + point1.x + "," + point1.y + "," + point2.x + "," + point2.y);
    }


    /**
     * This kills the tcpServer background thread
     */
    public static void stopAll() {
        tcpServer.kill = true;
    }


    /**
     * Call this at the end of every update so that we don't buffer too much
     */
    public static void markEndOfUpdate(){
        tcpServer.markEndUpdate();
    }
}
