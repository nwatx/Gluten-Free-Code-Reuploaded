package teamcode;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.Vision.FtcRobotControllerVisionActivity;

import RobotUtilities.TelemetryAdvanced;

@TeleOp
public class FPV extends LinearOpMode
{
    TelemetryAdvanced m_telemetry;

    long lastUpdateTime = 0;
    @Override
    public void runOpMode() throws InterruptedException
    {
        m_telemetry = new TelemetryAdvanced(59,53);
        telemetry.setMsTransmissionInterval(0);

        waitForStart();

        while (opModeIsActive())
        {
            if(SystemClock.uptimeMillis()-lastUpdateTime < 0){
                continue;
            }
            lastUpdateTime = SystemClock.uptimeMillis();
            if(FtcRobotControllerVisionActivity.updateFPV){continue;}

            long startCompose = System.currentTimeMillis();




            for(int i = 0; i < m_telemetry.size_y; i ++){
                telemetry.addLine(m_telemetry.getLine(i));
//                        .replace("4", "█")
//                        .replace("3", "▓")
//                        .replace("2", "▒")
//                        .replace("1", "░")
//                        .replace(" ","╰"));
            }


            long timeCompose = System.currentTimeMillis() - startCompose;

            long startNetwork = System.currentTimeMillis();
            telemetry.update();
            long timeNetwork = System.currentTimeMillis() - startNetwork;

            System.out.println("Compose took " + timeCompose + "ms, network TX took " + timeNetwork + " ms");

            FtcRobotControllerVisionActivity.updateFPV = true;
        }
    }
}
