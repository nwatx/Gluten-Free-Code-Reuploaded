package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevExtensions2;

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
public class BlankTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Runnable mainThread;
    private ExpansionHubEx revMaster;
    private ExpansionHubEx revSlave;

    private boolean isRunning = true;
    public long totalUpdates = 0;


    double startSampleTime = 0;
    @Override
    public void runOpMode() {
        RevExtensions2.init();


        telemetry.setAutoClear(true);
        revMaster = hardwareMap.get(ExpansionHubEx.class,"Expansion Hub 4");
        revSlave = hardwareMap.get(ExpansionHubEx.class,"Expansion Hub 2");

        //setup this separate thread
        mainThread = new Runnable() {
            @Override
            public void run() {
                while(isRunning){
                    totalUpdates ++;
                    revMaster.getBulkInputData();
                    revSlave.getBulkInputData();
                }
            }
        };

        waitForStart();
        //run this on another thread
        new Thread(mainThread).start();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            telemetry.addData("Millis per update",
                    (runtime.milliseconds()-startSampleTime)/totalUpdates);
            if(runtime.milliseconds()-startSampleTime > 5000){
                startSampleTime = runtime.milliseconds();
                totalUpdates = 0;
            }
            telemetry.update();

        }
        isRunning = false;
    }

}
