package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.OpMode;

        import org.firstinspires.ftc.teamcode.Autonomous.DetectionCalculation;
        import org.firstinspires.ftc.teamcode.Autonomous.ObjectDetectorThingy;


/**
 * Created by shell bots on 9/26/2020, copied by Wave Droids on 2/14/22
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "Manual", name = "Camera Test Manual")
public class TeleOpCameraTest extends OpMode {

    private ObjectDetectorThingy objectDetectorThingy = null;

    @Override
    public void init() {
        objectDetectorThingy = new ObjectDetectorThingy(hardwareMap);
        //this.msStuckDetectStop = 60000;

        // Step 0 - Initialized
        telemetry.addData("Initialized", 4);
        objectDetectorThingy.start();
    }

    @Override
    public void loop() {
        DetectionCalculation.CapstonePosition rp = objectDetectorThingy.getPosition();
        int analysis1 = objectDetectorThingy.getAnalysis();
        telemetry.addData("13206: Capstone Position", rp.toString() + " " + String.valueOf(analysis1));
        telemetry.update();
    }

    /**
     * Runs once after STOP is pushed
     */
    @Override
    public void stop() {
        objectDetectorThingy.save();
        objectDetectorThingy.end();
        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }
}