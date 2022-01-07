


package org.firstinspires.ftc.teamcode.Autonomous.Blue;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.hardware.DcMotor;

        import org.firstinspires.ftc.teamcode.Autonomous.AutonomousPLUS;
        import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(group = "Blue", name = "Blue Warehouse Park")

public class BlueWarehousePark extends AutonomousPLUS {

    @Override
    public void runOpMode() {

        super.runOpMode();

        //Do this to pass inspection.
        waitForStart();
        robot.clawArm.setDirection(DcMotor.Direction.REVERSE);
        robot.clawArm.setPower(0.4);
        sleep(1000);

        moveRobotLeft(2000);
        sleep(300);
        robot.encoderReset();
    }

}
