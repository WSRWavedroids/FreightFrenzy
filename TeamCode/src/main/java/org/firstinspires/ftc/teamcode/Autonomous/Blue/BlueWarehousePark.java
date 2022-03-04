


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
        moveArmAuto("Up", 0.4, 400);
        robot.holdArm("Auto");

        moveRobotLeft(2500);
        prepareNextAction(1000);
    }

}
