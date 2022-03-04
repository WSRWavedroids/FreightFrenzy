


package org.firstinspires.ftc.teamcode.Autonomous.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Autonomous.AutonomousPLUS;

@Autonomous(group = "Red", name = "Red Warehouse Park")

public class RedWarehousePark extends AutonomousPLUS {

    @Override
    public void runOpMode() {

        super.runOpMode();

        //Do this to pass inspection.
        waitForStart();
        moveArmAuto("Up", 0.4, 400);
        robot.holdArm("Auto");

        moveRobotRight(2500);
        prepareNextAction(1000);
    }

}
