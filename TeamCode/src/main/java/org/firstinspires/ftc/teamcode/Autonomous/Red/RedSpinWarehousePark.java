package org.firstinspires.ftc.teamcode.Autonomous.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.AutonomousPLUS;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(group = "Red", name = "Red Warehouse Park")
@Disabled
public class RedSpinWarehousePark extends AutonomousPLUS {

    Robot robot = new Robot();


    @Override
    public void runOpMode() {

        super.runOpMode();

        //Do this to pass inspection.
        waitForStart();

        //todo: Test Pseudo Code
        moveRobotLeft(50, 2);
        turnDuckSpinner(2);
        moveRobotRight(100, 4);
        robot.stopAllMotors();
    }

}
