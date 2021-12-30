package org.firstinspires.ftc.teamcode.Autonomous.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.AutonomousPLUS;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(group = "Blue", name = "Blue Park")
public class BluePark extends AutonomousPLUS {

    Robot robot = new Robot();


    @Override
    public void runOpMode() {

        super.runOpMode();

        //Do this to pass inspection.
        waitForStart();

        moveRobotForward(50, 2);
        //robot.stopAllMotors();
        //sleep(300);
        //moveRobotRight(500);
        //robot.stopAllMotors();


    }
}
