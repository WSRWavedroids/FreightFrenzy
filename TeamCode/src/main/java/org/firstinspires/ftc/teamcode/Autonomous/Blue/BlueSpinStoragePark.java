package org.firstinspires.ftc.teamcode.Autonomous.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.AutonomousPLUS;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(group = "Blue", name = "Blue Storage Park")
@Disabled
public class BlueSpinStoragePark extends AutonomousPLUS {

    Robot robot = new Robot();


    @Override
    public void runOpMode() {

        super.runOpMode();

        //Do this to pass inspection.
        waitForStart();

        //todo: Test Pseudo Code
        moveRobotRight(50, 2);
        turnDuckSpinner(2);
        moveRobotForward(25, 1);
        robot.stopAllMotors();
    }

}