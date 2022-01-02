package org.firstinspires.ftc.teamcode.Autonomous.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.AutonomousPLUS;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(group = "Red", name = "Red Storage Park")
public class RedSpinStoragePark extends AutonomousPLUS {

        Robot robot = new Robot();


        @Override
        public void runOpMode() {

            super.runOpMode();

            //Do this to pass inspection.
            waitForStart();

            //todo: Test Pseudo Code
            moveRobotLeft(50, 2);
            turnDuckSpinner(2);
            moveRobotForward(25, 1);
            robot.stopAllMotors();


    }
}


