package org.firstinspires.ftc.teamcode.Autonomous.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.AutonomousPLUS;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(group = "Red", name = "Red Storage Park")
    public class RedStoragePark extends AutonomousPLUS {


        @Override
        public void runOpMode() {

            super.runOpMode();
            //Do this to pass inspection.
            waitForStart();

            moveRobotForward(250);
            sleep(300);
            robot.encoderReset();

            moveRobotLeft(1350);
            sleep(1000);
            robot.encoderReset();

            turnDuckSpinnerRed(5);
            sleep(5000);
            robot.encoderReset();

            moveRobotForward(950);
            sleep(300);
            robot.encoderReset();

            moveRobotLeft(400);
            sleep(300);
            robot.encoderReset();

        }
    }
