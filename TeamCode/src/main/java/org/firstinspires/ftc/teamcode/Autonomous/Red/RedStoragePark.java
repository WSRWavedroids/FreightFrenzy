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
            prepareNextAction(300);

            moveRobotLeft(1350);
            prepareNextAction(1000);

            turnDuckSpinnerRed(5);
            prepareNextAction(5000);

            moveRobotForward(950);
            prepareNextAction(300);

            moveRobotLeft(200);
            prepareNextAction(200);

        }
    }
