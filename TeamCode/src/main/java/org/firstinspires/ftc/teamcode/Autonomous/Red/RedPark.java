package org.firstinspires.ftc.teamcode.Autonomous.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.AutonomousPLUS;

    @Autonomous(group = "Red", name = "Red Park")
    public class RedPark extends AutonomousPLUS {

        @Override
        public void runOpMode() {

            super.runOpMode();
            //Do this to pass inspection.
            waitForStart();

            moveRobotLeft(3000);
            sleep(300);
            moveRobotForward(3000);

        }
    }
