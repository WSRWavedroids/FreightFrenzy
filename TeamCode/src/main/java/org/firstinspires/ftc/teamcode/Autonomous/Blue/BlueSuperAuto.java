


package org.firstinspires.ftc.teamcode.Autonomous.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Autonomous.AutonomousPLUS;

@Autonomous(group = "Blue", name = "Blue Super Auto")

public class BlueSuperAuto extends AutonomousPLUS {

    @Override
    public void runOpMode() {

        super.runOpMode();

        //Do this to pass inspection.
        waitForStart();

        moveRobotRight(300);
        prepareNextAction(300);

        moveRobotBackward(1100);
        prepareNextAction(1000);

        turnDuckSpinnerBlue(3);
        prepareNextAction(3000);

        moveRobotRight(1700);
        prepareNextAction(1000);

        stopDuckSpinner(3);
        prepareNextAction(300);


        // Robot should now be in storage unit

        robot.openAndCloseClaw(0);
        prepareNextAction(500);

        robot.moveArm("Up", 0.45);
        sleep(300);

        moveRobotForward(1200);
        prepareNextAction(300);

        robot.moveArm("Down", 0.4);
        sleep(1500);

        robot.openAndCloseClaw(0.4);
        prepareNextAction(1000);

        robot.moveArm("Up", 0.5);
        sleep(2000);

        moveRobotBackward(1250);
        prepareNextAction(400);

        moveRobotLeft(600);
        prepareNextAction(400);


    }

}
