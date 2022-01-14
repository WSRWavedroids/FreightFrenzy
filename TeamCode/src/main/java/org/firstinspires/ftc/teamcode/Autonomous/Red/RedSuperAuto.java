


package org.firstinspires.ftc.teamcode.Autonomous.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.AutonomousPLUS;

@Autonomous(group = "Red", name = "Red Super Auto")

public class RedSuperAuto extends AutonomousPLUS {

    @Override
    public void runOpMode() {

        super.runOpMode();

        //Do this to pass inspection.
        waitForStart();

        robot.openAndCloseClaw(0);
        prepareNextAction(300);

        robot.moveArm("Up", 0.5);
        sleep(1000);

        moveRobotForward(650);
        prepareNextAction(300);


        robot.moveArm("Down", 0.4);
        sleep(1500);

        robot.openAndCloseClaw(0.4);
        prepareNextAction(700);

        robot.moveArm("Up", 0.5);
        sleep(1000);

        moveRobotLeft(2550);
        prepareNextAction(1000);

        moveRobotBackward(150);
        prepareNextAction(500);

        turnDuckSpinnerRed(6);
        prepareNextAction(2500);

        moveRobotForward(950);
        prepareNextAction(300);

        // Robot should now be parallel to red shipping hub


    }

}
