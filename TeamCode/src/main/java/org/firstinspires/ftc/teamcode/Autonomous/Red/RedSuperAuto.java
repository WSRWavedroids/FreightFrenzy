


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

        moveArmAuto("Up", 0.8, 600);
        robot.holdArm("Auto");
        //robot.encoderReset();

        moveRobotForward(1225);
        prepareNextAction(300);

        moveArmAuto("Down", 0.3, 400);
        // robot.holdArm("Auto");
        robot.encoderReset();

        robot.openAndCloseClaw(0.4);
        prepareNextAction(300);

        moveArmAuto("Up", 0.8, 300);
        robot.holdArm("Auto");
        robot.encoderReset();

        moveRobotBackward(500);
        prepareNextAction(1000);

        moveRobotLeft(2450);
        prepareNextAction(1000);

        moveRobotBackward(400);
        prepareNextAction(500);

        turnDuckSpinnerRed(6);
        prepareNextAction(2500);

        moveRobotForward(900);
        prepareNextAction(300);

        // Robot should now be parallel to red shipping hub


    }

}
