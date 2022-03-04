


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

        moveRobotBackward(1250);
        prepareNextAction(1000);

        turnDuckSpinnerBlue(3);
        prepareNextAction(3000);

        robot.stopDuckSpinner();
        prepareNextAction(300);

        moveRobotRight(1800);
        prepareNextAction(1000);


        // Robot should now be in storage unit

        robot.openAndCloseClaw(0);
        prepareNextAction(300);

        moveArmAuto("Up", 0.7, 550);
        robot.holdArm("Auto");
        //robot.encoderReset();

        moveRobotForward(1250);
        prepareNextAction(300);

        moveArmAuto("Down", 0.3, 400);
        robot.holdArm("Auto");
        //robot.encoderReset();

        robot.openAndCloseClaw(0.4);
        prepareNextAction(300);

        moveArmAuto("Up", 0.8, 250);
        robot.encoderReset();

        moveRobotBackward(1350);
        prepareNextAction(400);

        moveRobotLeft(775);
        prepareNextAction(400);


    }

}
