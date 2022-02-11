package org.firstinspires.ftc.teamcode.Autonomous.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Autonomous.AutonomousPLUS;

@Autonomous(group = "Red", name = "Red Deliver To Shipping Hub")
public class RedDeliverToShippingHub extends AutonomousPLUS {

    @Override
    public void runOpMode() {

        super.runOpMode();

        //Do this to pass inspection.
        waitForStart();

        robot.openAndCloseClaw(0);
        prepareNextAction(500);

        moveArmAuto("Up", 0.8, 600);
        robot.holdArm("Auto");
        //robot.encoderReset();

        moveRobotForward(1150);
        prepareNextAction(400);

        moveArmAuto("Down", 0.3, 400);
        // robot.holdArm("Auto");
        robot.encoderReset();

        robot.openAndCloseClaw(0.4);
        prepareNextAction(300);

        moveArmAuto("Up", 0.8, 200);
        robot.holdArm("Auto");
        robot.encoderReset();

        moveRobotBackward(1225);
        prepareNextAction(900);

        moveRobotRight(3000);
        prepareNextAction(600);


    }

    }