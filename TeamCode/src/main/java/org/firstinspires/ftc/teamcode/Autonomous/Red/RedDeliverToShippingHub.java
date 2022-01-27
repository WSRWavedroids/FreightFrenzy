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

        robot.moveArm("Up", 0.6);
        prepareNextAction(300);

        moveRobotForward(1125);
        prepareNextAction(400);

        robot.moveArm("Down", 0.4);
        prepareNextAction(900);

        robot.openAndCloseClaw(0.4);
        prepareNextAction(300);

        robot.moveArm("Up", 0.5);
        prepareNextAction(1000);

        moveRobotBackward(1150);
        prepareNextAction(900);

        moveRobotRight(3000);
        prepareNextAction(600);


    }

    }
