package org.firstinspires.ftc.teamcode.Autonomous.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Autonomous.AutonomousPLUS;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(group = "Blue", name = "Blue Deliver To Shipping Hub")
public class BlueDeliverToShippingHub extends AutonomousPLUS {

    @Override
    public void runOpMode() {

        super.runOpMode();

        //Do this to pass inspection.
        waitForStart();

        robot.openAndCloseClaw(0);
        prepareNextAction(500);

        robot.moveArm("Up", 0.45);
        sleep(300);

        moveRobotForward(1100);
        prepareNextAction(400);

        robot.moveArm("Down", 0.4);
        sleep(1500);

        robot.openAndCloseClaw(0.4);
        prepareNextAction(500);

        robot.moveArm("Up", 0.5);
        sleep(1000);

        moveRobotBackward(1200);
        prepareNextAction(400);

        moveRobotLeft(3000);
        prepareNextAction(500);

        }

    }
