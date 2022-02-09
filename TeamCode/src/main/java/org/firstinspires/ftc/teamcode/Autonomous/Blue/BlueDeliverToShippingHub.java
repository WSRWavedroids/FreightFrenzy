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
        prepareNextAction(200);

        moveArmAuto("Up", 0.8, 500);
        robot.holdArm("Auto");
        //robot.encoderReset();

        moveRobotForward(1150);
        prepareNextAction(400);

        moveArmAuto("Down", 0.3, 350);
        robot.encoderReset();

        robot.openAndCloseClaw(0.4);
        prepareNextAction(200);

        moveArmAuto("Up", 0.8, 500);
        //robot.holdArm("Auto");
        robot.encoderReset();

        moveRobotBackward(1200);
        prepareNextAction(400);

        moveRobotLeft(3000);
        prepareNextAction(500);

        }

    }
