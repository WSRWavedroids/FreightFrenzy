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

        waitForStart();

        robot.openAndCloseClaw(0);
        sleep(500);
        robot.encoderReset();

        robot.clawArm.setDirection(DcMotor.Direction.REVERSE);
        robot.clawArm.setPower(0.45);
        sleep(300);

        moveRobotForward(600);
        sleep(400);
        robot.encoderReset();

        robot.clawArm.setDirection(DcMotor.Direction.FORWARD);
        robot.clawArm.setPower(0.4);
        sleep(1500);

        robot.openAndCloseClaw(0.4);
        sleep(500);
        robot.encoderReset();

        robot.clawArm.setDirection(DcMotor.Direction.REVERSE);
        robot.clawArm.setPower(0.5);
        sleep(1000);

        moveRobotBackward(600);
        sleep(700);
        robot.encoderReset();

        moveRobotRight(3000);
        sleep(500);

        robot.encoderReset();


    }

    }
