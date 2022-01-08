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

        robot.clawArm.setDirection(DcMotor.Direction.REVERSE);
            robot.clawArm.setPower(0.4);
            sleep(1000);

            moveRobotForward(300);
            sleep(500);
            robot.encoderReset();

            robot.clawArm.setDirection(DcMotor.Direction.FORWARD);
            robot.clawArm.setPower(0.8);
            sleep(1000);

            robot.openAndCloseClaw(.4);

        }

    }
