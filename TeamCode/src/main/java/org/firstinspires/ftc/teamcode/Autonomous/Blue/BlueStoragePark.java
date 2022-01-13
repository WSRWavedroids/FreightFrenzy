package org.firstinspires.ftc.teamcode.Autonomous.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Autonomous.AutonomousPLUS;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(group = "Blue", name = "Blue Storage Park")
public class BlueStoragePark extends AutonomousPLUS {

    @Override
    public void runOpMode() {

        super.runOpMode();

        //Do this to pass inspection.
        waitForStart();

        moveRobotRight(300);
        sleep(300);
        robot.encoderReset();

        moveRobotBackward(1200);
        sleep(1000);
        robot.encoderReset();

        turnDuckSpinnerBlue(5);
        sleep(5000);
        robot.encoderReset();

        moveRobotRight(1000);
        sleep(300);
        robot.encoderReset();

        moveRobotBackward(200);
        sleep(200);
        robot.encoderReset();

//        robot.stopAllMotors();


    }
}
