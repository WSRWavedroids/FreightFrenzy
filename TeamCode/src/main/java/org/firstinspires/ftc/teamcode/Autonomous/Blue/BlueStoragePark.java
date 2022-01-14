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
        prepareNextAction(300);

        moveRobotBackward(1200);
        prepareNextAction(1000);

        turnDuckSpinnerBlue(5);
        prepareNextAction(5000);

        moveRobotRight(1000);
        prepareNextAction(300);

        moveRobotBackward(200);
        prepareNextAction(200);

//        robot.stopAllMotors();


    }
}
