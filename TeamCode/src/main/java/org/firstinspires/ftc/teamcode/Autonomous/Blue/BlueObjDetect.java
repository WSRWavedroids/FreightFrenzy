package org.firstinspires.ftc.teamcode.Autonomous.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Autonomous.AutonomousPLUS;
import org.firstinspires.ftc.teamcode.Autonomous.DetectionCalculation;
import org.firstinspires.ftc.teamcode.Autonomous.ObjectDetectorThingy;
import org.firstinspires.ftc.teamcode.Robot;

import java.io.ObjectStreamException;

@Autonomous(group = "Blue", name = "Blue Object Detection")
public class BlueObjDetect extends AutonomousPLUS {

    @Override
    public void runOpMode() {


        super.runOpMode();

        //Do this to pass inspection.
        waitForStart();

        ObjectDetectorThingy objectDetectorThingy = new ObjectDetectorThingy(hardwareMap);
        DetectionCalculation.CapstonePosition pos = objectDetectorThingy.getPosition();

        robot.openAndCloseClaw(0);
        prepareNextAction(500);

        if (pos == DetectionCalculation.CapstonePosition.LEFT) {
            //moveArmAuto("Up", 0.8, 100);
            //robot.holdArm("Auto");
            moveRobotLeft(200);
        } else if (pos == DetectionCalculation.CapstonePosition.MIDDLE){
            //moveArmAuto("Up", 0.5, 300);
            //robot.holdArm("Auto");
            moveRobotForward(200);
        } else if (pos == DetectionCalculation.CapstonePosition.RIGHT){
            //moveArmAuto("Up", 0, 500);
            //robot.holdArm("Auto");
            moveRobotRight(200);
        }

        moveRobotForward(1150);
        prepareNextAction(400);


        //moveArmAuto("Up", 0.8, 500);
        //robot.holdArm("Auto");
        //robot.encoderReset();

       /* moveRobotForward(1150);
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

        */

        }

    }
