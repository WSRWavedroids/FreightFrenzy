package org.firstinspires.ftc.teamcode.Autonomous;
/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Autonomous.DetectionCalculation;


/**
 * This is the autonomous mode. It moves the robot without us having to touch the controller.
 * Previous programmers really sucked at explaining what any of this meant, so we're trying to do better.
 *
 */

//todo Create function to convert tick values based on speed.
public abstract class AutonomousPLUS extends LinearOpMode {

    // This section tells the program all of the different pieces of hardware that are on our robot that we will use in the program.
    private ElapsedTime runtime = new ElapsedTime();

    public double speed = 0.5;
    public String level;

    //DO NOT DELETE THIS LINE! CAPITALIZATION IS VERY IMPORTANT!!!
    public Robot robot = new Robot();
    ObjectDetectorThingy objectDetectorThingy = null;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry, this);
        robot.encoderReset();
        robot.encoderRunningMode();

    }


    public void moveRobotForward(int ticks) {
        if (opModeIsActive()){
            //ticks = (int) (ticks * robot.multi);
            robot.setTargets("Forward", ticks);
            robot.positionRunningMode();
            }
            robot.powerSet(speed);

            while (opModeIsActive() &&
                    robot.isWheelsBusy()) {
                robot.tellMotorOutput();
                //nothings here
            }

            robot.stopAllMotors();
            robot.encoderRunningMode();
            robot.stopAllMotors();

        }


    public void moveRobotBackward(int ticks){
        if (opModeIsActive()){
            //ticks = (int) (ticks * robot.multi);
            robot.setTargets("Backward", ticks);
            robot.positionRunningMode();
            robot.powerSet(speed);

            while (opModeIsActive() &&
                    robot.isWheelsBusy()) {
                robot.tellMotorOutput();
                //nothings here
            }

            robot.stopAllMotors();
            robot.encoderRunningMode();


        }

    }
    public void moveRobotLeft(int ticks) {

        if (opModeIsActive()){
            //ticks = (int) (ticks * robot.multi);
            robot.setTargets("Left", ticks);
            robot.positionRunningMode();
            robot.powerSet(speed);

            while (opModeIsActive() &&
                    robot.isWheelsBusy()) {
                robot.tellMotorOutput();
                //nothings here
            }

            robot.stopAllMotors();
            robot.encoderRunningMode();


        }
    }

    public void moveRobotRight(int ticks) {

        if (opModeIsActive()) {
            //ticks = (int) (ticks * robot.multi);
            robot.setTargets("Right", ticks);
            robot.positionRunningMode();
            robot.powerSet(speed);

            while (opModeIsActive() &&
                    robot.isWheelsBusy()) {
                robot.tellMotorOutput();
                //nothings here
            }

            robot.stopAllMotors();
            robot.encoderRunningMode();


        }
    }

    public void turnRobotRight(int ticks) {

        if (opModeIsActive()) {
            robot.setTargets("Right", ticks);
            robot.positionRunningMode();
            robot.powerSet(speed);

            while (opModeIsActive() &&
                    robot.isWheelsBusy()) {
                robot.tellMotorOutput();
                //nothings here
            }

            robot.stopAllMotors();
            robot.encoderRunningMode();


        }
    }


    public void turnDuckSpinnerBlue(double maxSeconds){
        robot.duckSpinner.setPower(0.65);

        while (opModeIsActive() && getRuntime() < maxSeconds) {
            robot.tellMotorOutput();
            //nothings here
        }

        robot.stopAllMotors();

    }

    public void turnDuckSpinnerRed(double maxSeconds){
        robot.duckSpinner.setPower(-0.65);

        while (opModeIsActive() && getRuntime() < maxSeconds) {
            robot.tellMotorOutput();
            //nothings here
        }

        robot.stopAllMotors();

    }

    public void prepareNextAction(long pause){
        sleep(pause);
        robot.encoderReset();
    }

    public void moveArmAuto(String direction, double power, int ticks) {
        if (opModeIsActive()) {

            if (direction == "Up") {
                robot.clawArm.setDirection(DcMotor.Direction.REVERSE);
                robot.clawArm.setTargetPosition(robot.clawArm.getCurrentPosition() + ticks);
            } else if (direction == "Down") {
                robot.clawArm.setDirection(DcMotor.Direction.FORWARD);
                robot.clawArm.setTargetPosition(robot.clawArm.getCurrentPosition() + ticks);
            }

            robot.clawArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.clawArm.setPower(power);

            while (opModeIsActive() &&
                    (robot.clawArm.getTargetPosition() != robot.clawArm.getCurrentPosition())) {
                robot.tellMotorOutput();
                //nothings here
            }

            robot.clawArm.setPower(0);
            robot.clawArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void setMaxFrameSize(int maxWidth, int maxHeight) {
    }

   /* public void findTargetLevel() {
        if (objectDetectorThingy.getPosition() == DetectionCalculation.CapstonePosition.LEFT) {
            level = "Bottom";
        } else if (objectDetectorThingy.getPosition() == DetectionCalculation.CapstonePosition.MIDDLE){
            level = "Middle";
        } else if (objectDetectorThingy.getPosition() == DetectionCalculation.CapstonePosition.RIGHT){
            level = "Top";
        }
    }

    */

}