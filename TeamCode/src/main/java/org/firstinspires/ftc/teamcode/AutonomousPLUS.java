package org.firstinspires.ftc.teamcode;/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is the autonomous mode. It moves the robot withou us having to touch the controller. Previous programmers erally sucked
 */

@Autonomous(name="Auto", group="Pushbot")
@Disabled
public class AutonomousPLUS extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor frontLeftDrive;
    public DcMotor frontRightDrive;
    public DcMotor backLeftDrive;
    public DcMotor backRightDrive;
    public DcMotor duckSpinner;

    private double speed = 0.75;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        //void init(Telemetry telemetry, DcMotor frontLeftDrive, DcMotor frontRightDrive, DcMotor backLeftDrive, DcMotor backRightDrive)
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontRightDrive  = hardwareMap.get(DcMotor.class, "frontRightDrive");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");
        duckSpinner = hardwareMap.get(DcMotor.class, "duckSpinner");
        // this.frontLeftDrive = frontLeftDrive;
        // this.frontRightDrive = frontRightDrive;
        // this.backLeftDrive = backLeftDrive;
        // this.backRightDrive = backRightDrive;

        //frontLeftDrive.setTargetPosition(0);
        //frontRightDrive.setTargetPosition(0);
        //backLeftDrive.setTargetPosition(0);
        //backRightDrive.setTargetPosition(0);

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        duckSpinner.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        //  leftDrive.setDirection(DcMotor.Direction.FORWARD);
        // rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        //Do this to pass inspection.
        waitForStart();

        moveRobotForwardTicks(inchesToTicks(12.));

        while (isWheelsBusy()){}
        sleep(200);
        turnRobotTicks(inchesToTicks(5.));

    }
    public boolean isWheelsBusy(){
        return backLeftDrive.isBusy() || frontLeftDrive.isBusy() || frontRightDrive.isBusy() || backRightDrive.isBusy();
    }

    public int inchesToTicks(double inches){
        // returns the inches * ticks per rotation / wheel circ
        return (int) ((inches/12.25) * 537.6 / .5);

    }

    private void moveRobotForwardTicks(int ticks){
        frontLeftDrive.setTargetPosition(ticks + frontLeftDrive.getCurrentPosition());
        frontRightDrive.setTargetPosition(ticks + frontRightDrive.getCurrentPosition());
        backLeftDrive.setTargetPosition(ticks + backLeftDrive.getCurrentPosition());
        backRightDrive.setTargetPosition(ticks + backRightDrive.getCurrentPosition());
    }

    public void turnRobotTicks(int ticks){
        //turn the robot a certain amount of inches, + = left hand turn
        backLeftDrive.setTargetPosition(backLeftDrive.getCurrentPosition() - ticks);
        frontLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() - ticks);
        frontRightDrive.setTargetPosition(frontRightDrive.getCurrentPosition() + ticks);
        backRightDrive.setTargetPosition(backRightDrive.getCurrentPosition() + ticks);
    }
}
