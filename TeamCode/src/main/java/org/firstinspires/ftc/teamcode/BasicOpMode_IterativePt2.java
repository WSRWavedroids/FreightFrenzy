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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")
@Disabled
public class BasicOpMode_IterativePt2 extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor frontLeftDrive;
    public DcMotor frontRightDrive;
    public DcMotor backLeftDrive;
    public DcMotor backRightDrive;
    public DcMotor duckSpinner;
    public DcMotor clawArm;
    public CRServo claw;

    private double speed = 0.75;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        //void init(Telemetry telemetry, DcMotor frontLeftDrive, DcMotor frontRightDrive, DcMotor backLeftDrive, DcMotor backRightDrive)


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");
        duckSpinner = hardwareMap.get(DcMotor.class, "duckSpinner");
        clawArm = hardwareMap.get(DcMotor.class, "clawArm");
        claw = hardwareMap.get(CRServo.class, "claw");


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


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        //  leftDrive.setDirection(DcMotor.Direction.FORWARD);
        // rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        singleJoystickDrive();

        if (this.gamepad1.right_trigger > 0.5) {
            speed = 1.0;
        } else if (this.gamepad1.left_trigger > 0.5) {
            speed = 0.5;
        } else if (this.gamepad1.right_bumper) {
            speed = 0.25;
        }

        if (this.gamepad1.y) {
           // turnOnSpinner(0.65);
            duckSpinner.setPower(0.65);
        } else if (this.gamepad1.a) {
           // turnOnSpinner(-0.65);
            duckSpinner.setPower(-0.65);
        } else {
            //turnOnSpinner(0);
            duckSpinner.setPower(0);
        }


        telemetry.addData("ServoPort", "Port: " + claw.getPortNumber());
        if (this.gamepad2.a) {
            claw.getController().setServoPosition(claw.getPortNumber(), 0);
        } else if (this.gamepad2.b) {
            claw.getController().setServoPosition(claw.getPortNumber(), 0.25);
        }

        if (this.gamepad2.left_stick_y > 0.5) {
            clawArm.setDirection(DcMotor.Direction.FORWARD);
            clawArm.setPower(0.3);
        } else if (this.gamepad2.left_stick_y < -0.5) {
            clawArm.setDirection(DcMotor.Direction.REVERSE);
            clawArm.setPower(1);
        } else {
            clawArm.setDirection(DcMotor.Direction.REVERSE);
            clawArm.setPower(0.1);
            //setClawArmPower(true);
        }

/*
            // 1 below
            //tennisArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            clawArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //clawArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // Don't remember if next line is needed.  I believe setTargetPosition is inited to zero so probably not needed
            //clawArm.setTargetPosition(0);
            // 2 below
            clawArm.setPower(0.65); //set to the max speed you want the arm to move at
        }

        public void setClawArmPower(boolean up)
        {
            if(up)
            {
                clawArm.setTargetPosition(clawArm.getCurrentPosition()+200);
                clawArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // 3 below
                //tennisArm.setPower(0.7);
            } else {
                clawArm.setTargetPosition(clawArm.getCurrentPosition()-200);
                clawArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // 4 below
                //tennisArm.setPower(-0.7);
            }
        }
    /*
    }*/


        // Setup a variable for each drive wheel to save power level for telemetry
        //double leftPower;
        //double rightPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        //double drive = -gamepad1.left_stick_y;
        //double turn  =  gamepad1.right_stick_x;
        //leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        //rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheels
        //leftDrive.setPower(leftPower);
        //rightDrive.setPower(rightPower);

        // Show the elapsed game time and wheel power.
        //telemetry.addData("Status", "Run Time: " + runtime.toString());
        //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);

    }
        private void setServoPosition (CRServo claw,double v){
        }

        /*
         * Code to run ONCE after the driver hits STOP
         */
        @Override
        public void stop () {

        }

        public void turnOnSpinner ( double power){
            duckSpinner.setPower(power);
        }

        public void setIndividualPowers ( float[] motorPowers){
            if (motorPowers.length != 4) {
                return;
            }
            frontLeftDrive.setPower(motorPowers[0]);
            frontRightDrive.setPower(motorPowers[1]);
            backLeftDrive.setPower(motorPowers[2]);
            backRightDrive.setPower(motorPowers[3]);
        }

        private void singleJoystickDrive () {
            float leftX = this.gamepad1.left_stick_x;
            float leftY = this.gamepad1.left_stick_y;
            float rightX = this.gamepad1.right_stick_x;

            float[] motorPowers = new float[4];
            motorPowers[0] = (leftY - leftX - rightX);
            motorPowers[1] = (leftY + leftX + rightX);
            motorPowers[2] = (leftY + leftX - rightX);
            motorPowers[3] = (leftY - leftX + rightX);

            float max = getLargestAbsVal(motorPowers);
            if (max < 1) {
                max = 1;
            }

            for (int i = 0; i < motorPowers.length; i++) {
                motorPowers[i] *= (speed / max);

                float abs = Math.abs(motorPowers[i]);
                if (abs < 0.05) {
                    motorPowers[i] = 0.0f;
                }
                if (abs > 1.0) {
                    motorPowers[i] /= abs;
                }
            }

            setIndividualPowers(motorPowers);
        }

        private float getLargestAbsVal ( float[] values){
            float max = 0;
            for (float val : values) {
                if (Math.abs(val) > max) {
                    max = Math.abs(val);
                }
            }
            return max;
        }

}






