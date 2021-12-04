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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@TeleOp(name="Lean Mean TeleOp Machine", group="Iterative Opmode")
//@Disabled
public class Lean_Mean_Teleop_Machine extends OpMode {
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
        telemetry.addData("HYPE", "ARE! YOU! READY?!?!?!?!");
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        telemetry.addData("HYPE", "Let's do this!!!");
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        singleJoystickDrive();

        // This little section updates the driver hub on the runtime and the motor powers.
        // It's mostly used for troubleshooting.

        double frontLeftPower = frontLeftDrive.getPower();
        double frontRightPower = frontRightDrive.getPower();
        double backLeftPower = frontRightDrive.getPower();
        double backRightPower = frontRightDrive.getPower();

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "Front Left (%.2f)", frontLeftPower);
        telemetry.addData("Motors", "Front Right (%.2f)", frontRightPower);
        telemetry.addData("Motors", "Back Left (%.2f)", backLeftPower);
        telemetry.addData("Motors", "Back Right (%.2f)", backRightPower);


        // This section checks what bumpers/triggers are being pressed and changes the speed accordingly.

        if (this.gamepad1.right_trigger > 0.5) {
            telemetry.addData("Speed", "Fast Boi");
            speed = 1.0;
        } else if (this.gamepad1.left_trigger > 0.5) {
            telemetry.addData("Speed", "Slow Boi");
            speed = 0.5;
        } else if (this.gamepad1.right_bumper) {
            telemetry.addData("Speed", "Super Slow Boi");
            speed = 0.25;
        } else {
            telemetry.addData("Speed", "Normal Boi");
        }


        // This section checks if the Y or A buttons on the first controller are being pressed and moves the duck spinner.

        if (this.gamepad1.y) {
            telemetry.addData("Duck Spinner", "Clockwise Spin");
           turnOnSpinner(0.65);
        } else if (this.gamepad1.a) {
            telemetry.addData("Duck Spinner", "Counterclockwise Spin");
           turnOnSpinner(-0.65);
        } else {
            telemetry.addData("Duck Spinner", "Not running");
            turnOnSpinner(0);
        }


        // This section checks if the A or B buttons on the second controller are being presses and moves the claw.

        telemetry.addData("ServoPort", "Port: " + claw.getPortNumber());
        if (this.gamepad2.a) {
            claw.getController().setServoPosition(claw.getPortNumber(), 0);
            telemetry.addData("Claw", "Closed");
        } else if (this.gamepad2.b) {
            claw.getController().setServoPosition(claw.getPortNumber(), 0.25);
            telemetry.addData("Claw", "Open");
        }


        // This section finds the position of the left joystick on the second controller and moves the arm.

        if (this.gamepad2.left_stick_y > 0.5) {
            clawArm.setDirection(DcMotor.Direction.FORWARD);
            clawArm.setPower(0.3);
            // This lowers the arm.
        } else if (this.gamepad2.left_stick_y < -0.5) {
            // Contrary to what you might think, because of the positioning of the motor, this actually raises the arm up.
            clawArm.setDirection(DcMotor.Direction.REVERSE);
            clawArm.setPower(1);
        } else {
            clawArm.setDirection(DcMotor.Direction.REVERSE);
            clawArm.setPower(0.1);
            // This keeps the arm held at whatever position it is currently at when not moving it with the joysticks.
            // It will drift down if the arm is holding freight.
        }

    }


        /*
         * Code to run ONCE after the driver hits STOP
         */
        @Override
        public void stop () {

        }


        /*
        * The holding cell for all of the random functions we call above.
         */

        public void turnOnSpinner ( double power){
            // This function tells the duck spinning motor to run. It's pretty simple.
            duckSpinner.setPower(power);
        }

        public void setIndividualPowers ( float[] motorPowers){
            // This function creates an array so that the function below works.
            // Don't mess with this function unless you know what you're doing.

            if (motorPowers.length != 4) {
                return;
            }
            frontLeftDrive.setPower(motorPowers[0]);
            frontRightDrive.setPower(motorPowers[1]);
            backLeftDrive.setPower(motorPowers[2]);
            backRightDrive.setPower(motorPowers[3]);
        }

        private void singleJoystickDrive () {
            // We don't really know how this function works, but it makes the wheels drive, so we don't question it.
            // Don't mess with this function unless you REALLY know what you're doing.

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
            // This function does some math!
            float max = 0;
            for (float val : values) {
                if (Math.abs(val) > max) {
                    max = Math.abs(val);
                }
            }
            return max;
        }

}






