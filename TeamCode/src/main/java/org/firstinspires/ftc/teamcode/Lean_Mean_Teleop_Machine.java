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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file is our iterative (Non-Linear) "OpMode" for TeleOp.
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is selected on the Robot Controller and executed.
 * This particular one is called "Lean Mean TeleOp Machine". I had a little too much fun with naming this.
 *
 * This OpMode controls the functions of the robot during the driver-controlled period.
 *
 * If the "@Disabled" line is not commented out, the program will not show up on the driver hub.
 * If you ever have problems with the program not showing up on the driver hub, it's probably because of that.
 *
 * Throughout this program, there are comments explaining what everything does because previous programmers
 * did a horrible job of doing that.
 */

@TeleOp(name="Lean Mean TeleOp Machine", group="Iterative Opmode")
public class Lean_Mean_Teleop_Machine extends OpMode {

    // This section tells the program all of the different pieces of hardware that are on our robot that we will use in the program.
    private ElapsedTime runtime = new ElapsedTime();
    private double speed = 0.75;
    public Robot robot = new Robot();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // Call the initialization protocol from the Robot class.
        robot.init(hardwareMap, telemetry, this);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {telemetry.addData("HYPE", "ARE! YOU! READY?!?!?!?!");}

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
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        robot.tellMotorOutput();


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
        } else if (this.gamepad1.left_bumper) {
            telemetry.addData("Speed", "Normal Boi");
            speed = 0.75;
        }


        // This section checks if the Y or A buttons on the first controller are being pressed and moves the duck spinner.

        if (this.gamepad1.y) {
            telemetry.addData("Duck Spinner", "Clockwise Spin");
            robot.turnDuckSpinner(0.65);

        } else if (this.gamepad1.a) {
            telemetry.addData("Duck Spinner", "Counterclockwise Spin");
            robot.turnDuckSpinner(-0.65);
        } else {
            telemetry.addData("Duck Spinner", "Not running");
            robot.stopDuckSpinner();
        }


        // This section checks if the A or B buttons on the second controller are being presses and moves the claw.

        telemetry.addData("ServoPort", "Port: " + robot.claw.getPortNumber());
        if (this.gamepad2.a) {
            robot.openAndCloseClaw(0);
        } else if (this.gamepad2.b) {
            robot.openAndCloseClaw(.4);
        }

        // This section checks what position the claw servo is in and updates the driver hub accordingly.

        if (robot.claw.getController().getServoPosition(robot.claw.getPortNumber()) == 0){
            telemetry.addData("Claw", "Closed");
        } else if (robot.claw.getController().getServoPosition(robot.claw.getPortNumber()) == 0.4){
            telemetry.addData("Claw", "Open");
        }


        // This section finds the position of the left joystick on the second controller and moves the arm.

        if (this.gamepad2.left_stick_y > 0.5) {

            // This lowers the arm.
            robot.moveArm("Down", 0.3);

            telemetry.addData("Arm", "Lowering Arm");

        } else if (this.gamepad2.left_stick_y < -0.5) {

            // Contrary to what you might think, because of the positioning of the motor, this actually raises the arm up.
            robot.moveArm("Up", 0.8);
            telemetry.addData("Arm", "Raising Arm");

        } else {

            // This keeps the arm held at whatever position it is currently at when not moving it with the joysticks.
            robot.holdArm();

        }

    }


        /*
         * Code to run ONCE after the driver hits STOP
         */
        @Override
        public void stop () { telemetry.addData("Status", "Robot Stopped"); }


        /*
        * The holding cell for all of the random functions we call above.
         */

        public void setIndividualPowers ( float[] motorPowers){
            // This function creates an array so that the function below works.
            // Don't mess with this function unless you know what you're doing.

            if (motorPowers.length != 4) {
                return;
            }
            robot.frontLeftDrive.setPower(motorPowers[0]);
            robot.frontRightDrive.setPower(motorPowers[1]);
            robot.backLeftDrive.setPower(motorPowers[2]);
            robot.backRightDrive.setPower(motorPowers[3]);
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