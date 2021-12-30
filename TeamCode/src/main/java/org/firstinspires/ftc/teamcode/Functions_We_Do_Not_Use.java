package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;

public class Functions_We_Do_Not_Use extends Robot {

    Robot robot = new Robot();
    private static int maxSpeed = 1;

    public double inchesToTicks(double inches){
        // returns the inches * ticks per rotation / wheel circ
        return ((inches/12.25) * 537.6 / .5);
    }

    /*
    public int ticksToInches(double inches){
        // With the 96mm wheels, the number below is the ticks per inch. Do not change the number unless you change the wheels.
        return (int) (inches * 30.318997078);
        telemetry.addData("Update", "This function is currently useless");
    }
    */

    public void encoderDriveInches(double inches){
        int ticks = (int) -inchesToTicks(inches);
        backLeftDrive.setTargetPosition(backLeftDrive.getCurrentPosition() + ticks);
        frontLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() + ticks);
        frontRightDrive.setTargetPosition(frontRightDrive.getCurrentPosition() + ticks);
        backRightDrive.setTargetPosition(backRightDrive.getCurrentPosition() + ticks);
    }

    public void encoderCrabwalkInches(double inches){
        //turn the robot a certain amount of inches, + = left hand turn
        int ticks = (int) -inchesToTicks(inches);
        backLeftDrive.setTargetPosition(backLeftDrive.getCurrentPosition() - ticks);
        frontLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() + ticks);
        frontRightDrive.setTargetPosition(frontRightDrive.getCurrentPosition() - ticks);
        backRightDrive.setTargetPosition(backRightDrive.getCurrentPosition() + ticks);
    }
    public void encoderTurnInches(double inches){
        //turn the robot a certain amount of inches, + = left hand turn
        int ticks = (int) -inchesToTicks(inches);
        backLeftDrive.setTargetPosition(backLeftDrive.getCurrentPosition() - ticks);
        frontLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() - ticks);
        frontRightDrive.setTargetPosition(frontRightDrive.getCurrentPosition() + ticks);
        backRightDrive.setTargetPosition(backRightDrive.getCurrentPosition() + ticks);
    }

    public void setTeleMode() {
        telemetry.addData("Mode", "Init for Tele");

        //inits the motors in a way suitible for manual control
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        duckSpinner.setDirection(DcMotor.Direction.FORWARD);
        clawArm.setDirection(DcMotor.Direction.FORWARD);

        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        duckSpinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        duckSpinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        clawArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void setEncoderMode() {
        //Tells the driver hub that we're doing things
        telemetry.addData("Mode", "Init for Encoder");

        //Sets the direction of the motors
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        duckSpinner.setDirection(DcMotor.Direction.FORWARD);

        //Tells the motors to chill when we're not powering them
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Reset encoders to make sure they're not going to do any wacky s***
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        duckSpinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Sets the target position to where it currently is
        backLeftDrive.setTargetPosition(backLeftDrive.getCurrentPosition());
        frontLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition());
        frontRightDrive.setTargetPosition(frontRightDrive.getCurrentPosition());
        backRightDrive.setTargetPosition(backRightDrive.getCurrentPosition());
        duckSpinner.setTargetPosition(duckSpinner.getCurrentPosition());

        //Sets the mode to run to position
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setPIDMode() {
        //Tells the driver hub that we're doing things
        telemetry.addData("Mode", "Init for PID");

        //Sets the direction of the motors
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        duckSpinner.setDirection(DcMotor.Direction.FORWARD);

        //Tells the motors to chill when we're not powering them
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Buckle up, b****es, because we're about to attempt to use SOMETHING COMPLICATED THAT WE DON'T UNDERSTAND!!!
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        duckSpinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public double getHeading(double leftStickX, double leftStickY){
        // inverse tangent, gives the angle of the point
        double theta = Math.atan2(leftStickX, leftStickY);
        return (theta + 3.1415/2);
    }
    public double getMagnitude(double leftStickX, double leftStickY) {
        //pretty much just Pythagorean Theorem, figure out how far point is from center - combined the GetHeading direction and speed can be gotten
        return (Math.sqrt(leftStickY * leftStickY + leftStickX * leftStickX));
    }
    //for use in tele to get heading and speed
    public void drive(double directionInRadians, float turnInRadians, double powerInPercentage) {
        // drives the motors via speed control. Takes a direction in radians (use getHeading()), the turn between -1 and 1, and the power (between 0-1)
        telemetry.addData("Mode", "Driving");

        //takes input in direction, turning, and %of the max speed desired
        double wheelsSetA = Math.sin(directionInRadians - .7957) * powerInPercentage;
        double wheelsSetB = Math.sin(directionInRadians + .7957) * powerInPercentage;
        double motorCheck;
        float turn = turnInRadians * -1;
        //checks if one of the wheel sets is > 100% power, if so reduce it to one, and reduce the other by the same factor
        double[] powers = {wheelsSetA + turn, wheelsSetA - turn, wheelsSetB + turn, wheelsSetB - turn};
        double largestSpeedSoFar = powers[0];

        for (int i = 1; i < 4; i++) {
            if (Math.abs(powers[i]) > largestSpeedSoFar) {
                largestSpeedSoFar = Math.abs(powers[i]);
            }
        }
        motorCheck = maxSpeed / largestSpeedSoFar;
        if(largestSpeedSoFar > 1) {
            for (int h = 0; h < 4; h++) {
                powers[h] = powers[h] * motorCheck;
            }
        }
        this.backLeftDrive.setPower(powers[0]);
        this.frontRightDrive.setPower(powers[1]);
        this.frontLeftDrive.setPower(powers[2]);
        this.backRightDrive.setPower(powers[3]);

        telemetry.addData("back left:", powers[0]);
        telemetry.addData("front right:", powers[1]);
        telemetry.addData("front left:", powers[2]);
        telemetry.addData("back right:", powers[3]);
        telemetry.addData("turning", turnInRadians);
        telemetry.addData("power in percentage", powerInPercentage);
    }

}
