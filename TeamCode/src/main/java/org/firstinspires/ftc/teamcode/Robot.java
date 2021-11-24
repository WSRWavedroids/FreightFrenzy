package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.tensorflow.lite.Interpreter;

import java.io.File;

public class Robot{
    //  public Vector2d position;

    public DcMotor frontLeftDrive;
    public  final double nintydegreeturninches = 4 * 3.1415;
    public DcMotor frontRightDrive;
    public DcMotor backLeftDrive;
    public DcMotor backRightDrive;
    public DcMotor duckSpinner;
    public ColorSensor Color;
    private final double wheelSizeMM = 100;
    public Telemetry telemetry;


    //init and declare var
    private static int maxSpeed = 1;
    //construct robot
    public Robot (DcMotor duckSpinner, DcMotor frontLeftDrive, DcMotor frontRightDrive, DcMotor backLeftDrive, DcMotor backRightDrive, Telemetry T){
        this.frontLeftDrive = frontLeftDrive;
        this.frontRightDrive = frontRightDrive;
        this.backLeftDrive = backLeftDrive;
        this.backRightDrive = backRightDrive;
        this.duckSpinner = duckSpinner;
        this.telemetry = T;
    }

    public double inchesToTicks(double inches){
        // returns the inches * ticks per rotation / wheel circ
        return ((inches/12.25) * 537.6 / .5);

    }
    public void encoderDriveInches(double inches){
        int ticks = (int) -inchesToTicks(inches);
        backLeftDrive.setTargetPosition(backLeftDrive.getCurrentPosition() + ticks);
        frontLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() + ticks);
        frontRightDrive.setTargetPosition(frontRightDrive.getCurrentPosition() + ticks);
        backRightDrive.setTargetPosition(backRightDrive.getCurrentPosition() + ticks);
    }
    public boolean isWheelsBusy(){
        if(backLeftDrive.isBusy() ||frontLeftDrive.isBusy() || frontRightDrive.isBusy() || backRightDrive.isBusy()){
            return true;
        }
        else{
            return false;
        }
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

    public void turnDuckSpinner(){
        duckSpinner.setPower(0.6);
        telemetry.addData("Ducks", "Whee!");
    }
    public void stopDuckSpinner(){
        duckSpinner.setPower(0);
    }
    public void setTeleMode() {
        telemetry.addData("Mode", "Init for Tele");

        //inits the motors in a way suitible for manual control
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        duckSpinner.setTargetPosition(duckSpinner.getCurrentPosition());

        duckSpinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        duckSpinner.setDirection(DcMotor.Direction.FORWARD);

    }

    public void setWheelPower(double power){
        // sets the 4 drive trains to a certain power between 0 - 1
        backLeftDrive.setPower(power);
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        backRightDrive.setPower(power);
    }
    public void setEncoderMode() {
        //sets the drivetrain in a drive to position mode
        telemetry.addData("Mode", "Init for Encoder");

        //inits the motors in a way suitible for manual control
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //stop and resest enc
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //set pos to current pos
        backLeftDrive.setTargetPosition(backLeftDrive.getCurrentPosition());
        frontLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition());
        frontRightDrive.setTargetPosition(frontRightDrive.getCurrentPosition());
        backRightDrive.setTargetPosition(backRightDrive.getCurrentPosition());

        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        duckSpinner.setTargetPosition(duckSpinner.getCurrentPosition());
        duckSpinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        duckSpinner.setDirection(DcMotor.Direction.FORWARD);
    }
    public void setPIDMode() {
        //sets the drive train in a mode for PID
        telemetry.addData("Mode", "Init for PID");

        //inits the motors in a way suitible for manual control
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        duckSpinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        duckSpinner.setDirection(DcMotor.Direction.FORWARD);
    }

    public double getHeading(double leftStickX, double leftStickY){
        // inverse tangent, gives the angle of the point
        double theta = Math.atan2(leftStickX, leftStickY);
        return (theta + 3.1415/2);
    }
    public double getMagnitude(double leftStickX, double leftStickY) {
        //pretty much just pythag, figure out how far point is from center - combined the GetHeading direction and speed can be gotten
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
    public void Halt(){
        //stops the wheels
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Mode", "Stopping");
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
        backLeftDrive.setPower(0);
    }

}