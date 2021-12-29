package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Autonomous.AutonomousPLUS;

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

public class Robot {
    //public static DcMotor frontLeftDrive;
    //  public Vector2d position;

    public DcMotor frontLeftDrive;
    public DcMotor frontRightDrive;
    public DcMotor backLeftDrive;
    public DcMotor backRightDrive;
    private DcMotor duckSpinner;
    private DcMotor clawArm;
    private CRServo claw;
    private ColorSensor Color;
    private final double wheelSizeMM = 100;
    private Telemetry telemetry;

    //init and declare war
    private static int maxSpeed = 1;
    private OpMode opmode;
    private HardwareMap hardwareMap;

    //construct robot
    public Robot() {

    }

    //Initialize motors and servos
    public void init(HardwareMap hardwareMap, Telemetry telemetry, OpMode opmode){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.opmode = opmode;

        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");
        duckSpinner = hardwareMap.get(DcMotor.class, "duckSpinner");
        clawArm = hardwareMap.get(DcMotor.class, "clawArm");
        claw = hardwareMap.get(CRServo.class, "claw");


        this.frontLeftDrive = frontLeftDrive;
        this.frontRightDrive = frontRightDrive;
        this.backLeftDrive = backLeftDrive;
        this.backRightDrive = backRightDrive;
        this.duckSpinner = duckSpinner;
        this.clawArm = clawArm;
        this.claw = claw;

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        duckSpinner.setDirection(DcMotor.Direction.FORWARD);
        clawArm.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");

    }

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

    public void turnDuckSpinner(double power){
        duckSpinner.setPower(power);
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
        duckSpinner.setDirection(DcMotor.Direction.FORWARD);

        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        //frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Mode", "Stopping");
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
        backLeftDrive.setPower(0);
    }

    public void stopAllMotors() {

        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    /*
    * Autonomous Function Holding Cell
     */


     public void setTargets(String direction, int ticks) {

            if (direction == "Left") {
                frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
                frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
                backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
                backRightDrive.setDirection(DcMotor.Direction.REVERSE);

                frontLeftDrive.setTargetPosition(ticks - frontLeftDrive.getCurrentPosition());
                frontRightDrive.setTargetPosition(ticks + frontRightDrive.getCurrentPosition());
                backLeftDrive.setTargetPosition(ticks + backLeftDrive.getCurrentPosition());
                backRightDrive.setTargetPosition(ticks - backRightDrive.getCurrentPosition());

            } else if (direction == "Right"){
                frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
                frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
                backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
                backRightDrive.setDirection(DcMotor.Direction.REVERSE);

                frontLeftDrive.setTargetPosition(ticks + frontLeftDrive.getCurrentPosition());
                frontRightDrive.setTargetPosition(ticks - frontRightDrive.getCurrentPosition());
                backLeftDrive.setTargetPosition(ticks - backLeftDrive.getCurrentPosition());
                backRightDrive.setTargetPosition(ticks + backRightDrive.getCurrentPosition());

            } else if (direction == "Forward"){
                frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
                frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
                backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
                backRightDrive.setDirection(DcMotor.Direction.REVERSE);

                frontLeftDrive.setTargetPosition(ticks + frontLeftDrive.getCurrentPosition());
                frontRightDrive.setTargetPosition(ticks + frontRightDrive.getCurrentPosition());
                backLeftDrive.setTargetPosition(ticks + backLeftDrive.getCurrentPosition());
                backRightDrive.setTargetPosition(ticks + backRightDrive.getCurrentPosition());

            } else if (direction == "Backward"){
                frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
                frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
                backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
                backRightDrive.setDirection(DcMotor.Direction.REVERSE);

                frontLeftDrive.setTargetPosition(ticks - frontLeftDrive.getCurrentPosition());
                frontRightDrive.setTargetPosition(ticks - frontRightDrive.getCurrentPosition());
                backLeftDrive.setTargetPosition(ticks - backLeftDrive.getCurrentPosition());
                backRightDrive.setTargetPosition(ticks - backRightDrive.getCurrentPosition());

            }

     }

    public void positionRunningMode(){

            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

    public void powerSet(double speed) {
            frontLeftDrive.setPower(speed);
            frontRightDrive.setPower(speed);
            backLeftDrive.setPower(speed);
            backRightDrive.setPower(speed);

     }

    public void encoderRunningMode(){
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

     }

     public void encoderReset(){
         frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
     }

     public void tellMotorOutput(){
            telemetry.addData("Motors", "FL Power(%.2f)", frontLeftDrive.getPower(), "FL Location", frontLeftDrive.getCurrentPosition(), "FL Target", frontLeftDrive.getTargetPosition());
            telemetry.addData("Motors", "FR Power (%.2f)", frontRightDrive.getPower(), "FR Location", frontRightDrive.getCurrentPosition(), "FR Target", frontRightDrive.getTargetPosition());
            telemetry.addData("Motors", "BL Power (%.2f)", backLeftDrive.getPower(), "BL Location", backLeftDrive.getCurrentPosition(), "BL Target", backLeftDrive.getTargetPosition());
            telemetry.addData("Motors", "BR Power (%.2f)", backRightDrive.getPower(), "BR Location", backRightDrive.getCurrentPosition(), "BR Target", backRightDrive.getTargetPosition());


     }

}