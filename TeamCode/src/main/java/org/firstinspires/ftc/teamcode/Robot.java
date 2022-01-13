package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.io.File;

public class Robot {

    public DcMotor frontLeftDrive;
    public DcMotor frontRightDrive;
    public DcMotor backLeftDrive;
    public DcMotor backRightDrive;
    public DcMotor duckSpinner;
    public DcMotor clawArm;
    public CRServo claw;
    public Telemetry telemetry;

    //init and declare war
    public OpMode opmode;
    public HardwareMap hardwareMap;

    //construct robot
    public Robot() {

    }

    //Initialize motors and servos
    public void init(HardwareMap hardwareMap, Telemetry telemetry, OpMode opmode){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.opmode = opmode;

        // This section turns the names of the pieces of hardware into variables that we can program with.
        // Make sure that the device name is the exact same thing you typed in on the configuration on the driver hub.
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

        // This section sets the direction of all of the motors. Depending on the motor, this may change later in the program.
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        duckSpinner.setDirection(DcMotor.Direction.FORWARD);
        clawArm.setDirection(DcMotor.Direction.FORWARD);

        // This tells the motors to chill when we're not powering them.
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");

    }


    public boolean isWheelsBusy(){
        if(backLeftDrive.isBusy() || frontLeftDrive.isBusy() || frontRightDrive.isBusy() || backRightDrive.isBusy()){
            return true;
        }
        else{
            return false;
        }
    }


    public void turnDuckSpinner(double power){
        duckSpinner.setPower(power);
        telemetry.addData("Ducks", "Whee!");
    }
    public void stopDuckSpinner(){
        duckSpinner.setPower(0);
    }

    public void stopAllMotors() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }

     public void setTargets(String direction, int ticks) {

            if (direction == "Right"){
                frontLeftDrive.setTargetPosition(-ticks + frontLeftDrive.getCurrentPosition());
                frontRightDrive.setTargetPosition(ticks + frontRightDrive.getCurrentPosition());
                backLeftDrive.setTargetPosition(ticks + backLeftDrive.getCurrentPosition());
                backRightDrive.setTargetPosition(-ticks + backRightDrive.getCurrentPosition());

            } else if (direction == "Left"){
                frontLeftDrive.setTargetPosition(ticks + frontLeftDrive.getCurrentPosition());
                frontRightDrive.setTargetPosition(-ticks + frontRightDrive.getCurrentPosition());
                backLeftDrive.setTargetPosition(-ticks + backLeftDrive.getCurrentPosition());
                backRightDrive.setTargetPosition(ticks + backRightDrive.getCurrentPosition());

            } else if (direction == "Forward"){
                frontLeftDrive.setTargetPosition(-ticks + frontLeftDrive.getCurrentPosition());
                frontRightDrive.setTargetPosition(-ticks + frontRightDrive.getCurrentPosition());
                backLeftDrive.setTargetPosition(-ticks + backLeftDrive.getCurrentPosition());
                backRightDrive.setTargetPosition(-ticks + backRightDrive.getCurrentPosition());

            } else if (direction == "Backward") {
                frontLeftDrive.setTargetPosition(ticks - frontLeftDrive.getCurrentPosition());
                frontRightDrive.setTargetPosition(ticks - frontRightDrive.getCurrentPosition());
                backLeftDrive.setTargetPosition(ticks - backLeftDrive.getCurrentPosition());
                backRightDrive.setTargetPosition(ticks - backRightDrive.getCurrentPosition());

           /* else if (direction == "turnRight" )   {
                frontLeftDrive.setTargetPosition(ticks + frontLeftDrive.getCurrentPosition());
                frontRightDrive.setTargetPosition(ticks - frontRightDrive.getCurrentPosition());
                backLeftDrive.setTargetPosition(ticks + backLeftDrive.getCurrentPosition());
                backRightDrive.setTargetPosition(ticks - backRightDrive.getCurrentPosition());

            }*/
            }
    }

    public void positionRunningMode(){

            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

    /* lic void powerSet(double frontLeftSpeed, double frontRightSpeed, double backLeftSpeed, double backRightSpeed) {
            frontLeftDrive.setPower(frontLeftSpeed);
            frontRightDrive.setPower(frontRightSpeed);
            backLeftDrive.setPower(backLeftSpeed);
            backRightDrive.setPower(backRightspeed);*/
    public void powerSet(double speed) {
        frontLeftDrive.setPower(speed);
        frontRightDrive.setPower(speed);
        backLeftDrive.setPower(speed);
        backRightDrive.setPower(speed);

    }

    public void openAndCloseClaw (double position){
        telemetry.addData("ServoPort", "Port: " + claw.getPortNumber());
        claw.getController().setServoPosition(claw.getPortNumber(), position);
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
            telemetry.addData("Motors", String.format("FL Power(%.2f) FL Location (%d) FL Target (%d)", frontLeftDrive.getPower(), frontLeftDrive.getCurrentPosition(), frontLeftDrive.getTargetPosition()));
            telemetry.addData("Motors", String.format("FR Power(%.2f) FR Location (%d) FR Target (%d)", frontRightDrive.getPower(), frontRightDrive.getCurrentPosition(), frontRightDrive.getTargetPosition()));
            telemetry.addData("Motors", String.format("BL Power(%.2f) BL Location (%d) BL Target (%d)", backLeftDrive.getPower(), backLeftDrive.getCurrentPosition(), backLeftDrive.getTargetPosition()));
            telemetry.addData("Motors", String.format("BR Power(%.2f) BR Location (%d) BR Target (%d)", backRightDrive.getPower(), backRightDrive.getCurrentPosition(), backRightDrive.getTargetPosition()));
            telemetry.addData("Motors", "Duck Spinner (%.2f)", duckSpinner.getPower());
            telemetry.addData("Motors", "Arm (%.2f)", clawArm.getPower());
            telemetry.update();
     }

    public double inchesToTicks(double inches){
        // returns the inches * ticks per rotation / wheel circ
        return ((inches/12.25) * 537.6 / .5);
        //todo Reference that 1 inch ~= 50 ticks
    }

    }
