package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.*;

@Autonomous

public class ZachsTickTest extends LinearOpMode {
    // Motor and servo declarations
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    
    // here marks the beginning of the new additions and their resulting errors. NOTE: you have to add these into both the brain's I/O and the initializeHardware method
    private DcMotor intakeMotor;
    private DcMotorEx flyWheelMotor;
    private Servo hoodServo; // in case it isn't obvious this controls the "hood" that directs the balls out of the flywheels
    private Servo rollServo; // controls the rolling of the balls into the flywheel
    
    //timer
    ElapsedTime startTime = new ElapsedTime();
    
    
    int stage = 1;
    static final double ROBOT_RADIUS_M = 228.6; // Distance from robot center to wheel (adjust for your chassis)
    
    public void Movement(double x, double y, double rx) {
        rx = rx*(Math.PI/180);
        //turns mm to ticks
        int targetTicksX = (int)((x/(0.1*Math.PI))*420);
        int targetTicksY = (int)((y/(0.1*Math.PI))*420);
        
        
        //turns ticks into position
        int frontLeft = frontLeftMotor.getCurrentPosition()+targetTicksX+targetTicksY-((int)(2*rx*ROBOT_RADIUS_M));
        int frontRight = frontLeftMotor.getCurrentPosition()+targetTicksX-targetTicksY+((int)(2*rx*ROBOT_RADIUS_M));
        int backLeft = frontLeftMotor.getCurrentPosition()+targetTicksX+targetTicksY+((int)(2*rx*ROBOT_RADIUS_M));
        int backRight = frontLeftMotor.getCurrentPosition()+targetTicksX-targetTicksY-((int)(2*rx*ROBOT_RADIUS_M));
        
        //sets target posiotion
        frontLeftMotor.setTargetPosition(frontLeft);
        frontRightMotor.setTargetPosition(frontRight);
        backLeftMotor.setTargetPosition(backLeft);
        backRightMotor.setTargetPosition(backRight);
        
        //switch to run to position mode
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        //set power
        frontLeftMotor.setPower(0.6);
        frontRightMotor.setPower(0.6);
        backLeftMotor.setPower(0.6);
        backRightMotor.setPower(0.6);
    }
    public void WaitForDrive(){
        while (opModeIsActive() & (frontLeftMotor.isBusy()||frontRightMotor.isBusy()||backLeftMotor.isBusy()||backRightMotor.isBusy())){
            idle();
        }
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    public void runOpMode() {
        initializeHardware();
        
        waitForStart();
        
        while (opModeIsActive()){
            switch (stage){
                case 1:
                    Movement(1,0,0);
                    WaitForDrive();
                    startTime.reset();
                    stage++;
                    
                case 2:
                    if(startTime.seconds() >= 1){
                        Movement(0,1,0);
                        WaitForDrive();
                        startTime.reset();
                        stage++;
                    }
                    break;
                case 3:
                    if(startTime.seconds() >= 1){
                        Movement(-1,-1,0);
                        WaitForDrive();
                        startTime.reset();
                        stage++;
                    }
                    break;
                    
                case 4:
                    if(startTime.seconds() >= 1){
                        Movement(0,0,90);
                        WaitForDrive();
                        startTime.reset();
                        stage++;
                    }
                    break;
                    
            }
        }
        
        
    }
    
    private void initializeHardware() {
        // Motor initialization
        // front left is back right, front right is back left, back Left is front left, back right is front right
        frontLeftMotor = hardwareMap.dcMotor.get("backRightMotor"); // gang wtf
        backLeftMotor = hardwareMap.dcMotor.get("frontRightMotor"); // gang wtf
        frontRightMotor = hardwareMap.dcMotor.get("backLeftMotor"); // gang wtf
        backRightMotor = hardwareMap.dcMotor.get("frontLeftMotor"); // gang wtf
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        flyWheelMotor = hardwareMap.get(DcMotorEx.class, "flyWheelMotor");
        hoodServo = hardwareMap.servo.get("hoodServo");
        rollServo = hardwareMap.servo.get("rollServo");

        // Set motor directions
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flyWheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // set flywheel to use encoder so we can use ticks for more reliable shooting
        flyWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // this one resets the motor encoder as is best practice
        flyWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        //SET TO USE ENCODER FOR DRIVE TERRAIN 
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Ensure all motors have BRAKE behavior set
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flyWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}