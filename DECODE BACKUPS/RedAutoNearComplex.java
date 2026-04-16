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

public class RedAutoNearComplex extends LinearOpMode {
    // Motor and servo declarations
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    
    // here marks the beginning of the new additions and their resulting errors. NOTE: you have to add these into both the brain's I/O and the initializeHardware method
    private DcMotorEx intakeMotor;
    private DcMotorEx flyWheelMotor;
    private Servo hoodServo; // in case it isn't obvious this controls the "hood" that directs the balls out of the flywheels
    private Servo rollServo; // controls the rolling of the balls into the flywheel
    
    
    //pifd coof
    
    double a = 436;
    double b = 0;
    double c = 0; // 0.05
    double d = 19;
    
    //timer
    ElapsedTime startTime = new ElapsedTime();
    
    int stage = 1;
    static final double ROBOT_RADIUS_M = 228.6; // Distance from robot center to wheel (adjust for your chassis)
    
    public void Movement(double x, double y, double rx, double power) {
        rx = rx*(Math.PI/180);
        //turns mm to ticks
        int targetTicksX = (int)((x/(0.1*Math.PI))*420);
        int targetTicksY = (int)((y/(0.1*Math.PI))*420);
        
        
        //turns ticks into position
        int frontLeft = frontLeftMotor.getCurrentPosition()+targetTicksX-targetTicksY-((int)(2*rx*ROBOT_RADIUS_M));
        int frontRight = frontLeftMotor.getCurrentPosition()+targetTicksX+targetTicksY+((int)(2*rx*ROBOT_RADIUS_M));
        int backLeft = frontLeftMotor.getCurrentPosition()+targetTicksX-targetTicksY+((int)(2*rx*ROBOT_RADIUS_M));
        int backRight = frontLeftMotor.getCurrentPosition()+targetTicksX+targetTicksY-((int)(2*rx*ROBOT_RADIUS_M));
        
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
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);
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
        
        flyWheelMotor.setVelocityPIDFCoefficients(a,b,c,d);
        
        waitForStart();
        
        while (opModeIsActive()) {
            // switch case so that timing works out. The delay on each step of the auto is the number in the conditional after it
            switch (stage) {
                case 1:
                    hoodServo.setPosition(30 * Math.PI / 180);
                    // NOTE: FLYWHEEL VELOCITY IS LOWER THAN PRESET TO COUNTERACT OVERCORRECTION
                    flyWheelMotor.setVelocity(1110); // spins up flywheel and waits 5 seconds (look at conditional in case 2)
                    stage += 1;
                    startTime.reset();
                    break;
                case 2:
                    if(startTime.seconds() >= 2){
                        rollServo.setPosition(Math.PI); // disengages blocker and waits 2 seconds to prevent misfires
                        stage += 1;
                        startTime.reset();                        
                    }
                    break;
                case 3:
                    if(startTime.seconds() >= 1) {
                        intakeMotor.setVelocity(2000); // starts intaking to shoot and waits 13 seconds for it to fully unload (in next conditional)
                        stage += 1;
                        startTime.reset();
                    }
                    break;
                case 4:
                    if(startTime.seconds() >= 6){
                        //intakeMotor.setVelocity(2000);
                        rollServo.setPosition(0);
                        flyWheelMotor.setVelocityPIDFCoefficients(100, 0, 0, 15);
                        flyWheelMotor.setVelocity(1195); // might change to 1 for convinience
                        Movement(-0.9, 1.05, 40,0.8); // goes backwards and looks at pieces
                        // -0.65 middle one
                        WaitForDrive();
                        stage += 1;
                        startTime.reset();
                    }
                    break;
                case 5:
                    if(startTime.seconds() >= 0){
                        Movement(0.8, 0, 0,0.5); // forward to pick up pieces
                        WaitForDrive();
                        stage += 1;
                        startTime.reset();
                    }
                    break;
                case 6:
                    if(startTime.seconds() >= 0){
                        Movement(-0.7, 0.1, 0,0.8); // back
                        WaitForDrive();
                        stage += 1;
                        startTime.reset();
                    }
                    break;
                case 7:
                    if(startTime.seconds() >= 0){
                        intakeMotor.setVelocity(0); // stops intaking to avoid misfires or accidentally touching flywheel
                        hoodServo.setPosition(42.33 * Math.PI / 180);
                        Movement(0, -0.6, 0,0.8); // left to get into shooting position
                        WaitForDrive();
                        stage += 1;
                        startTime.reset();
                    }
                    break;
                case 8:
                    if(startTime.seconds() >= 0){
                        rollServo.setPosition(Math.PI); // opens roll servo
                        Movement(0, 0, -45,0.8); // rotates back
                        WaitForDrive();
                        stage += 1;
                        startTime.reset();
                    }
                    break;
                case 9:
                    if(startTime.seconds() >= 1){
                        intakeMotor.setVelocity(2000); // turns intake on to fire
                        stage += 1;
                        startTime.reset();
                    }
                    break;
                case 10:
                    if(startTime.seconds() >= 6){
                        intakeMotor.setVelocity(0); // turns intake off
                        flyWheelMotor.setVelocity(0); // turns off flywheel
                        hoodServo.setPosition(30 * Math.PI / 180); // sets the hood back to start position
                        rollServo.setPosition(0);
                        Movement(0, 0.7, 0,0.8); // leaves the zone
                        WaitForDrive();
                        stage += 1;
                        startTime.reset();
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
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
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
        
        // change intake to velocity so it won't stall
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
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