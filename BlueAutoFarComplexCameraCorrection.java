package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.*;
import java.util.List;

@Autonomous

public class BlueAutoFarComplexCameraCorrection extends LinearOpMode {
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
    private Servo cameraServo;
    
    
    //pifd coof
    
    double a = 436;
    double b = 0;
    double c = 0; // 0.05
    double d = 19;
    
    //timer
    ElapsedTime startTime = new ElapsedTime();
    
    int stage = 1;
    static final double ROBOT_RADIUS_M = 228.6; // Distance from robot center to wheel (adjust for your chassis)
    
    VisionPortal visionPortal; 
    AprilTagProcessor aprilTag; 
    int TARGET_TAG_ID = 20; 
    
    double xc = 0;
    double yc = 0;
    double zc = 0;
    double angle = 0;
    
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

    public void Center(targetX, targetY, targetYaw) {
        List<AprilTagDetection> detections = aprilTag.getDetections(); 
        for (AprilTagDetection detection : detections) { 
            if (detection.id == TARGET_TAG_ID && detection.ftcPose != null) {
                double x = targetX - detection.ftcPose.X;
                double y = targetY - detection.ftcPose.Y * Math.cos(10); // FIND OUT THE ANGLE FROM THE HORIZONTAL!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                double yaw = targetYaw - detection.ftcPose.yaw;
                Movement(y, x, yaw, 1);
                WaitForDrive();
            }
        } 
    }
    
    public void runOpMode() {
        initializeHardware();
        
        flyWheelMotor.setVelocityPIDFCoefficients(a,b,c,d);
        cameraServo.setPosition(10 * Math.PI / 180);
        
        aprilTag = new AprilTagProcessor.Builder() 
            .setDrawAxes(true) 
            .setDrawCubeProjection(true) 
            .setDrawTagOutline(true)
            .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)
            .build(); 
        visionPortal = new VisionPortal.Builder() 
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) 
            .addProcessor(aprilTag)
            .build(); 
        
        waitForStart();
        
        while (opModeIsActive()) {
            // switch case so that timing works out. The delay on each step of the auto is the number in the conditional after it
            switch (stage) {
                case 1:
                    hoodServo.setPosition(48.5 * Math.PI / 180);
                    // NOTE: FLYWHEEL VELOCITY IS LOWER THAN PRESET TO COUNTERACT OVERCORRECTION
                    flyWheelMotor.setVelocity(1600);
                    stage += 1;
                    startTime.reset();
                    break;
                case 2:
                    if(startTime.seconds() >= 0){
                        Movement(0.2, 0, -22, 1);// CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE
                        WaitForDrive();
                        stage += 1;
                        startTime.reset();
                        }
                    break;
                case 3:
                    if(startTime.seconds() >= 1){
                        rollServo.setPosition(Math.PI); // disengages blocker and waits 2 seconds to prevent misfires
                        stage += 1;
                        startTime.reset();                        
                    }
                    break;
                case 4:
                    if(startTime.seconds() >= 1) {
                        Center(-1, 5, -5);// CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE
                        intakeMotor.setVelocity(2000); // starts intaking to shoot and waits 13 seconds for it to fully unload (in next conditional)
                        stage += 1;
                        startTime.reset();
                    }
                    break;
                case 5:
                    if(startTime.seconds() >= 6){
                        rollServo.setPosition(0);
                        Movement(0.6, 0, -68,1); // goes forward and looks at balls // CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE
                        WaitForDrive();
                        stage += 1;
                        startTime.reset();
                    }
                    break;
                case 6:
                    if(startTime.seconds() >= 0){
                        Movement(0.7, 0, 0,1); // forward to pick up pieces// CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE
                        WaitForDrive();
                        stage += 1;
                        startTime.reset();
                    }
                    break;
                case 7:
                    if(startTime.seconds() >= 0){
                        intakeMotor.setVelocity(0); // stops intaking to avoid misfires or accidentally touching flywheel
                        Movement(-0.65, -0.7, 68, 1); // back and looks at goal// CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE
                        WaitForDrive();
                        stage += 1;
                        startTime.reset();
                    }
                    break;
                case 8:
                    if(startTime.seconds() >= 0){
                        rollServo.setPosition(Math.PI); // opens roll servo
                        stage += 1;
                        startTime.reset();
                    }
                    break;
                case 9:
                    if(startTime.seconds() >= 1){
                        Center(-1, 5, -5);// CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE
                        intakeMotor.setVelocity(1500); // turns intake on to fire
                        stage += 1;
                        startTime.reset();
                    }
                    break;
                case 10:
                    if(startTime.seconds() >= 6){
                        rollServo.setPosition(0);
                        Movement(0.6, 0, -68,1); // goes forward and looks at balls // CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE
                        WaitForDrive();
                        stage += 1;
                        startTime.reset();
                    }
                    break;
                case 11:
                    if(startTime.seconds() >= 0){
                        Movement(0.7, 0, 0,1); // forward to pick up pieces// CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE
                        WaitForDrive();
                        stage += 1;
                        startTime.reset();
                    }
                    break;
                case 12:
                    if(startTime.seconds() >= 0){
                        intakeMotor.setVelocity(0); // stops intaking to avoid misfires or accidentally touching flywheel
                        Movement(-0.65, -0.7, 68, 1); // back and looks at goal// CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE
                        WaitForDrive();
                        stage += 1;
                        startTime.reset();
                    }
                    break;
                case 13:
                    if(startTime.seconds() >= 0){
                        rollServo.setPosition(Math.PI); // opens roll servo
                        stage += 1;
                        startTime.reset();
                    }
                    break;
                case 14:
                    if(startTime.seconds() >= 1){
                        Center(-1, 5, -5);// CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE
                        intakeMotor.setVelocity(1500); // turns intake on to fire
                        stage += 1;
                        startTime.reset();
                    }
                    break;
                case 15:
                    if(startTime.seconds() >= 6){
                        rollServo.setPosition(0);
                        Movement(0.6, 0, -68,1); // goes forward and looks at balls // CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE
                        WaitForDrive();
                        stage += 1;
                        startTime.reset();
                    }
                    break;
                case 16:
                    if(startTime.seconds() >= 0){
                        Movement(0.7, 0, 0,1); // forward to pick up pieces // CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE
                        WaitForDrive();
                        stage += 1;
                        startTime.reset();
                    }
                    break;
                case 17:
                    if(startTime.seconds() >= 0){
                        intakeMotor.setVelocity(0); // stops intaking to avoid misfires or accidentally touching flywheel
                        Movement(-0.65, -0.7, 68, 1); // back and looks at goal // CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE
                        WaitForDrive();
                        stage += 1;
                        startTime.reset();
                    }
                    break;
                case 18:
                    if(startTime.seconds() >= 0){
                        rollServo.setPosition(Math.PI); // opens roll servo
                        stage += 1;
                        startTime.reset();
                    }
                    break;
                case 19:
                    if(startTime.seconds() >= 1){
                        Center(-1, 5, -5);
                        intakeMotor.setVelocity(1500); // turns intake on to fire
                        stage += 1;
                        startTime.reset();
                    }
                    break;
                case 20:
                    if(startTime.seconds() >= 6){
                        Movement(0.5, -0.5, 0, 1); // leaves zone
                        WaitForDrive();
                    }
            }
        }
    }
    
    private void initializeHardware() {
        // Motor initialization
        /*
        frontLeftMotor = hardwareMap.dcMotor.get("backRightMotor");
        backLeftMotor = hardwareMap.dcMotor.get("frontRightMotor");
        frontRightMotor = hardwareMap.dcMotor.get("backLeftMotor");
        backRightMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        */
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        flyWheelMotor = hardwareMap.get(DcMotorEx.class, "flyWheelMotor");
        hoodServo = hardwareMap.servo.get("hoodServo");
        rollServo = hardwareMap.servo.get("rollServo");
        cameraServo = hardwareMap.servo.get("cameraServo");

        // Set motor directions
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flyWheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // set flywheel to use encoder so we can use ticks for more reliable shooting
        flyWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // this one resets the motor encoder as is best practice
        flyWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Ensure all motors have BRAKE behavior set
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flyWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}