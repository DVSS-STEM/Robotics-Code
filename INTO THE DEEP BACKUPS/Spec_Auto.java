package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "2 Spec", group = "Autonomous")
public class Spec_Auto extends LinearOpMode {
    // Motor declarations
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotor armMotor;

    // Servo declarations
    private Servo specimen_claw;
    
    private ElapsedTime moveTimer = new ElapsedTime();

    // Constants
    private static final double CLAW_OPEN_POSITION = 0.5;
    private static final double CLAW_CLOSED_POSITION = 0.9;
    private static final double ARM_POWER = 1.0;
    
    private static final int wall_pos = 430;
    
    @Override
    public void runOpMode() {
        // Initialize motors
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        armMotor = hardwareMap.dcMotor.get("armMotor");

        // Initialize servo
        specimen_claw = hardwareMap.servo.get("specimen claw");

        // Set motor directions
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Configure arm motor for encoder use
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set zero power behavior
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        closeClaw();
        waitForStart();
        
        // 2600 is almost hang
        // 3000 is hang
        // 350 is wall grab
        // 
        
        if (opModeIsActive()) {
            
            closeClaw();
            
            moveForwardWithArm(0.2, 450, 3200);
            
            sleep(300);
            //////// UNCOMMENT THIS CODE FOR THE FIRST SECTION OF THE AUTO /////////////////////

            moveForward(0.5, 980);
            sleep(500);
            openClaw();
            sleep(500);
            //moveBack(0.5,100);
            //setArmPosition(2600);
            
            
            //setArmPosition(-50);
            
            //sleep(500);

            moveBack(0.5, 1000);
            sleep(200);
            
            strafeRight(1.0, 3500);
            
            moveForwardWithArm(0.5, 500,360);
            sleep(2000);
            moveBack(0.5,375);
            closeClaw();
            telemetry.addData("SpecArm",armMotor.getCurrentPosition());
            telemetry.update();
            sleep(100000000);
            
            
        }
    }

    /**
     * Move the robot forward and the arm simultaneously
     * @param power Motor power for driving
     * @param milliseconds Duration of movement
     * @param armPosition Target position for the arm in encoder ticks
     */
    private void moveForwardWithArm(double power, long milliseconds, int armPosition) {
        moveTimer.reset();

        // Set the arm to move to the target position
        setArmPosition(armPosition);

        // Start moving the robot forward
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftMotor.setPower(power);
        backLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backRightMotor.setPower(power);

        // Wait for the specified time while allowing arm to move
        while (opModeIsActive() && moveTimer.milliseconds() < milliseconds) {
            telemetry.addData("Move Forward With Arm", "Time: %2.1f", moveTimer.seconds());
            telemetry.addData("Arm Position", armMotor.getCurrentPosition());
            telemetry.update();
        }

        // Stop the drive motors
        stopDriveMotors();
    }

    /**
     * Move the robot backward and the arm simultaneously
     * @param power Motor power for driving
     * @param milliseconds Duration of movement
     * @param armPosition Target position for the arm in encoder ticks
     */
    private void moveBackwardWithArm(double power, long milliseconds, int armPosition) {
        moveTimer.reset();

        // Set the arm to move to the target position
        setArmPosition(armPosition);

        // Start moving the robot backward
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftMotor.setPower(-power);
        backLeftMotor.setPower(-power);
        frontRightMotor.setPower(-power);
        backRightMotor.setPower(-power);

        // Wait for the specified time while allowing arm to move
        while (opModeIsActive() && moveTimer.milliseconds() < milliseconds) {
            telemetry.addData("Move Backward With Arm", "Time: %2.1f", moveTimer.seconds());
            telemetry.addData("Arm Position", armMotor.getCurrentPosition());
            telemetry.update();
        }

        // Stop the drive motors
        stopDriveMotors();
    }

    /**
     * Set the arm position using encoder ticks
     * @param position Target position in encoder ticks
     */
    private void setArmPosition(int position) {
        armMotor.setTargetPosition(position);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1.0);
    }

    /**
     * Move forward for a specified time while allowing other operations
     * @param power Motor power
     * @param milliseconds Duration of movement
     */
    private void moveForward(double power, long milliseconds) {
        moveTimer.reset();
        frontLeftMotor.setPower(power);
        backLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backRightMotor.setPower(power);

        // Wait for the specified time while allowing arm to move
        while (opModeIsActive() && moveTimer.milliseconds() < milliseconds) {
            telemetry.addData("Move Forward", "Time: %2.1f", moveTimer.seconds());
            telemetry.addData("Arm Position", armMotor.getCurrentPosition());
            telemetry.update();
        }
        stopDriveMotors();
    }

    private void moveBack(double power, long milliseconds) {
        moveForward(-power, milliseconds);
    }

    private void strafeLeft(double power, long milliseconds) {
        moveTimer.reset();
        frontLeftMotor.setPower(-1.0);
        backLeftMotor.setPower(0.9);
        frontRightMotor.setPower(0.9);
        backRightMotor.setPower(-1.0);

        while (opModeIsActive() && moveTimer.milliseconds() < milliseconds) {
            telemetry.addData("Move Right", "Time: %2.1f", moveTimer.seconds());
            telemetry.addData("Arm Position", armMotor.getCurrentPosition());
            telemetry.update();
        }
        stopDriveMotors();
    }

    private void strafeRight(double power, long milliseconds) {
        moveTimer.reset();
        frontLeftMotor.setPower(0.6);
        backLeftMotor.setPower(-0.55);
        frontRightMotor.setPower(-0.55);
        backRightMotor.setPower(0.6);

        while (opModeIsActive() && moveTimer.milliseconds() < milliseconds) {
            telemetry.addData("Move Left", "Time: %2.1f", moveTimer.seconds());
            telemetry.addData("Arm Position", armMotor.getCurrentPosition());
            telemetry.update();
        }
        stopDriveMotors();
    }

    private void rotate10() {
        moveTimer.reset();
        frontLeftMotor.setPower(-1.0);
        backLeftMotor.setPower(-1.0);
        frontRightMotor.setPower(1.0);
        backRightMotor.setPower(1.0);

        while (opModeIsActive() && moveTimer.milliseconds() < 25) {
            telemetry.addData("Rotating", "180");
            telemetry.addData("Arm Position", armMotor.getCurrentPosition());
            telemetry.update();
        }
        stopDriveMotors();
    }
    
    private void rotate5() {
        moveTimer.reset();
        frontLeftMotor.setPower(-1.0);
        backLeftMotor.setPower(-1.0);
        frontRightMotor.setPower(1.0);
        backRightMotor.setPower(1.0);

        while (opModeIsActive() && moveTimer.milliseconds() < 13) {
            telemetry.addData("Rotating", "180");
            telemetry.addData("Arm Position", armMotor.getCurrentPosition());
            telemetry.update();
        }
        stopDriveMotors();
    }
    
    
    private void rotateback10() {
        moveTimer.reset();
        frontLeftMotor.setPower(0.5);
        backLeftMotor.setPower(0.5);
        frontRightMotor.setPower(-0.5);
        backRightMotor.setPower(-0.5);

        while (opModeIsActive() && moveTimer.milliseconds() < 5) {
            telemetry.addData("Rotating", "180");
            telemetry.addData("Arm Position", armMotor.getCurrentPosition());
            telemetry.update();
        }
        stopDriveMotors();
    }
    private void rotate20() {
        moveTimer.reset();
        frontLeftMotor.setPower(1.0);
        backLeftMotor.setPower(1.0);
        frontRightMotor.setPower(-1.0);
        backRightMotor.setPower(-1.0);

        while (opModeIsActive() && moveTimer.milliseconds() < 18) {
            telemetry.addData("Rotating", "180");
            telemetry.addData("Arm Position", armMotor.getCurrentPosition());
            telemetry.update();
        }
        stopDriveMotors();
    }
    
    private void slideRight(double power, long milliseconds) {
        moveTimer.reset();
        frontLeftMotor.setPower(-0.05);
        backLeftMotor.setPower(-1.0);
        frontRightMotor.setPower(-1.0);
        backRightMotor.setPower(-0.05);

        while (opModeIsActive() && moveTimer.milliseconds() < milliseconds) {
            telemetry.addData("Move Left", "Time: %2.1f", moveTimer.seconds());
            telemetry.addData("Arm Position", armMotor.getCurrentPosition());
            telemetry.update();
        }
        stopDriveMotors();
    }

    private void stopDriveMotors() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    private void openClaw() {
        specimen_claw.setPosition(CLAW_OPEN_POSITION);
        sleep(500);
    }

    private void closeClaw() {
        specimen_claw.setPosition(CLAW_CLOSED_POSITION);
        sleep(500);
    }
    
    /**
     * Stop all motors
     */
    private void stopMotors() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }
    
}
