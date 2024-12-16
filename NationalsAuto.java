package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="NationalsAuto", group="Linear Opmode")
public class NationalsAuto extends LinearOpMode {
    // Motor and servo declarations
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotor armMotor;
    private Servo specimenClaw;

    // Drivetrain Encoder Constants
    private static final double DRIVETRAIN_COUNTS_PER_MOTOR_REV = 28;  // For REV Core Hex Motor
    private static final double DRIVETRAIN_GEAR_REDUCTION = 20.0;  // If using geared motors
    private static final double WHEEL_DIAMETER_INCHES = 4.0;  // Typical mecanum wheel size
    private static final double DRIVETRAIN_COUNTS_PER_INCH = (DRIVETRAIN_COUNTS_PER_MOTOR_REV * DRIVETRAIN_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    // Strafing Correction Factors
    private double frontLeftStrafeCorrectionFactor = 0.0;   // Adjusts front left motor during strafing
    private double backLeftStrafeCorrectionFactor = -0.05;  // Adjusts back left motor during strafing
    private double frontRightStrafeCorrectionFactor = 0.0;  // Adjusts front right motor during strafing
    private double backRightStrafeCorrectionFactor = 0.05;  // Adjusts back right motor during strafing
    private static final double STRAFE_CORRECTION_SCALE = 1.5;

    // Arm Motor Encoder Constants for Hex Ultraplanetary
    private static final double ARM_COUNTS_PER_MOTOR_REV = 28;  // Base motor revolution
    private static final double ARM_GEAR_REDUCTION = 40.0;  // Example: 40:1 gear reduction
    private static final double ARM_COUNTS_PER_DEGREE = (ARM_COUNTS_PER_MOTOR_REV * ARM_GEAR_REDUCTION) / 360.0;

    // Servo position constants
    private static final double CLAW_OPEN_POSITION = 1.0;
    private static final double CLAW_CLOSED_POSITION = 0.0;

    // Speed constants
    private static final double DRIVE_SPEED = 0.5;
    private static final double TURN_SPEED = 0.4;
    private static final double ARM_SPEED = 0.5;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeHardware();

        // Wait for start button
        waitForStart();

        if (opModeIsActive()) {
            // Demonstrate concurrent movements:
            // 1. Rotate arm to 90 degrees
            // 2. Strafe right 24 inches
            // Both movements happen simultaneously
            moveArmToDegreeAsync(90, ARM_SPEED);
            strafeRightAsync(DRIVE_SPEED, 24);

            // Wait for both movements to complete
            while (opModeIsActive() && 
                   (armMotor.isBusy() || 
                    frontLeftMotor.isBusy() || 
                    backLeftMotor.isBusy() || 
                    frontRightMotor.isBusy() || 
                    backRightMotor.isBusy())) {
                
                // Optional: Update telemetry during movement
                telemetry.addData("Arm Position", armMotor.getCurrentPosition());
                telemetry.addData("Front Left Position", frontLeftMotor.getCurrentPosition());
                telemetry.update();
            }


            // Demonstrate another concurrent movement sequence
            // Open claw while strafing left
            openClawAsync();
            strafeLeftAsync(DRIVE_SPEED, 12);

            // Wait for movements to complete
            while (opModeIsActive() && 
                   (frontLeftMotor.isBusy() || 
                    backLeftMotor.isBusy() || 
                    frontRightMotor.isBusy() || 
                    backRightMotor.isBusy())) {
                
                telemetry.addData("Strafe Progress", "In Progress");
                telemetry.update();
            }

            // Stop all motors
            stopAllMotors();
        }
    }

    private void openClawAsync() {
        specimenClaw.setPosition(CLAW_OPEN_POSITION);
        // No sleep to allow concurrent movement
    }
    private void closeClawAsync() {
        specimenClaw.setPosition(CLAW_CLOSED_POSITION);
        // No sleep to allow concurrent movement
    }


    private void initializeHardware() {
        // Motor initialization
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        armMotor = hardwareMap.dcMotor.get("armMotor");
        specimenClaw = hardwareMap.servo.get("specimen claw");

        // Set motor directions
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set motor modes
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set zero power behavior
        setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void moveArmToDegreeAsync(double targetDegree, double speed) {
        if (opModeIsActive()) {
            // Calculate target position based on degrees
            int newArmTarget = (int)(targetDegree * ARM_COUNTS_PER_DEGREE);

            // Set target position
            armMotor.setTargetPosition(newArmTarget);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set power
            armMotor.setPower(Math.abs(speed));
        }
    }

    private void strafeRightAsync(double speed, double inches) {
        if (opModeIsActive()) {
            // Calculate encoder targets
            int ticks = (int)(inches * DRIVETRAIN_COUNTS_PER_INCH);

            // Calculate base motor targets with strafing
            int frontLeftTarget = frontLeftMotor.getCurrentPosition() - ticks;
            int backLeftTarget = backLeftMotor.getCurrentPosition() + ticks;
            int frontRightTarget = frontRightMotor.getCurrentPosition() + ticks;
            int backRightTarget = backRightMotor.getCurrentPosition() - ticks;

            // Set motor targets
            frontLeftMotor.setTargetPosition(frontLeftTarget);
            backLeftMotor.setTargetPosition(backLeftTarget);
            frontRightMotor.setTargetPosition(frontRightTarget);
            backRightMotor.setTargetPosition(backRightTarget);

            // Set run mode
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Calculate corrected powers with dynamic scaling
            double correctedFrontLeftPower = speed + 
                (frontLeftStrafeCorrectionFactor * speed * STRAFE_CORRECTION_SCALE);
            double correctedBackLeftPower = speed + 
                (backLeftStrafeCorrectionFactor * speed * STRAFE_CORRECTION_SCALE);
            double correctedFrontRightPower = speed + 
                (frontRightStrafeCorrectionFactor * speed * STRAFE_CORRECTION_SCALE);
            double correctedBackRightPower = speed + 
                (backRightStrafeCorrectionFactor * speed * STRAFE_CORRECTION_SCALE);

            // Set motor powers
            frontLeftMotor.setPower(Math.abs(correctedFrontLeftPower));
            backLeftMotor.setPower(Math.abs(correctedBackLeftPower));
            frontRightMotor.setPower(Math.abs(correctedFrontRightPower));
            backRightMotor.setPower(Math.abs(correctedBackRightPower));
        }
    }

    private void stopAllMotors() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        armMotor.setPower(0);

        // Reset to RUN_USING_ENCODER mode
        setMotorRunModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setMotorRunModes(DcMotor.RunMode runMode) {
        frontLeftMotor.setMode(runMode);
        backLeftMotor.setMode(runMode);
        frontRightMotor.setMode(runMode);
        backRightMotor.setMode(runMode);
        armMotor.setMode(runMode);
    }

    private void setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        frontLeftMotor.setZeroPowerBehavior(behavior);
        backLeftMotor.setZeroPowerBehavior(behavior);
        frontRightMotor.setZeroPowerBehavior(behavior);
        backRightMotor.setZeroPowerBehavior(behavior);
        armMotor.setZeroPowerBehavior(behavior);
    }

    private void openClaw() {
        specimenClaw.setPosition(CLAW_OPEN_POSITION);
        sleep(500);  // Allow time for servo to move
    }

    private void closeClaw() {
        specimenClaw.setPosition(CLAW_CLOSED_POSITION);
        sleep(500);  // Allow time for servo to move
    }

    // Additional methods for other strafing directions can be added similarly
    private void strafeLeftAsync(double speed, double inches) {
        if (opModeIsActive()) {
            // Calculate encoder targets
            int ticks = (int)(inches * DRIVETRAIN_COUNTS_PER_INCH);

            // Calculate base motor targets
            int frontLeftTarget = frontLeftMotor.getCurrentPosition() + ticks;
            int backLeftTarget = backLeftMotor.getCurrentPosition() - ticks;
            int frontRightTarget = frontRightMotor.getCurrentPosition() - ticks;
            int backRightTarget = backRightMotor.getCurrentPosition() + ticks;

            // Set motor targets
            frontLeftMotor.setTargetPosition(frontLeftTarget);
            backLeftMotor.setTargetPosition(backLeftTarget);
            frontRightMotor.setTargetPosition(frontRightTarget);
            backRightMotor.setTargetPosition(backRightTarget);

            // Set run mode
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Calculate corrected powers with dynamic scaling
            double correctedFrontLeftPower = speed + 
                (frontLeftStrafeCorrectionFactor * speed * STRAFE_CORRECTION_SCALE);
            double correctedBackLeftPower = speed + 
                (backLeftStrafeCorrectionFactor * speed * STRAFE_CORRECTION_SCALE);
            double correctedFrontRightPower = speed + 
                (frontRightStrafeCorrectionFactor * speed * STRAFE_CORRECTION_SCALE);
            double correctedBackRightPower = speed + 
                (backRightStrafeCorrectionFactor * speed * STRAFE_CORRECTION_SCALE);

            // Set motor powers
            frontLeftMotor.setPower(Math.abs(correctedFrontLeftPower));
            backLeftMotor.setPower(Math.abs(correctedBackLeftPower));
            frontRightMotor.setPower(Math.abs(correctedFrontRightPower));
            backRightMotor.setPower(Math.abs(correctedBackRightPower));
        }
    }
}
