package org.firstinspires.ftc.teamcode;

//imports
    //eventloop components
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="NationalsTeleop")
public class NationalsTeleop extends LinearOpMode {
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

    // Arm Motor Encoder Constants
    private static final double ARM_COUNTS_PER_MOTOR_REV = 288;
    private static final double ARM_GEAR_REDUCTION = 4.0;
    private static final double ARM_COUNTS_PER_DEGREE = (ARM_COUNTS_PER_MOTOR_REV * ARM_GEAR_REDUCTION) / 360.0;

    // Arm Position Boundaries
    private static final int ARM_MIN_POSITION = 0;  // Lowest position in encoder ticks
    private static final int ARM_MAX_POSITION = (int)(ARM_COUNTS_PER_DEGREE * 205);  // Maximum rotation (e.g., 135 degrees)
    private static final int ARM_SOFT_LIMIT_BUFFER = (int)(ARM_COUNTS_PER_DEGREE * 30);  // 5-degree soft buffer

    // Servo position constants
    private static final double CLAW_OPEN_POSITION = 0.6;
    private static final double CLAW_CLOSED_POSITION = 0.23;
    private static final double FINAL_CLOSED_POSITION = 0.0;

    // Strafing Correction Factors
    private double frontLeftStrafeCorrectionFactor = 0.0;
    private double backLeftStrafeCorrectionFactor = -0.05;
    private double frontRightStrafeCorrectionFactor = 0.0;
    private double backRightStrafeCorrectionFactor = 0.05;
    private static final double STRAFE_CORRECTION_SCALE = 1.5;

    // Button state tracking
    private boolean lastBButtonState = false;

    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeHardware();

        waitForStart();

        while (opModeIsActive()) {
            // Drive control calculations
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;
            
            double rx2 = gamepad2.right_stick_x;
            
            boolean up2 = gamepad2.dpad_up;
            boolean down2 = gamepad2.dpad_down;
            boolean left2 = gamepad2.dpad_left;
            boolean right2 = gamepad2.dpad_right;
            
            double frontLeftPower = 0;
            double backLeftPower = 0;
            double frontRightPower = 0;
            double backRightPower = 0;
            
            double misc = -0.07;

            // Drive mode selection
            if (up2) {
                moveDriveTrainAsync(-0.2, -0.2, -0.2, -0.2);
            } else if (down2) {
                moveDriveTrainAsync(0.2, 0.2, 0.2, 0.2);
            } else if (left2) {
                moveDriveTrainAsync(0.3, -0.3 - 0.05, -0.3, 0.3);
            } else if (right2) {
                moveDriveTrainAsync(-0.3, 0.3 + 0.05, 0.3, -0.3);
            } else if (rx2 != 0) {
                moveDriveTrainAsync(-rx2 * 0.3, -rx2 * 0.3, rx2 * 0.3, rx2 * 0.3);
            } else {
                // Original complex movement logic
                if (y * y > x * x) {
                    if (y > 0) {
                        frontLeftPower = y + rx;
                        backLeftPower = y + rx + 0;
                        frontRightPower = y - rx;
                        backRightPower = y - rx;
                    } else {
                        frontLeftPower = y + rx;
                        backLeftPower = y + rx - 0;
                        frontRightPower = y - rx;
                        backRightPower = y - rx;
                    }
                } else {
                    if (x > 0) {
                        frontLeftPower = x + rx;
                        backLeftPower = -x + rx + misc;
                        frontRightPower = -x - rx;
                        backRightPower = x - rx;
                    } else if (x < 0) {
                        frontLeftPower = x + rx;
                        backLeftPower = -x + rx - misc;
                        frontRightPower = -x - rx;
                        backRightPower = x - rx;
                    } else {
                        frontLeftPower = x + rx;
                        backLeftPower = -x + rx;
                        frontRightPower = -x - rx;
                        backRightPower = x - rx;
                    }
                }
                
                // Set motor powers for manual mode
                moveDriveTrainWithPower(frontLeftPower, backLeftPower, frontRightPower, backRightPower);
            }

            // Servo and special button controls
            if (gamepad2.b) {
                specimenClaw.setPosition(CLAW_OPEN_POSITION);
            }
            
            if (gamepad2.right_trigger != 0) {
                specimenClaw.setPosition(FINAL_CLOSED_POSITION);

            }
            
            if (gamepad2.y) {
                specimenClaw.setPosition(CLAW_CLOSED_POSITION);
            }
            
            if (gamepad2.x) {
                // Combo action: close claw, move forward, and adjust arm
                specimenClaw.setPosition(FINAL_CLOSED_POSITION);
                moveDriveTrainAsync(-0.2, -0.2, -0.2, -0.2);
                moveArmAsync(-0.7, 100);  // Adjust duration/angle as needed
            }
            
            if (gamepad2.a) {
                // Combo action: close claw, move backward, and adjust arm
                specimenClaw.setPosition(FINAL_CLOSED_POSITION);
                moveDriveTrainAsync(0.2, 0.2, 0.2, 0.2);
                moveArmAsync(-0.7, 100);  // Adjust duration/angle as needed
            }
            
            
            

            // Arm control with boundary checks
            int currentArmPosition = armMotor.getCurrentPosition();
            double armPower = 0;

            // Right bumper (raise arm)
            if (gamepad2.right_bumper) {
                // Check if arm is below max position with soft buffer
                if (currentArmPosition < (ARM_MAX_POSITION - ARM_SOFT_LIMIT_BUFFER)) {
                    armPower = 0.6;
                }
            } 
            // Left bumper (lower arm)
            else if (gamepad2.left_bumper) {
                // Check if arm is above min position with soft buffer
                if (currentArmPosition > (ARM_MIN_POSITION + ARM_SOFT_LIMIT_BUFFER)) {
                    armPower = -0.6;
                }
            }

            // Set arm power with boundary protection
            armMotor.setPower(armPower);
            
            
        }
    }

    private void initializeHardware() {
        // Motor initialization
        frontLeftMotor = hardwareMap.dcMotor.get("backRightMotor");
        backLeftMotor = hardwareMap.dcMotor.get("frontRightMotor");
        frontRightMotor = hardwareMap.dcMotor.get("backLeftMotor");
        backRightMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        armMotor = hardwareMap.dcMotor.get("armMotor");
        specimenClaw = hardwareMap.servo.get("specimen claw");

        // Set motor directions
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set arm motor to use encoder
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        

        // Set motor modes and zero power behavior
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_USING_ENCODER);
        setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Ensure initial servo position is open
        specimenClaw.setPosition(CLAW_OPEN_POSITION);
    }

    private void moveDriveTrainAsync(double frontLeft, double backLeft, double frontRight, double backRight) {
        // Calculate strafing correction
        double correctedFrontLeftPower = frontLeft + 
            (frontLeftStrafeCorrectionFactor * frontLeft * STRAFE_CORRECTION_SCALE);
        double correctedBackLeftPower = backLeft + 
            (backLeftStrafeCorrectionFactor * backLeft * STRAFE_CORRECTION_SCALE);
        double correctedFrontRightPower = frontRight + 
            (frontRightStrafeCorrectionFactor * frontRight * STRAFE_CORRECTION_SCALE);
        double correctedBackRightPower = backRight + 
            (backRightStrafeCorrectionFactor * backRight * STRAFE_CORRECTION_SCALE);

        // Set motor powers
        frontLeftMotor.setPower(correctedFrontLeftPower);
        backLeftMotor.setPower(correctedBackLeftPower);
        frontRightMotor.setPower(correctedFrontRightPower);
        backRightMotor.setPower(correctedBackRightPower);
    }

    private void moveDriveTrainWithPower(double frontLeft, double backLeft, double frontRight, double backRight) {
        frontLeftMotor.setPower(frontLeft);
        backLeftMotor.setPower(backLeft);
        frontRightMotor.setPower(frontRight);
        backRightMotor.setPower(backRight);
    }

    // Modify moveArmAsync to respect position boundaries
    private void moveArmAsync(double power, int durationMs) {
        int currentPosition = armMotor.getCurrentPosition();
        
        // Check upper boundary
        if (power > 0 && currentPosition >= (ARM_MAX_POSITION - ARM_SOFT_LIMIT_BUFFER)) {
            return;  // Prevent moving further up
        }
        
        // Check lower boundary
        if (power < 0 && currentPosition <= (ARM_MIN_POSITION + ARM_SOFT_LIMIT_BUFFER)) {
            return;  // Prevent moving further down
        }
        
        // Move arm
        armMotor.setPower(power);
        sleep(durationMs);
        armMotor.setPower(0);
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
}