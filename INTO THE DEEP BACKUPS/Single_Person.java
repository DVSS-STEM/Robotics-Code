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

@TeleOp(name="Single Person")
public class Single_Person extends LinearOpMode {
    // Motor and servo declarations
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotor armMotor;
    private DcMotor armMotor2;
    private Servo specimenClaw;
    private Servo sampleClaw;

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
    private static final int ARM_MAX_POSITION = -(int)(ARM_COUNTS_PER_DEGREE * 250);  // Maximum rotation (e.g., 205 degrees)
    private static final int ARM_SOFT_LIMIT_BUFFER = (int)(ARM_COUNTS_PER_DEGREE * 26);  // 15-degree soft buffer

    // Servo position constants
    private static final double CLAW_OPEN_POSITION = 0.9;
    // For sample claw
    private static final double CLAW_OPEN_POSITION2 = 1.0;
    
    private static final double CLAW_CLOSED_POSITION = 0.5;
    private static final double FINAL_CLOSED_POSITION = 0.5;
    
    private static int direction = 1;

    // Button state tracking
    private boolean lastBButtonState = false;

    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeHardware();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("WallArmPosition", armMotor.getCurrentPosition());
            // Drive control calculations
            double y = gamepad1.left_stick_y*direction;
            double x = -gamepad1.left_stick_x*direction;
            double rx = -gamepad1.right_stick_x;
            
            double rx2 = gamepad2.right_stick_x;
            
            boolean up2 = gamepad1.dpad_up;
            boolean down2 = gamepad1.dpad_down;
            boolean left2 = gamepad1.dpad_left;
            boolean right2 = gamepad1.dpad_right;
            
            double frontLeftPower = 0;
            double backLeftPower = 0;
            double frontRightPower = 0;
            double backRightPower = 0;

            // Drive mode selection
            if (up2) {
                moveDriveTrainAsync(-0.2, -0.2, -0.2, -0.2);
            } else if (down2) {
                moveDriveTrainAsync(0.2, 0.2, 0.2, 0.2);
            } else if (left2) {
                moveDriveTrainAsync(0.3, -0.3, -0.3, 0.3);
            } else if (right2) {
                moveDriveTrainAsync(-0.3, 0.3, 0.3, -0.3);
            } else if (rx2 != 0) {
                moveDriveTrainAsync(-rx2 * 0.3, -rx2 * 0.3, rx2 * 0.3, rx2 * 0.3);
            } else {
                // Original complex movement logic
                if (y * y > x * x) {
                    if (y > 0) {
                        frontLeftPower = y + rx;
                        backLeftPower = y + rx;
                        frontRightPower = y - rx;
                        backRightPower = y - rx;
                    } else {
                        frontLeftPower = y + rx;
                        backLeftPower = y + rx;
                        frontRightPower = y - rx;
                        backRightPower = y - rx;
                    }
                } else {
                    frontLeftPower = x + rx;
                    backLeftPower = -x + rx;
                    frontRightPower = -x - rx;
                    backRightPower = x - rx;
                }
                
                // Set motor powers for manual mode
                moveDriveTrainWithPower(frontLeftPower, backLeftPower, frontRightPower, backRightPower);
            }

            // Servo and special button controls
            if (gamepad1.x) {
                specimenClaw.setPosition(CLAW_OPEN_POSITION);
            }
            
            
            if (gamepad1.y) {
                specimenClaw.setPosition(CLAW_CLOSED_POSITION);
            }
            
            if (gamepad1.b) {
                sampleClaw.setPosition(0.6);
            }
            
            if (gamepad1.a) {
                sampleClaw.setPosition(1);
            }
            
            
            if (gamepad1.right_bumper) {
                armMotor2.setPower(-0.5);
            } else if (gamepad1.left_bumper) {
                armMotor2.setPower(0.5);
            } else {
                armMotor2.setPower(0);
            }
            
            if (gamepad1.ps){
                if (direction == 1){
                    direction = -1;
                    gamepad1.rumble(1);
                    
                } else{
                    direction = 1;
                    gamepad1.rumble(1);
                }
            }
            

            // Arm control with boundary checks
            int currentArmPosition = armMotor.getCurrentPosition();
            double armPower = 0;
            
            // Right bumper (raise arm)
            if (gamepad1.left_trigger >0.3) {
                if (currentArmPosition < (ARM_MIN_POSITION - ARM_SOFT_LIMIT_BUFFER)) {
                    armPower = 0.6;
                }
            } 
            // Left bumper (lower arm)
            else if (gamepad1.right_trigger > 0.3) {
                if (currentArmPosition > (ARM_MAX_POSITION + ARM_SOFT_LIMIT_BUFFER)) {
                    armPower = -0.6;
                }
            }
            
            // Add safety check to prevent arm from going beyond limits
            if (currentArmPosition >= (ARM_MIN_POSITION + ARM_SOFT_LIMIT_BUFFER) && armPower < 0) {
                armPower = 0;
            } else if (currentArmPosition <= (ARM_MAX_POSITION - ARM_SOFT_LIMIT_BUFFER) && armPower > 0) {
                armPower = 0;
            }
            
            // Set arm power
            armMotor.setPower(armPower);
            telemetry.addData("Sample claw:",sampleClaw.getPosition());
            telemetry.update();
        }
    }

    private void initializeHardware() {
        // Motor initialization
        frontLeftMotor = hardwareMap.dcMotor.get("backRightMotor");
        backLeftMotor = hardwareMap.dcMotor.get("frontRightMotor");
        frontRightMotor = hardwareMap.dcMotor.get("backLeftMotor");
        backRightMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        armMotor = hardwareMap.dcMotor.get("armMotor");
        armMotor2 = hardwareMap.dcMotor.get("armMotor2");
        
        specimenClaw = hardwareMap.servo.get("specimen claw");
        sampleClaw = hardwareMap.servo.get("sample claw");
        

        // Set motor directions
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set arm motor to use encoder
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set motor modes and zero power behavior for all motors
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Ensure all motors have BRAKE behavior set
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Ensure initial servo position is open
        specimenClaw.setPosition(CLAW_CLOSED_POSITION);
        
    }

    private void moveDriveTrainAsync(double frontLeft, double backLeft, double frontRight, double backRight) {
        // Set motor powers directly without any correction factors
        frontLeftMotor.setPower(frontLeft);
        backLeftMotor.setPower(backLeft);
        frontRightMotor.setPower(frontRight);
        backRightMotor.setPower(backRight);
    }

    private void moveDriveTrainWithPower(double frontLeft, double backLeft, double frontRight, double backRight) {
        frontLeftMotor.setPower(frontLeft);
        backLeftMotor.setPower(backLeft);
        frontRightMotor.setPower(frontRight);
        backRightMotor.setPower(backRight);
    }

    private void moveArmAsync(double power, int durationMs) {
        int currentPosition = armMotor.getCurrentPosition();
        
        // Check upper boundary
        if (power > 0 && currentPosition >= (ARM_MAX_POSITION - ARM_SOFT_LIMIT_BUFFER)) {
            return;
        }
        
        // Check lower boundary
        if (power < 0 && currentPosition <= (ARM_MIN_POSITION + ARM_SOFT_LIMIT_BUFFER)) {
            return;
        }
        
        // Move arm
        armMotor.setPower(-power * 0.3);
        sleep(durationMs);
        armMotor.setPower(0);
    }

    private void setMotorRunModes(DcMotor.RunMode runMode) {
        frontLeftMotor.setMode(runMode);
        backLeftMotor.setMode(runMode);
        frontRightMotor.setMode(runMode);
        backRightMotor.setMode(runMode);
        armMotor.setMode(runMode);
        armMotor2.setMode(runMode);
    }
}