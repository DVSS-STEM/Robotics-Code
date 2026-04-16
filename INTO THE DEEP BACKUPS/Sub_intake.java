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

@TeleOp(name="1P sub intake")
public class Sub_intake extends LinearOpMode {
    // Motor and servo declarations
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotor armMotor2;
    private Servo sampleClaw;
    private Servo subRotation;

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
    
    private double rotate_pos = .64;
    private static double rest_rot_pos = 0.64;
    
    private int arm_pos = 0;
    
    // Button state tracking
    private boolean lastBButtonState = false;
    private boolean last_r = false;
    private boolean last_l = false;
    private boolean last_lb = false;
    private boolean last_rb = false;

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
            
            boolean up2 = false;
            boolean down2 = false;
            boolean right2 = false;
            boolean left2 = false;
            
            rx2 = 0;
            
            
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
            
            if (gamepad1.triangle && rotate_pos ==rest_rot_pos){
                sampleClaw.setPosition(0); // full open (not clear for rotation)
            } else if (gamepad1.triangle && rotate_pos !=rest_rot_pos){
                gamepad1.rumble(100);
            }else if (gamepad1.cross){
                sampleClaw.setPosition(1); //full close(grab from short side - 1.5") clear for rotation
            } else if (gamepad1.circle){
                sampleClaw.setPosition(.7); //open & clear for rotation
            } else if (gamepad1.square){
                sampleClaw.setPosition(.86); //close (grab from long side - 3.5") clear for rotation
            }
            
            if (sampleClaw.getPosition()>=.7){
                
                if (gamepad1.dpad_up){
                    rotate_pos = 1; //claw almost perpendicular to the right side (outside of bot)
                } else if (gamepad1.dpad_down){
                    rotate_pos = .3; //claw perpendicular to the left side (middle of the bot)
                } else if (gamepad1.dpad_right && !last_r){
                    rotate_pos +=.05;
                } else if (gamepad1.dpad_left && !last_l){
                    rotate_pos -=.05;
                } else if (gamepad1.share){
                    rotate_pos = rest_rot_pos;
                }
            } else if (sampleClaw.getPosition()<.7 && (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_right || gamepad1.dpad_left)){
                gamepad1.rumble(300);
                
            }
            
            /*
            if (gamepad1.right_bumper && armMotor2.getCurrentPosition()>-2866) {
                armMotor2.setPower(-0.5);
            } else if (gamepad1.left_bumper && armMotor2.getCurrentPosition()<0) {
                armMotor2.setPower(0.5);
            } else {
                armMotor2.setPower(0);
            }
            */
            
            if (gamepad1.right_bumper && armMotor2.getCurrentPosition()>-2866){
                armMotor2.setTargetPosition(armMotor2.getCurrentPosition()-200);
                armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor2.setPower(0.5);
            } else if (gamepad1.left_bumper && armMotor2.getCurrentPosition()<0){
                armMotor2.setTargetPosition(armMotor2.getCurrentPosition()+200);
                armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor2.setPower(0.5);
            } else if (gamepad1.options){
                armMotor2.setTargetPosition(-2750);
                armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor2.setPower(0.5);
            }
            subRotation.setPosition(rotate_pos);
            
            last_l = gamepad1.dpad_left;
            last_r = gamepad1.dpad_right;
            telemetry.addData("Sample claw:",sampleClaw.getPosition());
            telemetry.addData("Sub arm:", armMotor2.getCurrentPosition());
            telemetry.addData("",arm_pos);
            telemetry.update();
        }
    }

    private void initializeHardware() {
        // Motor initialization
        frontLeftMotor = hardwareMap.dcMotor.get("backRightMotor");
        backLeftMotor = hardwareMap.dcMotor.get("frontRightMotor");
        frontRightMotor = hardwareMap.dcMotor.get("backLeftMotor");
        backRightMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        armMotor2 = hardwareMap.dcMotor.get("armMotor2");
        
        sampleClaw = hardwareMap.servo.get("sample claw");
        subRotation = hardwareMap.servo.get("sub rotation");
       // sampleClaw.setDirection(Servo.Direction.REVERSE);
        

        // Set motor directions
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set motor modes and zero power behavior for all motors
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Ensure all motors have BRAKE behavior set
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
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


    private void setMotorRunModes(DcMotor.RunMode runMode) {
        frontLeftMotor.setMode(runMode);
        backLeftMotor.setMode(runMode);
        frontRightMotor.setMode(runMode);
        backRightMotor.setMode(runMode);
        armMotor2.setMode(runMode);
    }
}
