//package declaration
package org.firstinspires.ftc.teamcode;

//------------------------------------------------------------------
//imports
    //eventloop components
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
    //hardware components
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
    //utils
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//------------------------------------------------------------------

@TeleOp(name = "FTC Provincials Tele")

public class Provincials extends LinearOpMode{
    //Hardware component declarations
        //Drivetrain motors
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
        //Arm motors
    private DcMotor SpecArmMotor;
    private DcMotor SubArmMotor;
        //Claw servos
    private Servo specClaw;
    private Servo subClaw;
    private Servo subRotation;

    //Constants
        //Drivetrain Encoder Constants
    private static final double DRIVETRAIN_COUNTS_PER_MOTOR_REV = 28; //REV Core Hex Motor
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
        //Servo Position Constants
            //Specimen Claw
    private static final double SPEC_OPEN_POSITION = 0.9;
    private static final double SPEC_CLOSED_POSITION = 0.5;
            //Sub Claw
    private static final double SUB_OPEN_POSITION = 0;
    private static final double SUB_S_CLOSED_POSITION = 1; //full close (grab from short side -> 1.5"), clear for rotation
    private static final double SUB_L_CLOSED_POSITION = 0.86; // 'half' close (grab from long side -> 3.5"), clear for rotation
    private static final double SUB_CLEARED_POSITION = 0.7; //open, but cleared for rotation
            //Sub Claw Rotation
    private static final double ROTATE_R = 1;
    private static final double ROTATE_L = 0.3;
    private static final double ROTATE_REST = 0.63;
    private static final double ROTATE_INCREMENTS = 0.05;

    //Other Program Variables
        //Button State Tracking
    private static boolean LAST_R_1 = false;
    private static boolean LAST_L_1 = false;
        //Drivetrain Controls
    private static double y = 0; //Driver left joystick y
    private static double x = 0; //Driver left joystick x
    private static double r = 0; //Driver right joystick x
    private static double r2 = 0; //Operator right joystick x
    private static boolean U2 = false; //Operator d-pad fine movement forward
    private static boolean D2 = false; //Operator d-pad fine movement backward
    private static boolean R2 = false; //Operator d-pad fine movement right
    private static boolean L2 = false; //Operator d-pad fine movement left
        //Motor Power Variables
    private static double frontLeftPower = 0;
    private static double backLeftPower = 0;
    private static double frontRightPower = 0;
    private static double backRightPower = 0;
    private static double SpecArmPower = 0;
    private static double SubArmPower = 0;
        //Arm Active Positions
    private static int SpecArmPosition = 0;
    private static int SubArmPosition = 0;
        //Sub Claw Rotation
    private static double ROTATE_POS = ROTATE_REST; 

    @Override
    public void runOpMode(){
        initializeHardware();
        waitForStart();

        while (opModeIsActive()){
            //Drivetrain Controls
            y = gamepad1.left_stick_y;
            x = -gamepad1.left_stick_x;
            r = -gamepad1.right_stick_x;
            r2 = gamepad2.right_stick_x;

            U2 = gamepad2.dpad_up;
            D2 = gamepad2.dpad_down;
            R2 = gamepad2.dpad_right;
            L2 = gamepad2.dpad_left;

            if (U2){
                moveDriveTrainAsync(-0.3, -0.3, -0.3, -0.3);
            } else if (D2){
                moveDriveTrainAsync(0.3, 0.3, 0.3, 0.3);
            } else if (L2){
                moveDriveTrainAsync(0.3, -0.3, -0.3, 0.3);
            } else if (R2){
                moveDriveTrainAsync(-0.3, 0.3, 0.3, -0.3);
            } else if (r2 !=0){
                moveDriveTrainAsync(-r2*0.3, -r2*0.3, r2*0.3, r2*0.3);
            } else {
                if (y * y > x * x) {
                    if (y > 0) {
                        frontLeftPower = y + r;
                        backLeftPower = y + r;
                        frontRightPower = y - r;
                        backRightPower = y - r;
                    } else {
                        frontLeftPower = y + r;
                        backLeftPower = y + r;
                        frontRightPower = y - r;
                        backRightPower = y - r;
                    }
                } else {
                    frontLeftPower = x + r;
                    backLeftPower = -x + r;
                    frontRightPower = -x - r;
                    backRightPower = x - r;
                }
                moveDriveTrainWithPower(frontLeftPower, backLeftPower, frontRightPower, backRightPower);
            }
            //------------------------------------------------------------------
            
            //DRIVER (P1) Controls
            if (gamepad1.triangle && ROTATE_POS == ROTATE_REST){
                subClaw.setPosition(0); // full open (not clear for rotation)
            } else if (gamepad1.triangle && ROTATE_POS !=ROTATE_REST){
                gamepad1.rumble(100);
            }else if (gamepad1.cross){
                subClaw.setPosition(1); //full close(grab from short side - 1.5") clear for rotation
            } else if (gamepad1.circle){
                subClaw.setPosition(.7); //open & clear for rotation
            } else if (gamepad1.square){
                subClaw.setPosition(.86); //close (grab from long side - 3.5") clear for rotation
            }

            if (subClaw.getPosition()>=.7){
                
                if (gamepad1.dpad_up){
                    ROTATE_POS = 1; //claw almost perpendicular to the right side (outside of bot)
                    subRotation.setPosition(ROTATE_POS);
                } else if (gamepad1.dpad_down){
                    ROTATE_POS = .3; //claw perpendicular to the left side (middle of the bot)
                    subRotation.setPosition(ROTATE_POS);
                } else if (gamepad1.dpad_right && !LAST_R_1){
                    ROTATE_POS +=.05;
                    subRotation.setPosition(ROTATE_POS);
                } else if (gamepad1.dpad_left && !LAST_L_1){
                    ROTATE_POS -=.05;
                    subRotation.setPosition(ROTATE_POS);
                } else if (gamepad1.share){
                    ROTATE_POS = ROTATE_REST;
                    subRotation.setPosition(ROTATE_POS);
                }
            } else if (subClaw.getPosition()<.7 && (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_right || gamepad1.dpad_left)){
                gamepad1.rumble(300);
                
            }

            if (gamepad1.right_bumper && SubArmMotor.getCurrentPosition()>-2866){
                SubArmMotor.setTargetPosition(SubArmMotor.getCurrentPosition()-200);
                SubArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SubArmMotor.setPower(0.5);
            } else if (gamepad1.left_bumper && SubArmMotor.getCurrentPosition()<0){
                SubArmMotor.setTargetPosition(SubArmMotor.getCurrentPosition()+200);
                SubArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SubArmMotor.setPower(0.5);
            } else if (gamepad1.options){
                SubArmMotor.setTargetPosition(-2750);
                SubArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SubArmMotor.setPower(0.5);
            }
            //subRotation.setPosition(ROTATE_POS);
            
            LAST_L_1 = gamepad1.dpad_left;
            LAST_R_1 = gamepad1.dpad_right;
            //------------------------------------------------------------------
            /*
            SpecArmPosition = SpecArmMotor.getCurrentPosition();
            if (gamepad2.b) {
                specClaw.setPosition(SPEC_OPEN_POSITION);
            }else if (gamepad2.y) {
                specClaw.setPosition(SPEC_CLOSED_POSITION);
            }

            // Right bumper (raise arm)
            if (gamepad2.left_trigger !=0) {
                if (SpecArmPosition > (ARM_MAX_POSITION + ARM_SOFT_LIMIT_BUFFER)) {
                    SpecArmPower = 0.6;
                }
            } 
            // Left bumper (lower arm)
            else if (gamepad2.right_trigger !=0) {
                if (SpecArmPosition < (ARM_MIN_POSITION - ARM_SOFT_LIMIT_BUFFER)) {
                    SpecArmPower = -0.6;
                }
            }*/
            /*
            // Add safety check to prevent arm from going beyond limits
            if (SpecArmPosition <= (ARM_MIN_POSITION + ARM_SOFT_LIMIT_BUFFER) && SpecArmPower > 0) {
                SpecArmPower = 0;
            } else if (SpecArmPosition >= (ARM_MAX_POSITION - ARM_SOFT_LIMIT_BUFFER) && SpecArmPower > 0) {
                SpecArmPower = 0;
            }
            */
            SpecArmPosition = SpecArmMotor.getCurrentPosition();
            if (gamepad2.left_trigger > 0.3) {
                if (SpecArmPosition < (ARM_MAX_POSITION - ARM_SOFT_LIMIT_BUFFER)) {
                    SpecArmPower = -0.6;
                }
            } 
            // Left bumper (lower arm)
            else if (gamepad2.right_trigger > 0.3) {
                if (SpecArmPosition > (ARM_MIN_POSITION + ARM_SOFT_LIMIT_BUFFER)) {
                    SpecArmPower = 0.6;
                }
            }
            /*
            // Add safety check to prevent arm from going beyond limits
            if (SpecArmPosition <= (ARM_MIN_POSITION + ARM_SOFT_LIMIT_BUFFER) && SpecArmPower < 0) {
                SpecArmPower = 0;
            } else if (SpecArmPosition >= (ARM_MAX_POSITION - ARM_SOFT_LIMIT_BUFFER) && SpecArmPower > 0) {
                SpecArmPower = 0;
            }
            */

            SpecArmMotor.setPower(SpecArmPower);
            telemetry.addData("ARMpos",SpecArmPosition);
            telemetry.update();




        }
    }

    public void initializeHardware(){
        //Motor Initialization
        frontLeftMotor = hardwareMap.dcMotor.get("backRightMotor");
        backLeftMotor = hardwareMap.dcMotor.get("frontRightMotor");
        frontRightMotor = hardwareMap.dcMotor.get("backLeftMotor");
        backRightMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        SpecArmMotor = hardwareMap.dcMotor.get("armMotor");
        SubArmMotor = hardwareMap.dcMotor.get("armMotor2");

        //Servo Initialization
        specClaw = hardwareMap.servo.get("specimen claw");
        subClaw = hardwareMap.servo.get("sample claw");
        subRotation = hardwareMap.servo.get("sub rotation");

        //Set Motor Directions
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        SpecArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Set Motor Modes
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set Zero Power to Brake
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SpecArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SubArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Ensure Servos are in Initial Position
        specClaw.setPosition(SPEC_OPEN_POSITION);
        subClaw.setPosition(SUB_OPEN_POSITION);
        subRotation.setPosition(ROTATE_REST);
    }

    private void setMotorRunModes(DcMotor.RunMode runMode){
        frontLeftMotor.setMode(runMode);
        backLeftMotor.setMode(runMode);
        frontRightMotor.setMode(runMode);
        backRightMotor.setMode(runMode);
        SpecArmMotor.setMode(runMode);
        SubArmMotor.setMode(runMode);
    }

    private void moveDriveTrainWithPower(double frontLeft, double backLeft, double frontRight, double backRight) {
        frontLeftMotor.setPower(frontLeft);
        backLeftMotor.setPower(backLeft);
        frontRightMotor.setPower(frontRight);
        backRightMotor.setPower(backRight);
    }

    private void moveDriveTrainAsync(double frontLeft, double backLeft, double frontRight, double backRight) {
        // Set motor powers directly without any correction factors
        frontLeftMotor.setPower(frontLeft);
        backLeftMotor.setPower(backLeft);
        frontRightMotor.setPower(frontRight);
        backRightMotor.setPower(backRight);
    }

}