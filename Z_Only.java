package org.firstinspires.ftc.teamcode;

//imports
    //eventloop components
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

    //utilities
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.*;
import java.lang.Math;


@TeleOp
public class Z_Only extends LinearOpMode {
    // Servo-related variables
    Servo specimen_claw;
    boolean specimen_claw_open_status = true;
    
    // Servo position constants
    private static final double CLAW_OPEN_POSITION = 1.0;  // Adjust these values
    private static final double CLAW_CLOSED_POSITION = 0.0;  // to match your servo's range
    
    // Clamping power for continuous servo (if using CRServo)
    private static final double CLAW_CLAMPING_POWER = 0.5;  // Adjust as needed
    
    private static final double PASSIVE_CLAMP_POWER = 0.1;  // Low power to maintain grip
    
    // Button state tracking to prevent multiple toggles
    boolean lastBButtonState = false;

    @Override
    public void runOpMode(){
        // Motor initializations (unchanged)
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        
        // Initialize the servo
        specimen_claw = hardwareMap.servo.get("specimen claw");  // Make sure this matches your config
        
        DcMotor armMotor = hardwareMap.dcMotor.get("armMotor");
        
        // Motor direction and zero power behavior (unchanged)
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Ensure initial servo position is open
        specimen_claw.setPosition(CLAW_OPEN_POSITION);

        waitForStart();

        while (opModeIsActive()) {
            // Existing drive code (unchanged)
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;
            
            // Rest of your existing drive code... (keep the motor power calculations)

            boolean up2 = gamepad2.dpad_up; // Remember, Y stick value is reversed
            boolean down2 = gamepad2.dpad_down; // Counteract imperfect strafing
            boolean left2 = gamepad2.dpad_left;
            boolean right2 = gamepad2.dpad_right;
            
            double frontLeftPower;
            double backLeftPower;
            double frontRightPower;
            double backRightPower;
            
            double penis = -0.07;

            
            if (up2) {
                

                frontLeftPower = -0.2;
                backLeftPower = -0.2;
                frontRightPower = -0.2;
                backRightPower = -0.2;
            } else if (down2) {
                    frontLeftPower = 0.2;
                    backLeftPower = 0.2;
                    frontRightPower = 0.2;
                    backRightPower = 0.2;
                
            } else if (left2) {
                    frontLeftPower = 0.3;
                    backLeftPower = -0.3 - 0.05;
                    frontRightPower = -0.3;
                    backRightPower = 0.3;
                    
                    
            } else if (right2) {
                    frontLeftPower = -0.3;
                    backLeftPower = (0.3 + 0.05);
                    frontRightPower = 0.3;
                    backRightPower = -0.3;
                
            } else {
                if (y * y > x * x) {
                    if (y>0) {
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
                    if (x>0) {
                        frontLeftPower = x + rx;
                        backLeftPower = -x + rx + penis;
                        frontRightPower = -x - rx;
                        backRightPower = x - rx;
                    } else if (x<0) {
                        frontLeftPower = x + rx;
                        backLeftPower = -x + rx - penis;
                        frontRightPower = -x - rx;
                        backRightPower = x - rx;
                    } else {
                        frontLeftPower = x + rx;
                        backLeftPower = -x + rx;
                        frontRightPower = -x - rx;
                        backRightPower = x - rx;
                    }

                }
            }

            // Servo control logic
            if (gamepad2.b && !lastBButtonState) {
                // Toggle claw state
                if (specimen_claw_open_status) {
                    // Close the claw
                    specimen_claw.setPosition(CLAW_CLOSED_POSITION);
                    specimen_claw_open_status = false;
                } else {
                    // Open the claw
                    specimen_claw.setPosition(CLAW_OPEN_POSITION);
                    specimen_claw_open_status = true;
                }
            }
            
            // Store current B button state for next iteration
            lastBButtonState = gamepad2.b;
            
            // Existing motor power setting code
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            
            // Existing arm motor code
            if (gamepad2.right_bumper) {
                armMotor.setPower(0.5);
                sleep(10);
                armMotor.setPower(0);
            } else if (gamepad2.left_bumper) {
                armMotor.setPower(-0.5);
                sleep(10);
                armMotor.setPower(0);
            }
            
            if (!specimen_claw_open_status) {
                
                specimen_claw.setPosition(CLAW_CLOSED_POSITION-0.02);
            }
            
            if (gamepad2.y) {
                specimen_claw.setPosition(0);
            }
            
            telemetry.update();
        }
    }
}