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
public class Test extends LinearOpMode {
    // Servo-related variables
    Servo specimen_claw;
    boolean specimen_claw_open_status = true;
    
    // Servo position constants
    private static final double CLAW_OPEN_POSITION = 0.6;  // Adjust these values
    private static final double CLAW_CLOSED_POSITION = 0.23;  // to match your servo's range
    private static final double final_closed = 0.15;
    // Clamping power for continuous servo (if using CRServo)
    private static final double CLAW_CLAMPING_POWER = 0.5;  // Adjust as needed
    

    // Button state tracking to prevent multiple toggles
    boolean lastBButtonState = false;

    @Override
    public void runOpMode(){
        // Motor initializations (unchanged)
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("frontRightMotor");
        
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("backLeftMotor");
        
        DcMotor backRightMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        
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
        boolean ultraclosed = false;

        waitForStart();

        while (opModeIsActive()) {
            // Existing drive code (unchanged)
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;
            
            double rx2 = gamepad2.right_stick_x;
            
            // Rest of your existing drive code... (keep the motor power calculations)

            boolean up2 = gamepad2.dpad_up; // Remember, Y stick value is reversed
            boolean down2 = gamepad2.dpad_down; // Counteract imperfect strafing
            boolean left2 = gamepad2.dpad_left;
            boolean right2 = gamepad2.dpad_right;
            
            double frontLeftPower;
            double backLeftPower;
            double frontRightPower;
            double backRightPower;
            
            double misc = -0.07;

            
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
                    
            } else if (rx2 != 0) {
                frontLeftPower = -rx2 * 0.3;
                backLeftPower = -rx2 * 0.3;
                frontRightPower = rx2 * 0.3;
                backRightPower = rx2 * 0.3;
                
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
                        backLeftPower = -x + rx + misc;
                        frontRightPower = -x - rx;
                        backRightPower = x - rx;
                    } else if (x<0) {
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
            }

            // Servo control logic
            if (gamepad2.b) {
                specimen_claw.setPosition(CLAW_OPEN_POSITION);
            }
            
            if (gamepad2.y) {
                specimen_claw.setPosition(CLAW_CLOSED_POSITION);
                
            }
            
            if (gamepad2.x) {
                //y function goes here

                specimen_claw.setPosition(final_closed);
                
                //up function goes here
                frontLeftPower = -0.2;
                backLeftPower = -0.2;
                frontRightPower = -0.2;
                backRightPower = -0.2;
                //left bumper function goes here
                armMotor.setPower(-0.7);
                armMotor.setPower(0);
                
            }
            if (gamepad2.a) {
                //y function goes here

                specimen_claw.setPosition(final_closed);
                
                //up function goes here
                frontLeftPower = 0.2;
                backLeftPower = 0.2;
                frontRightPower = 0.2;
                backRightPower = 0.2;
                //left bumper function goes here
                armMotor.setPower(-0.7);
                armMotor.setPower(0);
                
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
                armMotor.setPower(0.6);
                sleep(5);
                armMotor.setPower(0);
            } else if (gamepad2.left_bumper) {
                armMotor.setPower(-0.6);
                sleep(5);
                armMotor.setPower(0);
            }
            
            
            
            //macro for up, y, and left bumper moment. Functionalize all movements eventually.
            
            telemetry.update();
        }
    }
}