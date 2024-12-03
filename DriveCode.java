package org.firstinspires.ftc.teamcode;

//imports
    //eventloop components
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

    //hardware components
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

    //utilities
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.*;
import java.lang.Math;


@TeleOp
public class DriveCode extends LinearOpMode {
    boolean claw1 = false;
    boolean claw2 = false;
    double rot_pos = 0.5;
    int vpos = 0;
    int hpos = 0;
    CRServo sub_peck;
    Servo specimen_claw, sample_claw, sub_rotation;

    DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, vertical_1, vertical_2, horizontal, armMotor;

    @Override
    public void runOpMode(){

        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        vertical_1 = hardwareMap.dcMotor.get("Vertical 1");
        vertical_2 = hardwareMap.dcMotor.get("Vertical 2");
        horizontal = hardwareMap.dcMotor.get("Horizontal");
        armMotor = hardwareMap.dcMotor.get("armMotor");
        
        sub_peck = hardwareMap.crservo.get("peck");
        specimen_claw = hardwareMap.servo.get("specimen claw");
        sample_claw = hardwareMap.servo.get("sample claw");
        sub_rotation = hardwareMap.servo.get("rotation");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        specimen_claw.setDirection(Servo.Direction.REVERSE);
        sample_claw.setDirection(Servo.Direction.REVERSE);
        sub_rotation.setDirection(Servo.Direction.REVERSE);
        
        //set zero power behaviour to brake
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vertical_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vertical_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //sub_peck.setZeroPowerBehavior(CrServo.ZeroPowerBehavior.BRAKE);


       waitForStart();

        while (opModeIsActive()) {
            
             
            
            double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;
            
            

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = 0.9*(y + x + rx) / denominator;
            double backLeftPower = 0.9*(y - x + rx) / denominator;
            double frontRightPower = 0.9*(y - x - rx) / denominator;
            double backRightPower = 0.9*(y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            
            if (gamepad1.dpad_up){
                vpos+=12;
                vertical_1.setTargetPosition(vpos);
                vertical_2.setTargetPosition(vpos);
                vertical_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                vertical_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                vertical_1.setPower(1);
                vertical_2.setPower(1);
            } else if (gamepad1.dpad_down){
                vpos-=12;
                vertical_1.setTargetPosition(vpos);
                vertical_2.setTargetPosition(vpos);
                vertical_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                vertical_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                vertical_1.setPower(1);
                vertical_2.setPower(1);
            }
            
            if (gamepad1.dpad_right){
                horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                horizontal.setPower(1);
                sleep(25);
                horizontal.setPower(0);
            } else if (gamepad1.dpad_left){
                horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                horizontal.setPower(-1);
                sleep(25);
                horizontal.setPower(0);
            }
            
            if (gamepad1.right_bumper){
                armMotor.setPower(0.5);
                sleep(25);
                armMotor.setPower(0);
            } else if (gamepad1.left_bumper){
                armMotor.setPower(-0.5);
                sleep(25);
                armMotor.setPower(0);
            }
            
            if (gamepad1.right_trigger>0.3){
                sub_peck.setPower(1);
                sleep(25);
                sub_peck.setPower(0);
            } else if (gamepad1.left_trigger>0.3){
                sub_peck.setPower(-1);
                sleep(25);
                sub_peck.setPower(0);
            }
            
            if (gamepad1.a){
                specimen_claw.setPosition(0);
            } else if (gamepad1.b){
                specimen_claw.setPosition(0.4);
            }
            
            if (gamepad1.x){
                sample_claw.setPosition(0);
            } else if (gamepad1.y){
                sample_claw.setPosition(0.4);
            }
            /*
            if (gamepad1.a){
                if (claw1){
                   specimen_claw.setPosition(0.1); 
                   claw1 = false;
                } else if (!claw1){
                    specimen_claw.setPosition(0.5);
                    claw1 = true;
                }
            }
            
            if (gamepad1.b){
                if (claw2){
                   sample_claw.setPosition(0.1); 
                   claw2 = false;
                } else if (!claw2){
                    sample_claw.setPosition(0.5);
                    claw2 = true;
                }
            }
            /*
            if (gamepad1.x){
                rot_pos +=0.05;
                //sub_rotation.setPosition(0.4);
                telemetry.addData("Y:","1");
            } else if (gamepad1.y){
                rot_pos -=0.05;
                //sub_rotation.setPosition(0.6);
                telemetry.addData("X:","1");
            }
            */
            
            telemetry.update();
            
           
        }
    }
    
}