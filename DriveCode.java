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

    MotorManager manager;

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
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        manager = new MotorManager([
            MotorTPR("StudicaMotor", 356),
            MotorTPR("HexMotor", 560),
            MotorTPR("CoreHexMotor", 288)
        ]);
       waitForStart();

        while (opModeIsActive()) {
            manager.stopCompleted();
            telemetry.addData("----ACTIVE MOTORS----");
            for (DcMotor motor : manager.getPoweredMotors()) {
                telemetry.addData("Motor Name", motor.getDeviceName());
                telemetry.addData("Ticks remaining", motor.getTargetPosition() - motor.getCurrentPosition());
                telemetry.addData(",");
            }
            double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            

            if (gamepad1.dpad_right){
                manager.run(horizontal, 12, 1);
            } else if (gamepad1.dpad_left){
                manager.run(horizontal, -12, 1);
            }
            
            if (gamepad1.right_bumper){
                manager.run(armMotor, 6, 1);
            } else if (gamepad1.left_bumper){
                manager.run(armMotor, -6, 1);
            }
            
            if (gamepad1.right_trigger>0.3){
                manager.run(sub_peck, 12, 1);
            } else if (gamepad1.left_trigger>0.3){
                manager.run(sub_peck, -12, 1);
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
            
            telemetry.update();
            
           
        }
    }
    

    
}
