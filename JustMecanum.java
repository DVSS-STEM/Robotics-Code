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

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        
        //set zero power behaviour to brake
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


       waitForStart();

        while (opModeIsActive()) {
            
             
            //This isn't ideal but Zeyad is used to it.
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
            

    public void runToTicks(DcMotor motor, int ticks, double power) {
        vertical_1.setTargetPosition(ticks);
        vertical_2.setTargetPosition(ticks);
        vertical_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vertical_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vertical_1.setPower(power);
        vertical_2.setPower(power);
    }
    public void resetTicks(DcMotor motor) {
        motor.setMode(DcMotor.STOP_AND_RESET_ENCODER);
    }
    public 
    
}