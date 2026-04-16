package org.firstinspires.ftc.teamcode;

//imports
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.*;

@TeleOp(name = "Z-Controls")
public class Z_T_controlls extends LinearOpMode{
   
    private ElapsedTime runtime = new ElapsedTime();

    //control variables
    boolean specimen_claw_open_status = false;
    boolean sample_claw_open_status = false;
    double claw_closed_position = 0.0;
    double claw_open_position = 0.5;
    double rotation_position = 0.5;

    boolean initial = true;
    double y = 0;
    double x = 0;
    double r = 0;

    public void runOpMode(){

        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");

        DcMotor vertical_1 = hardwareMap.dcMotor.get("Vertical 1");
        DcMotor vertical_2 = hardwareMap.dcMotor.get("Vertical 2");
        DcMotor horizontal = hardwareMap.dcMotor.get("Horizontal");
        DcMotor armMotor = hardwareMap.dcMotor.get("armMotor");

        //Servo specimen_claw = hardwareMap.servo.get("specimen claw");
        //Servo sample_claw = hardwareMap.servo.get("sample claw");
        //Servo sub_rotation = hardwareMap.servo.get("rotation");
        CRServo sub_peck = hardwareMap.crservo.get("peck");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        //specimen_claw.setDirection(Servo.Direction.REVERSE);
        //sample_claw.setDirection(Servo.Direction.REVERSE);
        //sub_rotation.setDirection(Servo.Direction.REVERSE);
        
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertical_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertical_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vertical_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vertical_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()){
           /* 
            if(initial){
                specimen_claw.setPosition(claw_closed_position);
                initial = false;
            }
            */
            y = gamepad1.left_stick_y;
            x = gamepad1.right_stick_x;
            r = gamepad1.left_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(r), 1);

            double FL_power = (y+x-r)/denominator;
            double FR_power = (y-x+r)/denominator;
            double BL_power = (y-x-r)/denominator;
            double BR_power = (y+x+r)/denominator;
        

            //controlling vertical extension up & down
            if (gamepad1.dpad_up){
                vertical_1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                vertical_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                vertical_1.setPower(1);
                vertical_2.setPower(1);
                sleep(25);
                vertical_1.setPower(0);
                vertical_2.setPower(0);
            }

            if (gamepad1.dpad_down){
                vertical_1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                vertical_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                vertical_1.setPower(-1);
                vertical_2.setPower(-1);
                sleep(25);
                vertical_1.setPower(0);
                vertical_2.setPower(0);
            }

            //controlling horizontal extension in & out
            if (gamepad1.dpad_right){
                horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                horizontal.setPower(1);
                sleep(25);
                horizontal.setPower(0);
            }

            if (gamepad1.dpad_left){
                horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                horizontal.setPower(-1);
                sleep(25);
                horizontal.setPower(0);
            }

            //controlling arm mechanism up & down
            if (gamepad1.left_bumper){
                armMotor.setPower(0.5);
                sleep(25);
                armMotor.setPower(0);
            }

            if (gamepad1.right_bumper){
                armMotor.setPower(-0.5);
                sleep(25);
                armMotor.setPower(0);
            }

            //control pecking mechanism up & down
            if (gamepad1.right_trigger>0.3){
                sub_peck.setPower(1);
                sleep(25);
                sub_peck.setPower(0);
            }

            if (gamepad1.left_trigger>0.3){
                sub_peck.setPower(-1);
                sleep(25);
                sub_peck.setPower(0);
            }
            /*
            //controlling claws opening and closing
            if (gamepad1.a || gamepad2.a){
                if (!sample_claw_open_status){
                    sample_claw.setPosition(claw_open_position);
                    sample_claw_open_status = true;
                } else if (sample_claw_open_status){
                    sample_claw.setPosition(claw_closed_position);
                    sample_claw_open_status = false;
                }
            }

            if (gamepad1.b || gamepad2.b){
                if(!specimen_claw_open_status){
                    specimen_claw.setPosition(claw_open_position);
                    specimen_claw_open_status = true;
                } else if (specimen_claw_open_status){
                    specimen_claw.setPosition(claw_closed_position);
                    specimen_claw_open_status = false;
                }
            }
            
            //adjusting submersible rotation
            if (gamepad1.y || gamepad2.y){
                rotation_position += 0.05;
            }
            if (gamepad1.x || gamepad2.x){
                rotation_position -=0.05;
            }

            sub_rotation.setPosition(rotation_position);
            */
            if (gamepad2.dpad_up){
                vertical_1.setTargetPosition(1);
                vertical_2.setTargetPosition(1);
                vertical_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                vertical_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                vertical_1.setPower(1);
                vertical_2.setPower(1);
            }
            
            if (gamepad2.dpad_down){
                vertical_1.setTargetPosition(0);
                vertical_2.setTargetPosition(0);
                vertical_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                vertical_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                vertical_1.setPower(1);
                vertical_2.setPower(1);
            }
            
            if (gamepad2.dpad_right){
                horizontal.setTargetPosition(1);
                horizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                horizontal.setPower(1);
            }
            
            if (gamepad2.dpad_left){
                horizontal.setTargetPosition(0);
                horizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                horizontal.setPower(1);
            }
            
            frontLeft.setPower(FL_power);
            frontRight.setPower(FR_power);
            backLeft.setPower(BL_power);
            backRight.setPower(BR_power);
            
            
            
            telemetry.addData("FL",FL_power);
            telemetry.addData("FR",FR_power);
            telemetry.addData("BL",BL_power);
            telemetry.addData("BR",BR_power);
            telemetry.update();
    
        }
    }

}