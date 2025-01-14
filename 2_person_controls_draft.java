package org.firstinspires.ftc.teamcode;

//imports
    //eventloop components
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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


@TeleOp(name = "Zeyad & T Waterloo");
public class ZT_Mecanum extends LinearOpMode {
    
    //control variables
        //misc.
    private ElapsedTime runtime = new ElapsedTime();
    boolean initial = true;
    final int TPR_R = 560; //TPR = ticks per revolution, R = REV
    final int TPR_H = 288; //TPR = ticks per revolution, H = Hex motor
    final double Arm_rot_deg = 145;
        //drive
    double x = 0; //left stick x value
    double y = 0; //left stick y value
    double r = 0; //right stick x value
    double denom = 0; //denominator for scaling power
    double FLP = 0; //front left power
    double FRP = 0; //front right power
    double BLP = 0; //back left power
    double BRP = 0; //back right power
    boolean forward_orientation = true;
        //claws
    boolean specimen_claw_open_status = false;
    boolean sample_claw_open_status = false;
    double claw_closed_position = 0.0;
    double claw_open_position = 0.5;
    double rotation_position = 0.5; //current submersible claw rotation
    double pecking_position = 0.0; //current pecking arm position
    double rotation_initial = 0.5; //rotation neutral position
    double peck_initial = 0; //peck initial position
    double peck_2 = 0.7; //peck standby position

        //extensions
    final int rack_teeth = 115;
    final int pinion_teeth = 32;
    final int extension_max = (int)(Math.floor(TPR_S*(rack_teeth/pinion_teeth))); //maximum extension in ticks -> int(floor([ticks/rev]*rack teeth / pinion teeth))
    final int extension_min = 0; // extension minomum in ticks
    final int arm_max = (int)(Math.floor(TPR_H*(Arm_rot_deg/360)*125/60));
    final int arm_min = 0;
    final List<Double> peck_positions = Arrays.asList(0,0.7,0.8);


    //hardware variables
        //Dc Motors
            //drivetrain
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
            //extensions
    private DcMotor vertical_1 = null;
    private DcMotor vertical_2 = null;
    private DcMotor horizontal = null;
            //arm
    private DcMotor armMotor = null;
        //Servos
            //Continuous
    //private CRServo sub_peck = null;
            //Angular
    private Servo specimen_claw = null;
    private Servo sample_claw = null;
    private Servo sub_rotation = null;
    private Servo sub_peck = null;

    @Override
    public void runOpMode(){
        telemetry.addData("Status:","Ready");
        telemetry.update();

        //initialize hardware variables
            //Dc Motors
                //drivetrain
        frontLeft = hardwareMap.get(DcMotor.class,"frontLeft");
        frontRight = hardwareMap.get(DcMotor.class,"frontRight");
        backLeft = hardwareMap.get(DcMotor.class,"backLeft");
        backRight = hardwareMap.get(DcMotor.class,"backRight");
                //extensions
        vertical_1 = hardwareMap.get(DcMotor.class,"Vertical 1");
        vertical_2 = hardwareMap.get(DcMotor.class,"Vertical 2");
        horizontal = hardwareMap.get(DcMotor.class,"Horizontal");
                //arm
        armMotor = hardwareMap.get(DcMotor.class,"armMotor");
            //Servos
                //continuous
       // sub_peck = hardwareMap.get(CRServo.class,"peck");
                //angular
        specimen_claw = hardwareMap.get(Servo.class,"specimen claw");
        sample_claw = hardwareMap.get(Servo.class,"sample claw");
        sub_rotation = hardwareMap.get(Servo.class,"rotation");
        sub_peck = hardwareMap.get(Servo.class,"peck");

        //set motor directions
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        specimen_claw.setDirection(Servo.Direction.REVERSE);
        sample_claw.setDirection(Servo.Direction.REVERSE);
        sub_rotation.setDirection(Servo.Direction.REVERSE);

        //reset encoders on all DcMotors
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertical_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertical_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set zero power behaviour to brake
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

            //ensure specimen claw is closed following the end of autonomous
            if(initial){
                specimen_claw.setPosition(claw_closed_position);
                initial = false;
            }
//--------------------------------------------- DRIVER CONTROLS ---------------------------------------------
            //mecanum drivetrain 
                //joystick inputs
            x = gamepad1.left_stick_x; 
            y = gamepad1.left_stick_y;
            r = gamepad1.right_stick_x;
            denom = Math.max((Math.abs(y)+Math.abs(x)+Math.abs(r)),1); //power values passed are clipped to 1; this denominator helps adjust for this issue.
                //power variables for each motor
            FLP = (y+x+r)/denom;
            FRP = (y-x-r)/denom;
            BLP = (y-x+r)/denom;
            BRP = (y+x-r)/denom;
                //setting power to the motors
            frontLeft.setPower(FLP);
            frontRight.setPower(FRP);
            backLeft.setPower(BLP);
            backRight.setPower(BRP);

            //vertical extension; see V_ext method (ln 312)
            if (gamepad1.dpad_up && !gamepad1.dpad_down){
                V_ext(1);
            }
            if (gamepad1.dpad_down && !gamepad1.dpad_up){
                V_ext(-1);
            }

            //horizontal extension; see H_ext method (ln 323)
            if (gamepad1.dpad_right && !gamepad1.dpad_left){
                H_ext(1);
            }
            if (gamepad1.dpad_left && !gamepad1.dpad_right){
                H_ext(-1);
            }

            //arm movement; see move_arm method (ln 331)
            if (gamepad1.right_bumper){
                move_arm(-1);
            }
            if (gamepad1.left_bumper){
                move_arm(1);
            }

            //sub arm (peck); adjusting the targeted positional value for the peck, actual movement called at end of opMode (ln 305)
            if (gamepad1.right_trigger>0.3){
                //pecking(1);
                pecking_position+=0.05;
            } else if (gamepad1.left_trigger>0.3){
                //pecking(-1);
                pecking_position-=0.05;
            }

            //claw controls
            if (gamepad1.a){
                if (!sample_claw_open_status){
                    sample_claw.setPosition(claw_open_position);
                    sample_claw_open_status = true;
                } else if (sample_claw_open_status){
                    sample_claw.setPosition(claw_closed_position);
                    sample_claw_open_status = false;
                }
            }
            if (gamepad1.b){
                if(!specimen_claw_open_status){
                    specimen_claw.setPosition(claw_open_position);
                    specimen_claw_open_status = true;
                } else if (specimen_claw_open_status){
                    specimen_claw.setPosition(claw_closed_position);
                    specimen_claw_open_status = false;
                }
            }

            //submersible claw rotation; adjusting the targeted positional value for the claw rotation, actual movement called at end of opMode (ln 304)
            if (gamepad1.y && !gamepad1.x){
                rotation_position += 0.05;
            } else if (gamepad1.x && !gamepad1.y){
                rotation_position -= 0.05;
            }

            // robot 90 rotation; values are placeholders, testing required for actual values
            if (gamepad1.RightStickButton){
                if(forward_orientation){
                    //if facing forward (away from driver), rotate 90 deg counter clockwise to face submersible
                    //left side motors rotate backward, right side motors rotate forward
                    frontLeft.setPower(-1);
                    frontRight.setPower(1);
                    backLeft.setPower(-1);
                    backRight.setPower(1);
                    sleep(50); //adjust value until rotation is 90 deg
                    frontLeft.setPower(0);
                    frontRight.setPower(0);
                    backLeft.setPower(0);
                    backRight.setPower(0);
                } else if (!forward_orientation){
                    //if facing sideways (towards the submersible), rotate 90 deg clockwise to face forward
                    //left side motors rotate forward, right side motors rotate backward
                    frontLeft.setPower(1);
                    frontRight.setPower(-1);
                    backLeft.setPower(1);
                    backRight.setPower(-1);
                    sleep(50); //adjust value until rotation is 90 deg
                    frontLeft.setPower(0);
                    frontRight.setPower(0);
                    backLeft.setPower(0);
                    backRight.setPower(0);
                }
            }

//--------------------------------------------- OPPERATOR CONTROLS ---------------------------------------------
            //opperator manual inputs
            if (gamepad2.a){
                if (!sample_claw_open_status){
                    sample_claw.setPosition(claw_open_position);
                    sample_claw_open_status = true;
                } else if (sample_claw_open_status){
                    sample_claw.setPosition(claw_closed_position);
                    sample_claw_open_status = false;
                }
            }
            if (gamepad2.b){
                if(!specimen_claw_open_status){
                    specimen_claw.setPosition(claw_open_position);
                    specimen_claw_open_status = true;
                } else if (specimen_claw_open_status){
                    specimen_claw.setPosition(claw_closed_position);
                    specimen_claw_open_status = false;
                }
            }

            if (gamepad2.y && !gamepad1.y && gamepad1.x){
                rotation_position += 0.05;
            } else if (gamepad2.x && !gamepad1.x && gamepad1.y){
                rotation_position -= 0.05;
            }

            //preset controls
                //vertical extension
            if (gamepad2.dpad_up && !gamepad2.dpad_down){
                //preset_V_ext(extension_max);
                specimen_hang();
            }
            if (gamepad2.dpad_down && !gamepad2.dpad_up){
                //preset_V_ext(extension_min);
                wall_grab();
            }

                //horizontal extension
            if (gamepad2.dpad_right && !gamepad2.dpad_left){
                //preset_H_ext(extension_max);
                sub_grab();
            }
            if (gamepad2.dpad_left && !gamepad2.dpad_right){
                //preset_H_ext(extension_min);
                sub_retraction();
            }

            sub_rotation.setPosition(rotation_position);
            sub_peck.setPosition(pecking_position);
            
        }


    }

    public void V_ext(int m){

        vertical_1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vertical_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vertical_1.setPower(m);
        vertical_2.setPower(m);
        sleep(25);
        vertical_1.setPower(0);
        vertical_2.setPower(0);
    }

    public void H_ext(int v){

        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setPower(v);
        sleep(25);
        horizontal.setPower(0);
    }

    public void move_arm(int n){

        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setPower(n);
        sleep(25);
        armMotor.setPower(0);
    }

    
    public void pecking(int p){

        sub_peck.setPower(p);
        sleep(25);
        sub_peck.setPower(0);
    }
    

    public void preset_V_ext(int tV){

        vertical_1.setTargetPosition(tV);
        vertical_2.setTargetPosition(tV);
        vertical_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vertical_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vertical_1.setPower(1);
        vertical_2.setPower(1);
    }

    public void preset_H_ext(int tH){

        horizontal.setTargetPosition(tH);
        horizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        horizontal.setPower(1);
    }

    public void preset_arm_ext(int tA){

        armMotor.setTargetPosition(tA);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
    }

    public void preset_peck_ext(double tP){

        pecking_position = tP;
    }
    
    public void wall_grab(){

        preset_V_ext(extension_min);
        preset_arm_ext(arm_min);
        specimen_claw.setPosition(claw_open_position);
        specimen_claw_open_status = true;
        rotation_position = rotation_initial;
        preset_peck_ext(peck_initial);
        sleep(50);
        preset_H_ext(extension_min);
    }

    public void specimen_hang(){

        preset_V_ext(extension_max);
        preset_arm_ext(arm_max);
        rotation_position = rotation_initial;
        preset_peck_ext(peck_initial);
        sleep(50);
        preset_H_ext(extension_min);
    }

    public void sub_grab(){

        preset_V_ext(extension_min);
        preset_arm_ext(arm_min);
        sample_claw.setPosition(claw_open_position);
        sample_claw = true;
        rotation_position = rotation_initial;
        preset_peck_ext(peck_2);
        sleep(50);
        preset_H_ext(extension_max);
    }

    public void sub_retraction(){

        preset_V_ext(extension_min);
        preset_arm_ext(arm_min);
        rotation_position = rotation_initial;
        preset_peck_ext(peck_initial);
        sleep(50);
        preset_H_ext(extension_min);
    }



}
