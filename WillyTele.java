//package declaration
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import java.lang.Math; 

import java.util.List;
    //------------------------------------------------------------------

    @TeleOp(name = "DEV Provincials Tele")

    public class DEV_Provincials extends LinearOpMode{
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
        private static final int ARM_MAX_POSITION = (int)(ARM_COUNTS_PER_DEGREE * 250);  // Maximum rotation (e.g., 205 degrees)
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
        private static boolean LAST_Y_2 = false;
            //Drivetrain Controls
        private static double y = 0; //Driver left joystick y
        private static double x = 0; //Driver left joystick x
        private static double r = 0; //Driver right joystick x
        private static double y2 = 0; //Operator left joystick y
        private static double x2 = 0; //Operator left joystick x
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


        private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    
    /**
     * Variables to store the position and orientation of the camera on the robot. Setting these
     * values requires a definition of the axes of the camera and robot:
     *
     * Camera axes:
     * Origin location: Center of the lens
     * Axes orientation: +x right, +y down, +z forward (from camera's perspective)
     *
     * Robot axes (this is typical, but you can define this however you want):
     * Origin location: Center of the robot at field height
     * Axes orientation: +x right, +y forward, +z upward
     *
     * Position:
     * If all values are zero (no translation), that implies the camera is at the center of the
     * robot. Suppose your camera is positioned 5 inches to the left, 7 inches forward, and 12
     * inches above the ground - you would need to set the position to (-5, 7, 12).
     *
     * Orientation:
     * If all values are zero (no rotation), that implies the camera is pointing straight up. In
     * most cases, you'll need to set the pitch to -90 degrees (rotation about the x-axis), meaning
     * the camera is horizontal. Use a yaw of 0 if the camera is pointing forwards, +90 degrees if
     * it's pointing straight left, -90 degrees for straight right, etc. You can also set the roll
     * to +/-90 degrees if it's vertical, or 180 degrees if it's upside-down.
     */
    private Position cameraPosition = new Position(DistanceUnit.CM,
            -8.3, -16.0, 19.5, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            -90, -90, 180, 0);

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;



    double yRot = 0;
    double angleToGo = 0;
    double FLTicks = 0;
    double angoal = -14;
    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
        @Override
        public void runOpMode(){
            initializeHardware();
            waitForStart();
            initAprilTag();
            while (opModeIsActive()){
                //------------------------------------------------------------------
                controllerDriveInput();
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
                        ROTATE_POS = 1;//claw almost perpendicular to the right side (outside of bot)
                        subRotation.setPosition(ROTATE_POS); 
                    } else if (gamepad1.dpad_down){
                        ROTATE_POS = .3; //claw perpendicular to the left side (middle of the bot)
                        subRotation.setPosition(ROTATE_POS);
                    } else if (gamepad1.dpad_right && !LAST_R_1){
                        ROTATE_POS +=.05; //controlled incremental rotation of the claw towards the outside of the robot (clockwise)
                        subRotation.setPosition(ROTATE_POS);
                    } else if (gamepad1.dpad_left && !LAST_L_1){
                        ROTATE_POS -=.05; //controlled incremental rotation of the claw towards the inside of the robot (counter-clockwise)
                        subRotation.setPosition(ROTATE_POS);
                    } else if (gamepad1.share){
                        ROTATE_POS = ROTATE_REST; //return to initial positions
                        subRotation.setPosition(ROTATE_POS);
                    }
                } else if (subClaw.getPosition()<.7 && (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_right || gamepad1.dpad_left)){
                    gamepad1.rumble(100); //physical feedback if rotation not possible
                    
                }

                LAST_R_1 = gamepad1.dpad_right;
                LAST_L_1 = gamepad1.dpad_left;

                if (gamepad1.right_bumper && SubArmMotor.getCurrentPosition()>-2866){
                    SubArmMotor.setTargetPosition(SubArmMotor.getCurrentPosition()-200);
                    SubArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SubArmMotor.setPower(0.5);
                } else if (gamepad1.left_bumper && SubArmMotor.getCurrentPosition()<0){
                    SubArmMotor.setTargetPosition(SubArmMotor.getCurrentPosition()+200);
                    SubArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SubArmMotor.setPower(0.5);
                } else if (gamepad1.options){
                    SubArmMotor.setTargetPosition(-2775); //height for grabbing from floor
                    SubArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SubArmMotor.setPower(1);
                } else if (gamepad1.touchpad){
                    SubArmMotor.setTargetPosition(-1500);
                    SubArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SubArmMotor.setPower(1);
                } else if (gamepad1.right_stick_button){
                    SubArmMotor.setTargetPosition(-2600);
                    SubArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SubArmMotor.setPower(1);
                }
                //------------------------------------------------------------------
                //Operator (P2) controls
                if (gamepad2.b) {
                    specClaw.setPosition(SPEC_CLOSED_POSITION);
                }else if (gamepad2.a) {
                    specClaw.setPosition(SPEC_OPEN_POSITION);
                }

                if (gamepad2.left_bumper && SpecArmMotor.getCurrentPosition() > -3500) {
                    SpecArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    SpecArmMotor.setPower(-0.3);
                } else if (gamepad2.right_bumper && SpecArmMotor.getCurrentPosition() < -100) {
                    SpecArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    SpecArmMotor.setPower(0.3);
                } else if (gamepad2.start) {
                    SpecArmMotor.setTargetPosition(-480);
                    SpecArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SpecArmMotor.setPower(1);
                } else if (gamepad2.back) {
                    SpecArmMotor.setTargetPosition(-2950);
                    SpecArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SpecArmMotor.setPower(1);
                } else if (gamepad2.left_trigger > 0.3 && SpecArmMotor.getCurrentPosition() > -3500) {
                    SpecArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    SpecArmMotor.setPower(-0.5 * gamepad2.left_trigger);
                } else if (gamepad2.right_trigger > 0.3 && SpecArmMotor.getCurrentPosition() < -100) {
                    SpecArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    SpecArmMotor.setPower(0.5 * gamepad2.right_trigger);
                } else if (gamepad2.left_stick_button) {
                    SpecArmMotor.setTargetPosition(-3400);
                    SpecArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SpecArmMotor.setPower(1);
                } else if (gamepad2.right_stick_button) {
                    SpecArmMotor.setTargetPosition(500);
                    SpecArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SpecArmMotor.setPower(1);
                } else if (!gamepad2.left_bumper && !gamepad2.right_bumper && gamepad2.left_trigger <= 0.3 && gamepad2.right_trigger <= 0.3) {
                    SpecArmMotor.setPower(0);
                }
                
                if (gamepad2.x){
                    specClaw.setPosition(SPEC_OPEN_POSITION);
                    sleep(350);
                    SpecArmMotor.setTargetPosition(-2950);
                    SpecArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SpecArmMotor.setPower(1);
                    Asyncronous_FB(-1,100);
                }
                
                
                
                //NOAHNOAHNOAHNOAHNOAH
                
                if (gamepad2.y/* && !LAST_Y_2*/){
                    //SpecArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    //700 ticks to 90 degrees
                    double yaw = telemetryAprilTag();
                    telemetry.addData("YAW: ", yaw);
                    telemetry.addData("FLT: ", frontLeftMotor.getCurrentPosition());
                    telemetry.addData("FRT: ", frontRightMotor.getCurrentPosition());
                    telemetry.addData("BLT: ", backLeftMotor.getCurrentPosition());
                    telemetry.addData("BRT: ", backRightMotor.getCurrentPosition());
                    FLTicks = frontLeftMotor.getCurrentPosition();
                    angleToGo = yaw - angoal;
                    yRot = angleToGo / Math.abs(angleToGo);
                    
                }
                
                LAST_Y_2 = gamepad2.y;
                //------------------------------------------------------------------
                telemetry.addData("SubArm",SubArmMotor.getCurrentPosition());
                telemetry.addData("SpecArm",SpecArmMotor.getCurrentPosition());
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
            subClaw.setPosition(0.7);
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


        private void controllerDriveInput(){
            y = Deadzone(gamepad1.left_stick_y);
            x = Deadzone(-gamepad1.left_stick_x);
            r = Deadzone(-gamepad1.right_stick_x);
            y2 = Deadzone(gamepad2.left_stick_y);
            x2 = Deadzone(-gamepad2.left_stick_x);
            r2 = Deadzone(-gamepad2.right_stick_x)*0.3;
            
            if (yRot != 0 && gamepad2.y) {
                if (Math.abs(frontLeftMotor.getCurrentPosition()) > Math.abs((700/90)*angleToGo + FLTicks*yRot)) {
                    r = yRot;
                }
            }
            U2 = gamepad2.dpad_up;
            D2 = gamepad2.dpad_down;
            R2 = gamepad2.dpad_right;
            L2 = gamepad2.dpad_left;
            
            double denom = Math.max((Math.abs(y)+Math.abs(x)+Math.abs(r)),1);
            frontLeftPower = (y+x+r)/denom;
            frontRightPower = (y-x-r)/denom;
            backLeftPower = (y-x+r)/denom;
            backRightPower = (y+x-r)/denom;
            if (y==0 && x==0 && r==0){
                double denom2 = Math.max((Math.abs(y)+Math.abs(x)+Math.abs(r)),1);
                frontLeftPower = (y2+x2+r2)/denom2;
                frontRightPower = (y2-x2-r2)/denom2;
                backLeftPower = (y2-x2+r2)/denom2;
                backRightPower = (y2+x2-r2)/denom2;
            }

            if (U2){
                frontLeftPower = -0.3;  frontRightPower = -0.3; backLeftPower = -0.3;   backRightPower = -0.3;
            } else if (D2){
                frontLeftPower = 0.3;   frontRightPower = 0.3;  backLeftPower = 0.3;    backRightPower = 0.3;
            } else if (R2){
                frontLeftPower = -0.3;  frontRightPower = 0.3;  backLeftPower = 0.3;    backRightPower = -0.3;
            } else if (L2){
                frontLeftPower = 0.3;   frontRightPower = -0.3; backLeftPower = -0.3;   backRightPower = 0.3;
            } /*else if (r2 !=0){
                frontLeftPower = -0.3*r2;   frontRightPower = 0.3*r2;   backLeftPower = -0.3*r2;    backRightPower = 0.3*r2;
            }*/

            frontLeftMotor.setPower(frontLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backLeftMotor.setPower(backLeftPower);
            backRightMotor.setPower(backRightPower);
            
            telemetry.addData("FLP",frontLeftPower);
            telemetry.addData("FRP",frontRightPower);
            telemetry.addData("BLP",backLeftPower);
            telemetry.addData("BRP",backRightPower);

        }

        private static double Deadzone (double joystick_axis){
            if (Math.abs(joystick_axis)>0.055){
                return joystick_axis;
            } else{
                return 0;
            }
        }
        
        
        
        
        //NOAHNOAHNOAHNOAHNOAHNOAHNOAHNOAHNOAHNOAHNOAH
        private double telemetryAprilTag() {

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());
            double ret= 0;
            // Step through the list of detections and display info for each one.
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                            detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            detection.robotPose.getPosition().z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                    ret = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
            }   // end for() loop
    
            // Add "key" information to telemetry
            telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
            telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
            return ret;
        }   // end method telemetryAprilTag()
        private void initAprilTag() {

            // Create the AprilTag processor.
            aprilTag = new AprilTagProcessor.Builder()
    
                    // The following default settings are available to un-comment and edit as needed.
                    //.setDrawAxes(false)
                    //.setDrawCubeProjection(false)
                    //.setDrawTagOutline(true)
                    //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                    //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                    //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                    .setCameraPose(cameraPosition, cameraOrientation)
    
                    // == CAMERA CALIBRATION ==
                    // If you do not manually specify calibration parameters, the SDK will attempt
                    // to load a predefined calibration for your camera.
                    //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                    // ... these parameters are fx, fy, cx, cy.
    
                    .build();
    
            // Adjust Image Decimation to trade-off detection-range for detection-rate.
            // eg: Some typical detection data using a Logitech C920 WebCam
            // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
            // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
            // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
            // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
            // Note: Decimation can be changed on-the-fly to adapt during a match.
            //aprilTag.setDecimation(3);
    
            // Create the vision portal by using a builder.
            VisionPortal.Builder builder = new VisionPortal.Builder();
    
            // Set the camera (webcam vs. built-in RC phone camera).
            if (USE_WEBCAM) {
                builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
            } else {
                builder.setCamera(BuiltinCameraDirection.BACK);
            }
    
            // Choose a camera resolution. Not all cameras support all resolutions.
            //builder.setCameraResolution(new Size(640, 480));
    
            // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
            //builder.enableLiveView(true);
    
            // Set the stream format; MJPEG uses less bandwidth than default YUY2.
            //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
    
            // Choose whether or not LiveView stops if no processors are enabled.
            // If set "true", monitor shows solid orange screen if no processors enabled.
            // If set "false", monitor shows camera view without annotations.
            //builder.setAutoStopLiveView(false);
    
            // Set and enable the processor.
            builder.addProcessor(aprilTag);
    
            // Build the Vision Portal, using the above settings.
            visionPortal = builder.build();
    
            // Disable or re-enable the aprilTag processor at any time.
            //visionPortal.setProcessorEnabled(aprilTag, true);
    
        }   // end method initAprilTag()
        
        //NOAHNOAHNOAHNOAHNOAHNOAHNOAHNOAHNOAHNOAHNOAH
        
        private void Asyncronous_FB(double power, int its){
            frontLeftMotor.setPower(power);
            frontRightMotor.setPower(power);
            backLeftMotor.setPower(power);
            backRightMotor.setPower(power);
            sleep(its);
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            
        }
    }
