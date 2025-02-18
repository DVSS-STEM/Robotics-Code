package org.firstinspires.ftc.teamcode;

//------------------------------------------------------------------
//imports
  //eventloop components
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
  //hardware components
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
  //utils
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.lang.Math;

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
  private static final double SPEC_OPEN_POSITION = 0.6;
  private static final double SPEC_CLOSED_POSITION = .9;
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
  private static boolean LAST_A_2 = false;
  private static boolean LAST_X_2 = false;
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
  private static int adjustor = 0; //incremental variable to adjust the ticks for the spec arm wall preset
      //Sub Claw Rotation
  private static double ROTATE_POS = ROTATE_REST; 

  @Override
  public void runOpMode(){
      initializeHardware();
      waitForStart();

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
              specClaw.setPosition(SPEC_OPEN_POSITION);
          }else if (gamepad2.y) {
              specClaw.setPosition(SPEC_CLOSED_POSITION);
          }

          if (gamepad2.right_bumper&& SpecArmMotor.getCurrentPosition()>-3150){
              SpecArmMotor.setTargetPosition(SpecArmMotor.getCurrentPosition()-100);
              SpecArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              SpecArmMotor.setPower(1.0);
              //SpecArmMotor.setPower(-0.5);
          } else if (gamepad2.left_bumper && SpecArmMotor.getCurrentPosition()<10){
              SpecArmMotor.setTargetPosition(SpecArmMotor.getCurrentPosition()+100);
              SpecArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              SpecArmMotor.setPower(1.0);
              //SpecArmMotor.setPower(0.5);
          } else if (gamepad2.start){
              SpecArmMotor.setTargetPosition(-480);
              SpecArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              SpecArmMotor.setPower(.75);
          } else if (gamepad2.back){
              SpecArmMotor.setTargetPosition(-3200);
              SpecArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              SpecArmMotor.setPower(0.75);
          } else if (gamepad2.left_stick_button){
              SpecArmMotor.setTargetPosition(0);
              SpecArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              SpecArmMotor.setPower(1.0);
          } else if (gamepad2.x && !LAST_X_2){
              SpecArmMotor.setTargetPosition(SpecArmMotor.getCurrentPosition()-10);
              SpecArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              SpecArmMotor.setPower(1);
          } else if (gamepad2.a && !LAST_A_2){
              SpecArmMotor.setTargetPosition(SpecArmMotor.getCurrentPosition()+10);
              SpecArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              SpecArmMotor.setPower(1);
          }
          
         /* if (gamepad2.right_stick_button){
              SpecArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          }
          */
          
          LAST_X_2 = gamepad2.x;
          LAST_A_2 = gamepad2.a;
          //------------------------------------------------------------------
          telemetry.addData("SubArm",SubArmMotor.getCurrentPosition());
          telemetry.addData("SpecArm",SpecArmMotor.getCurrentPosition());
          telemetry.addData("SpecClaw",specClaw.getPosition());
          telemetry.addData("adjustor",adjustor);
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
      subClaw.setPosition(1);
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
      r2 = Deadzone(gamepad2.right_stick_x);

      U2 = gamepad2.dpad_up;
      D2 = gamepad2.dpad_down;
      R2 = gamepad2.dpad_right;
      L2 = gamepad2.dpad_left;
      
      double denom = Math.max((Math.abs(y)+Math.abs(x)+Math.abs(r)),1);
      frontLeftPower = (y+x+r)/denom;
      frontRightPower = (y-x-r)/denom;
      backLeftPower = (y-x+r)/denom;
      backRightPower = (y+x-r)/denom;

      if (U2){
          frontLeftPower = -0.3;  frontRightPower = -0.3; backLeftPower = -0.3;   backRightPower = -0.3;
      } else if (D2){
          frontLeftPower = 0.3;   frontRightPower = 0.3;  backLeftPower = 0.3;    backRightPower = 0.3;
      } else if (R2){
          frontLeftPower = -0.3;  frontRightPower = 0.3;  backLeftPower = 0.3;    backRightPower = -0.3;
      } else if (L2){
          frontLeftPower = 0.3;   frontRightPower = -0.3; backLeftPower = -0.3;   backRightPower = 0.3;
      } else if (r2 !=0){
          frontLeftPower = -0.3*r2;   frontRightPower = 0.3*r2;   backLeftPower = -0.3*r2;    backRightPower = 0.3*r2;
      }

      frontLeftMotor.setPower(frontLeftPower);
      frontRightMotor.setPower(frontRightPower);
      backLeftMotor.setPower(backLeftPower);
      backRightMotor.setPower(backRightPower);

  }

  private static double Deadzone (double joystick_axis){
      if (Math.abs(joystick_axis)>0.055){
          return joystick_axis;
      } else{
          return 0;
      }
  }
}




