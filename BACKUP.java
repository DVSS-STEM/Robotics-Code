package org.firstinspires.ftc.teamcode;

//imports
    //eventloop components
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.*;

@TeleOp(name="Liam's Tick test")
public class BACKUP extends LinearOpMode {
    // Motor and servo declarations
    private DcMotorEx frontLeftMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backRightMotor;
    // here marks the beginning of the new additions and their resulting errors. NOTE: you have to add these into both the brain's I/O and the initializeHardware method
    private DcMotor intakeMotor;
    private DcMotorEx flyWheelMotor;
    
    private Servo hoodServo; // in case it isn't obvious this controls the "hood" that directs the balls out of the flywheels
    double hoodPos = 0; // tracks the hoods position
    private Servo rollServo; // controls the rolling of the balls into the flywheel
    //adjusting flywheel intake
    private boolean fly_on = false;
    private double fly_dir = 1.00;
    private double fly_pow = 1.00;
    private static boolean last_up = false;
    private static boolean last_down = false;
    boolean lastFlyToggle = false;
    boolean lastFlyReverse = false;
    boolean engaged = true;
    boolean lastRollToggle = false;
    /*
    double a = 10.0;
    double b = 10.0;
    double c = 0.05; // 0.05
    double d = 0.000357143;*/
    
    // end of new additions
    
    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeHardware();
        //flyWheelMotor.setVelocityPIDFCoefficients(a,b,c,d);
        rollServo.setPosition(0);

        waitForStart();

        while (opModeIsActive()) { //NOTE: I HAVE SWAPPED THE STICK X's FOR A QUICK FIX TO A PROBLEM. FIX IT AND THEN SWITCH THEM BACK!!!
            double y = -gamepad1.left_stick_y; // left stick x and y - strafe
            double rx = -gamepad1.left_stick_x;
            double x = -gamepad1.right_stick_x; // right stick x - rotate
            double ry = -gamepad1.right_stick_y*0.5; // right stick y - hood
            boolean leftTrigger = gamepad1.left_trigger > 0 || gamepad1.left_trigger < 0; // left trigger - intake
            boolean leftBumper = gamepad1.left_bumper;
            double frontLeftPower = 0; // we set the power to zero so it doesn't keep going the direction the rest of the way
            double backLeftPower = 0;
            double frontRightPower = 0;
            double backRightPower = 0;
            double intakePower = 0;
            double flywheelPower = 0;
            
            // DEAR ZACH: RIGHT NOW, THE MOTORS DON'T WORK, AND LEFT STICK X AND RIGHT STICK X DO THE OTHER PERSON'S JOB INSTEAD. PLEASE FIX
            if (y * y > x * x) { // is it better to do this or to use abs() instead?
                if (y > 0) { // chat what is the purpose of this conditional chat clip that
                    frontLeftPower = y + rx;
                    backLeftPower = y + rx;
                    frontRightPower = y - rx;
                    backRightPower = y - rx;
                } else {
                    frontLeftPower = y + rx;
                    backLeftPower = y + rx;
                    frontRightPower = y - rx;
                    backRightPower = y - rx;
                }
            } else {
                frontLeftPower = x + rx;
                backLeftPower = -x + rx;
                frontRightPower = -x - rx;
                backRightPower = x - rx;
                //frontLeftPower = x + rx;
                //backLeftPower = -x + rx;
                //frontRightPower = -x - rx;
                //backRightPower = x - rx;
            }
            
            
            
            
            
            // altering the hood rotation
            //if(ry * ry > 0) {
            //    hoodPos += ry * Math.PI / 180 / 8;
            //} else {
            //    hoodPos = hoodServo.getPosition();
            //}
            
            if(gamepad1.dpad_right || gamepad2.left_stick_x > 0) {
                hoodPos += 0.5* Math.PI / 180 / 8;
            } else if (gamepad1.dpad_left || gamepad2.left_stick_x < 0) {
                hoodPos -= 0.5 * Math.PI / 180 / 8;
            } else {
                hoodPos = hoodServo.getPosition();
            }
            
            if(hoodPos > Math.PI / 3.7){
                hoodPos = Math.PI / 3.7;
            }
            if(hoodPos < Math.PI / 6) {
                hoodPos = Math.PI / 6;
            }
            hoodServo.setPosition(hoodPos);
            
            // flywheel
            
            //if(rightTrigger > 0) {
            //    flywheelPower = rightTrigger;
            //    rollServo.setPosition(90);
            //    startTime = System.currentTimeMillis();
            //}
            
            
            if (gamepad1.a && !fly_on && !lastFlyToggle){
                fly_on = true;
            } else if (gamepad1.a && fly_on && !lastFlyToggle){
                fly_on = false;
            }
            
            /*
            if (gamepad1.b && !fly_on && fly_dir>0 && !lastFlyReverse){
                fly_dir = -1.0;
            } else if (gamepad1.b && !fly_on && fly_dir<0 && !lastFlyReverse){
                fly_dir = 1.0;
            } */
            
            if ((gamepad1.dpad_up || -gamepad2.left_stick_y > 0) && !last_up && (flyWheelMotor.getPower() < 1)){
                fly_pow += 70;
                
            } else if ((gamepad1.dpad_down || -gamepad2.left_stick_y < 0) && !last_down){
                fly_pow -= 10;
            }
            // ------------------------------------------------------------------------------
           
            if (fly_pow<0){
                fly_pow = 0;
            }
            if (fly_pow>5000){
                fly_pow = 5000;
            }
            
            if((gamepad1.right_bumper || gamepad2.right_bumper) && !lastRollToggle) {
                if(engaged) {
                    engaged = false;
                    rollServo.setPosition(Math.PI);
                } else {
                    engaged = true;
                    rollServo.setPosition(0);
                }
            }

            // intaking
            if(leftTrigger && !leftBumper){ // may not need the extra checks in conditional but I'm tryna code fast so if it stops it from erroring so be it
                intakePower = gamepad1.left_trigger;
            }
            
            if(leftBumper && !leftTrigger) {
                intakePower = -0.8;
            }
            // 1
            if(gamepad2.left_trigger != 0 || gamepad2.y) {
                hoodServo.setPosition(30 * Math.PI / 180);
                fly_pow = 1110;
            }
            // 2
            if(gamepad2.b) {
                hoodServo.setPosition(40.24 * Math.PI / 180);
                fly_pow = 1175;
            }
            // 3
            if(gamepad2.a) {
                hoodServo.setPosition(42.33 * Math.PI / 180);
                fly_pow = 1175;
            }
            // 4
            if(gamepad2.x || gamepad2.left_bumper) {
                hoodServo.setPosition(41.483 * Math.PI / 180);
                fly_pow = 1300;
            }
            // 5
            if(gamepad2.dpad_up) {
                hoodServo.setPosition(47.42 * Math.PI / 180);
                fly_pow = 1440;
            }
            // 6
            if(gamepad2.dpad_left) {
                hoodServo.setPosition(45.773 * Math.PI / 180);
                fly_pow = 1640;
            }
            // 7
            if(gamepad2.dpad_down) {
                hoodServo.setPosition(46.2504 * Math.PI / 180);
                fly_pow = 1760;
            }
            /*
            if(gamepad2.dpad_down) {
                hoodServo.setPosition(36 * Math.PI / 180);
                fly_pow = 0.9;
            }
            if(gamepad2.dpad_right) {
                hoodServo.setPosition(39 * Math.PI / 180);
                fly_pow = 0.9;
            } */
            
            if (fly_on){
                flywheelPower = 1.00*fly_dir*fly_pow;
            } else{
                flywheelPower = 0.00;
            }
            
            last_down = gamepad1.dpad_down || -gamepad2.left_stick_y < 0;
            last_up = gamepad1.dpad_up || -gamepad2.left_stick_y > 0;
            lastFlyToggle = gamepad1.a;
            lastFlyReverse = gamepad1.b;
            lastRollToggle = gamepad1.right_bumper;

            frontLeftMotor.setVelocity(frontLeftPower * 2550);
            backLeftMotor.setVelocity(backLeftPower * 2550);
            frontRightMotor.setVelocity(frontRightPower * 2550);
            backRightMotor.setVelocity(backRightPower * 2550);
            intakeMotor.setPower(intakePower);
            flyWheelMotor.setVelocity(flywheelPower); // notice different statement used. Sets motor to specific ticks/s. to convert rpm to ticks/s divide by 60.
            telemetry.addData("Power:", flyWheelMotor.getPower());
            telemetry.addData("RPM:", flyWheelMotor.getVelocity());
            telemetry.addData("Angle:", hoodPos * 180 / Math.PI);
            
            telemetry.addData("FL:", frontLeftMotor.getVelocity());
            telemetry.addData("BL:", backLeftMotor.getVelocity());
            telemetry.addData("FR:", frontRightMotor.getVelocity());
            telemetry.addData("BR:", backRightMotor.getVelocity());
            
            if(engaged) {
                telemetry.addLine("Blocker Engaged");
            } else {
                telemetry.addLine("Blocker Disengaged");
            }
            telemetry.update();
        }
    }

    private void initializeHardware() {
        // Motor initialization
        /*
        frontLeftMotor = hardwareMap.dcMotor.get("backRightMotor");
        backLeftMotor = hardwareMap.dcMotor.get("frontRightMotor");
        frontRightMotor = hardwareMap.dcMotor.get("backLeftMotor");
        backRightMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        */
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        flyWheelMotor = hardwareMap.get(DcMotorEx.class, "flyWheelMotor");
        hoodServo = hardwareMap.servo.get("hoodServo");
        rollServo = hardwareMap.servo.get("rollServo");

        // Set motor directions
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flyWheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // set flywheel to use encoder so we can use ticks for more reliable shooting
        flyWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // this one resets the motor encoder as is best practice
        flyWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Ensure all motors have BRAKE behavior set
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flyWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
