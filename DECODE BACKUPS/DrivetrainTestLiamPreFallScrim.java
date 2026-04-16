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

@TeleOp(name="Liam's Drivetrain Test")
public class DrivetrainTestLiamPreFallScrim extends LinearOpMode {
    // Motor and servo declarations
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    // here marks the beginning of the new additions and their resulting errors. NOTE: you have to add these into both the brain's I/O and the initializeHardware method
    private DcMotor intakeMotor;
    private DcMotor flyWheelMotor;
    private Servo hoodServo; // in case it isn't obvious this controls the "hood" that directs the balls out of the flywheels
    double hoodPos = 0; // tracks the hoods position
    private Servo rollServo;
    private Servo cameraServo;// controls the rolling of the balls into the flywheel
    //adjusting flywheel intake
    private boolean fly_on = false;
    private double fly_dir = 1.00;
    private double fly_pow = 1.00;
    private double batteryOffset = 0;
    boolean lastL3 = false;
    boolean lastR3 = false;
    private static boolean last_up = false;
    private static boolean last_down = false;
    boolean lastFlyToggle = false;
    boolean lastFlyReverse = false;
    boolean engaged = true;
    boolean lastRollToggle = false;
    
    // end of new additions

    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeHardware();
        rollServo.setPosition(0);
        cameraServo.setPosition(36 * Math.PI / 180);
        
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
                hoodPos += 0.1* Math.PI / 180 / 8;
            } else if (gamepad1.dpad_left || gamepad2.left_stick_x < 0) {
                hoodPos -= 0.1 * Math.PI / 180 / 8;
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
            
            if (gamepad1.b && !fly_on && fly_dir>0 && !lastFlyReverse){
                fly_dir = -1.0;
            } else if (gamepad1.b && !fly_on && fly_dir<0 && !lastFlyReverse){
                fly_dir = 1.0;
            }
            
            if ((gamepad1.dpad_up || -gamepad2.left_stick_y > 0) && fly_pow<1 && !last_up){
                fly_pow += 0.1;
                
            } else if ((gamepad1.dpad_down || -gamepad2.left_stick_y < 0) && fly_pow>0 && !last_down){
                fly_pow -=0.1;
            }
            // ------------------------------------------------------------------------------
            // todo: make the left stick on gamepad2 able to change hoodPos and power
            
            if (fly_pow<0){
                fly_pow = 0;
            } else if (fly_pow>1){
                fly_pow=1;
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
            if((leftTrigger || gamepad2.right_trigger != 0) && !leftBumper){ // also intakes when you are shooting to feed the flywheel. TEMPORARY!!!!!
                intakePower = 0.6;
            }
            
            if(leftBumper && !leftTrigger) {
                intakePower = -0.8;
            }
            
            if(gamepad2.left_trigger != 0) {
                hoodServo.setPosition(30 * Math.PI / 180);
                fly_pow = 0.7 + batteryOffset;
            }
            if(gamepad2.y) {
                hoodServo.setPosition(34 * Math.PI / 180);
                fly_pow = 0.7 + batteryOffset;
            }
            if(gamepad2.b) {
                hoodServo.setPosition(36 * Math.PI / 180);
                fly_pow = 0.7 + batteryOffset;
            }
            if(gamepad2.a) {
                hoodServo.setPosition(38 * Math.PI / 180);
                fly_pow = 0.7 + batteryOffset;
            }
            if(gamepad2.x) {
                hoodServo.setPosition(35 * Math.PI / 180);
                fly_pow = 0.8 + batteryOffset;
            }
            if(gamepad2.dpad_up || gamepad2.left_bumper) {
                hoodServo.setPosition(40 * Math.PI / 180);
                fly_pow = 0.8 + batteryOffset;
            }
            if(gamepad2.dpad_left) {
                hoodServo.setPosition(39 * Math.PI / 180);
                fly_pow = 0.8 + batteryOffset;
            }
            if(gamepad2.dpad_down) {
                hoodServo.setPosition(36 * Math.PI / 180);
                fly_pow = 0.9 + batteryOffset;
            }
            if(gamepad2.dpad_right) {
                hoodServo.setPosition(39 * Math.PI / 180);
                fly_pow = 0.9 + batteryOffset;
            }
            if(gamepad2.left_stick_button && !lastL3) {
                fly_pow -= 0.02;
                batteryOffset -= 0.02;
            }
            if(gamepad2.right_stick_button && !lastR3) {
                fly_pow += 0.02;
                batteryOffset += 0.02;
            }
            
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
            lastL3 = gamepad2.left_stick_button;
            lastR3 = gamepad2.right_stick_button;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            intakeMotor.setPower(intakePower);
            flyWheelMotor.setPower(flywheelPower);
            
            telemetry.addData("Power:", flywheelPower);
            telemetry.addData("Angle:", hoodPos * 180 / Math.PI);
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
        
        frontLeftMotor = hardwareMap.dcMotor.get("backRightMotor"); // gang wtf
        backLeftMotor = hardwareMap.dcMotor.get("frontRightMotor"); // gang wtf
        frontRightMotor = hardwareMap.dcMotor.get("backLeftMotor"); // gang wtf
        backRightMotor = hardwareMap.dcMotor.get("frontLeftMotor"); // gang wtf
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        flyWheelMotor = hardwareMap.dcMotor.get("flyWheelMotor");
        hoodServo = hardwareMap.servo.get("hoodServo");
        rollServo = hardwareMap.servo.get("rollServo");
        cameraServo = hardwareMap.servo.get("cameraServo");
        

        // Set motor directions
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    //frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flyWheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Ensure all motors have BRAKE behavior set
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flyWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}