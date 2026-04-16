package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "DEV Autonomous", group = "DEV")
public class DEVAutonomous extends LinearOpMode {

    // Motor and servo declarations
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotor SpecArmMotor;
    private Servo specClaw;

    // Constants for movement
    private static final double DRIVETRAIN_COUNTS_PER_INCH = (28 * 20.0) / (4.0 * Math.PI); // From your teleop code
    private static final double STRAFE_COUNTS_PER_INCH = DRIVETRAIN_COUNTS_PER_INCH * 1.1; // Adjust for strafing
    private static final double ARM_COUNTS_PER_DEGREE = (288 * 4.0) / 360.0; // From your teleop code

    // Claw positions
    private static final double SPEC_OPEN_POSITION = 0.6;
    private static final double SPEC_CLOSED_POSITION = 0.9;

    @Override
    public void runOpMode() {
        initializeHardware();
        waitForStart();

                        // FIRST HANG

        // Step 1: Move forward, raise arm, close claw (concurrent)
        moveSpecArmToPosition(-1000);
        moveBackward(10);
        specClaw.setPosition(SPEC_CLOSED_POSITION);

        // Step 2: Move forward
        moveForward(10);

        // Step 3: Open claw
        specClaw.setPosition(SPEC_OPEN_POSITION); // Open the claw


                // PUSH THOSE BLOCKS!!!

        // Step 4: Move backwards and raise arm (concurrent)
        moveSpecArmToPosition(10);
        moveBackward(10);

        // Step 5: Move right
        strafeRight(10);

        // Step 6: Move forwards
        moveForward(10);

        // Step 7: Move back
        moveBackward(10);

        // Step 8: Move forwards
        moveForward(10);

        // Step 9: Move right
        strafeRight(10);

        // Step 10: Move back
        moveBackward(10);

        // Step 11: Move forwards
        moveForward(10);

        // Step 12: Move right
        strafeRight(10);

        // Step 13: Move back
        moveBackward(10);




                // ROBOT IS HANGING BROOOOOOOOS REPEAT x1

        // Step 14: Close claw, lower arm, move left (concurrent)
        specClaw.setPosition(SPEC_CLOSED_POSITION); // Close the claw
        moveSpecArmToPosition(10);
        strafeLeft(10);

        // Step 15: Move forward
        moveForward(10);

        // Step 16: Open claw
        specClaw.setPosition(SPEC_OPEN_POSITION); // Open the claw

        // Step 17: Move backwards and raise arm (concurrent)
        moveBackward(10);
        moveSpecArmToPosition(10);

        // Step 18: Move right
        strafeRight(10);


                // ROBOT IS HANGING BROOOOOOOOS REPEAT x2



        // Step 14: Close claw, lower arm, move left (concurrent)
        specClaw.setPosition(SPEC_CLOSED_POSITION); // Close the claw
        moveSpecArmToPosition(10);
        strafeLeft(10);

        // Step 15: Move forward
        moveForward(10);

        // Step 16: Open claw
        specClaw.setPosition(SPEC_OPEN_POSITION); // Open the claw

        // Step 17: Move backwards and raise arm (concurrent)
        moveBackward(10);
        moveSpecArmToPosition(10);

        // Step 18: Move right
        strafeRight(10);





                // ROBOT IS HANGING BROOOOOOOOS REPEAT x3


        // Step 14: Close claw, lower arm, move left (concurrent)
        specClaw.setPosition(SPEC_CLOSED_POSITION); // Close the claw
        moveSpecArmToPosition(10);
        strafeLeft(10);

        // Step 15: Move forward
        moveForward(10);

        // Step 16: Open claw
        specClaw.setPosition(SPEC_OPEN_POSITION); // Open the claw

        // Step 17: Move backwards and raise arm (concurrent)
        moveBackward(10);
        moveSpecArmToPosition(10);

        // Step 18: Move right
        strafeRight(10);






                // ROBOT IS HANGING BROOOOOOOOS REPEAT x4

        // Step 14: Close claw, lower arm, move left (concurrent)
        specClaw.setPosition(SPEC_CLOSED_POSITION); // Close the claw
        moveSpecArmToPosition(10);
        strafeLeft(10);

        // Step 15: Move forward
        moveForward(10);

        // Step 16: Open claw
        specClaw.setPosition(SPEC_OPEN_POSITION); // Open the claw

        // Step 17: Move backwards and raise arm (concurrent)
        moveBackward(10);
        moveSpecArmToPosition(10);

        // Step 18: Move right
        strafeRight(10);

    }

    private void initializeHardware() {
        // Motor Initialization
        frontLeftMotor = hardwareMap.dcMotor.get("backRightMotor");
        backLeftMotor = hardwareMap.dcMotor.get("frontRightMotor");
        frontRightMotor = hardwareMap.dcMotor.get("backLeftMotor");
        backRightMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        SpecArmMotor = hardwareMap.dcMotor.get("armMotor");

        // Servo Initialization
        specClaw = hardwareMap.servo.get("specimen claw");

        // Set Motor Directions
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        SpecArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set Motor Modes
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set Zero Power to Brake
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SpecArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Ensure Servo is in Initial Position
        specClaw.setPosition(SPEC_OPEN_POSITION);
    }

    private void setMotorRunModes(DcMotor.RunMode runMode) {
        frontLeftMotor.setMode(runMode);
        backLeftMotor.setMode(runMode);
        frontRightMotor.setMode(runMode);
        backRightMotor.setMode(runMode);
        SpecArmMotor.setMode(runMode);
    }

    private void moveForward(double inches) {
        int target = (int) (inches * DRIVETRAIN_COUNTS_PER_INCH);
        setMotorTargets(target, target, target, target);
        setMotorPowers(-0.5, -0.5, -0.5, -0.5);
        waitForMotorsToFinish();
    }

    private void moveBackward(double inches) {
        int target = (int) (-inches * DRIVETRAIN_COUNTS_PER_INCH);
        setMotorTargets(target, target, target, target);
        setMotorPowers(0.5, 0.5, 0.5, 0.5);
        waitForMotorsToFinish();
    }

    private void strafeRight(double inches) {
        int target = (int) (inches * STRAFE_COUNTS_PER_INCH);
        setMotorTargets(-target, target, target, -target);
        setMotorPowers(-0.5, 0.5, 0.5, -0.5);
        waitForMotorsToFinish();
    }
    
// stafe right, 

    private void strafeLeft(double inches) {
        int target = (int) (inches * STRAFE_COUNTS_PER_INCH);
        setMotorTargets(target, -target, -target, target);
        setMotorPowers(0.5, -0.5, -0.5, 0.5);
        waitForMotorsToFinish();
    }

    private void moveSpecArmToPosition(int targetTicks) {
        SpecArmMotor.setTargetPosition(targetTicks);
        SpecArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SpecArmMotor.setPower(0.75); // Adjust power as needed
        while (opModeIsActive() && SpecArmMotor.isBusy()) {
            // Wait until the arm reaches the target position
        }
        SpecArmMotor.setPower(0); // Stop the motor
    }

    private void setMotorTargets(int fl, int bl, int fr, int br) {
        frontLeftMotor.setTargetPosition(fl);
        backLeftMotor.setTargetPosition(bl);
        frontRightMotor.setTargetPosition(fr);
        backRightMotor.setTargetPosition(br);
        setMotorRunModes(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void setMotorPowers(double fl, double bl, double fr, double br) {
        frontLeftMotor.setPower(fl);
        backLeftMotor.setPower(bl);
        frontRightMotor.setPower(fr);
        backRightMotor.setPower(br);
    }

    private void waitForMotorsToFinish() {
        while (opModeIsActive() && (frontLeftMotor.isBusy() || backLeftMotor.isBusy() || frontRightMotor.isBusy() || backRightMotor.isBusy())) {
            // Wait until all motors are done
        }
        setMotorPowers(0, 0, 0, 0); // Stop motors
    }
}
