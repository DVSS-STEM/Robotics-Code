package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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



import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous
public class AprilTagAutonomous extends LinearOpMode {
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

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    
    //----------------------------------------------------------------------------//
    
    //Starting position
    double[][] currentGoalPos = new double[3][2];
    //current goal position
    
    public void runOpMode() {

        initAprilTag();
        initializeHardware();
        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();
        

        //change first num to number of steps
        int currentstep = 0;
        double[][] positionOrder = new double[3][3];
        positionOrder[0] = new double[] {21, -35, 0};
        positionOrder[1] = new double[] {23, -40, 0};
        positionOrder[2] = new double[] {47, -40, 0};
        changeTarget(currentGoalPos, positionOrder[0]);
        while (opModeIsActive()) {
            currentGoalPos = telemetryAprilTag(currentGoalPos);
            if(moveToGoal()) {
                currentstep += 1;
                changeTarget(currentGoalPos, positionOrder[currentstep]);
            }

            // Push telemetry to the Driver Station.
            telemetry.update();

            // Save CPU resources; can resume streaming when needed.
            if (gamepad1.dpad_down) {
                visionPortal.stopStreaming();
            } else if (gamepad1.dpad_up) {
                visionPortal.resumeStreaming();
            }

            // Share the CPU.
            sleep(20);
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end method runOpMode()
    //are any drive motors busy
    private boolean anyBusy() {
        if (
            frontLeftMotor.isBusy() ||
            frontRightMotor.isBusy() ||
            backLeftMotor.isBusy() ||
            backRightMotor.isBusy()
            ) {
                return true;
            }
        return false;
    }
    //Move inexorably towards goal by increments
    private boolean moveToGoal() {
        if (!anyBusy()) {
            setDriveRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //if x is not at goal
            double diff = largeDiff(currentGoalPos[0][0], currentGoalPos[0][1]);
            if (diff != 0) {
                setDriveTicks((int)(diff * 45));
                setDriveRunModes(DcMotor.RunMode.RUN_TO_POSITION);
                currentGoalPos[0][0]+=diff;
                setDrivePower(1,0,0);
                return false;
            }
            //if y is not at goal
            diff = largeDiff(currentGoalPos[1][0], currentGoalPos[1][1]);
            if (diff != 0) {
                setDriveTicks((int)(diff * 45));
                setDriveRunModes(DcMotor.RunMode.RUN_TO_POSITION);
                currentGoalPos[1][0]+=diff;
                setDrivePower(0,1,0);
                return false;
            }
            return true;
        }else{
            return false;
        }
    }
    private static void changeTarget(double[][]positions, double[] arr) {
        positions[0][1] = arr[0];
        positions[1][1] = arr[1];
        positions[2][1] = arr[2];
        //return retArr;
    }
    private double largeDiff(double n1, double n2) {
        if (Math.abs(Math.abs(n1) - Math.abs(n2)) > 1) {
            double diff = n1 - n2;
            return diff / Math.abs(diff);
        }
        return 0;
    }
    private void setDriveTicks(int ticks){
        frontLeftMotor.setTargetPosition(ticks);
        backLeftMotor.setTargetPosition(ticks);
        frontRightMotor.setTargetPosition(ticks);
        backRightMotor.setTargetPosition(ticks);
    }
    private void setDrivePower(double x, double y, double rotation) {
        x = Math.max(-1, Math.min(1, x));
        y = Math.max(-1, Math.min(1, y));
        rotation = Math.max(-1, Math.min(1, rotation));
    
        double frontLeftPower = y + x + rotation;
        double backLeftPower = y - x + rotation;
        double frontRightPower = y - x - rotation;
        double backRightPower = y + x - rotation;
        if (Math.abs(frontLeftPower) > 1 || Math.abs(backLeftPower) > 1 || Math.abs(frontRightPower) > 1 || Math.abs(backRightPower) > 1) {
            double max = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backRightPower));
    
            frontLeftPower /= max;
            backLeftPower /= max;
            frontRightPower /= max;
            backRightPower /= max;
        }
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }
    private void setDriveRunModes(DcMotor.RunMode runMode){
        frontLeftMotor.setMode(runMode);
        backLeftMotor.setMode(runMode);
        frontRightMotor.setMode(runMode);
        backRightMotor.setMode(runMode);
    }
    private void setMotorRunModes(DcMotor.RunMode runMode){
        frontLeftMotor.setMode(runMode);
        backLeftMotor.setMode(runMode);
        frontRightMotor.setMode(runMode);
        backRightMotor.setMode(runMode);
        SpecArmMotor.setMode(runMode);
        SubArmMotor.setMode(runMode);
    }
    public void initializeHardware() {
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
    /**
     * Initialize the AprilTag processor.
     */
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

    /**
     * Add telemetry about AprilTag detections.
     */
    private double[][] telemetryAprilTag(double[][] cope) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        if (currentDetections.size() == 0) {
            return cope;
        }
        double[][] retArr = new double[3][2];
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
                retArr[0][0] = detection.robotPose.getPosition().x;
                retArr[1][0] = detection.robotPose.getPosition().y;
                retArr[2][0] = detection.robotPose.getPosition().z;
                retArr[0][1] = cope[0][1];
                retArr[1][1] = cope[1][1];
                retArr[2][1] = cope[2][1];
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        return retArr;
    }   // end method telemetryAprilTag()
    // todo: write your code here
}