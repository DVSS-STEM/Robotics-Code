/* Copyright (c) 2024 Dryw Wade. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name="RedRight", group="Autonomous")
public class RedRightAutonomous extends LinearOpMode {

    // Hardware used
    private DcMotor armMotor = null;
    private Servo gripperServo = null;


    private ElapsedTime runtime = new ElapsedTime();


    //Map taken from Will's MecanumTeleOp
    // Declare our motors
    // Make sure your ID's match your configuration
    DcMotor frontLeft, backLeft, frontRight, backRight;
    DcMotor horz;
    //Servo claw_horz, claw_vertical, claw_horz_rot;
    CRServo peck;
    double rot_inc = 0;
    DcMotor vert1, vert2; 
    //Where we think we are
    /*
    * All of the following assumes:
    * 1. Looking at the field from the red wall.
    * 2. Origin is the center of the field.
    * Robot is positioned as if it were its center. 
    * If it were centered on the origin, it would be at 0, 0
    * X axis runs from -72 to 72 as you go from left to right
    * Y axis runs from -72 to 72 as you go further away from red wall.
    * (Both of the above as per https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html)
    * Facing is with 0 as pointing straight up Y axis (towards positive infinity), 360 degrees points you straight down again.
    * 
    */
    //BASICALLY CONFIG BUT CAN'T BE CONSTRUED TO BE CHEATING
    boolean isRed = true;
    boolean isLeft = false;
    double xPos = -15;
    double yPos = -63;
    
    //rotation as described above
    double facing = 0;

    // Set target position. NEEDS TO BE CALIBRATED
    double ticksPerInch = 45.0; 

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
    //FILL THIS v----------------------------------------------------------------------------------------
    private Position cameraPosition = new Position(DistanceUnit.CM,
            13, 1.25, 44.5, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            -90, -90, -90, 0);
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        if (isRed == true) {
            yPos *= -1;
            xPos *= -1;
        }
        if (isLeft) {
            xPos *= -1;
        }
        horz = hardwareMap.dcMotor.get("Horizontal");
        peck = hardwareMap.crservo.get("peck");
        //claw_horz = hardwareMap.servo.get("sample claw");
        //claw_vertical = hardwareMap.servo.get("specimen claw");
        //claw_horz_rot = hardwareMap.servo.get("rotation");
        vert1 = hardwareMap.dcMotor.get("Vertical 1");
        vert2 = hardwareMap.dcMotor.get("Vertical 2");
        armMotor = hardwareMap.dcMotor.get("armMotor");
        horz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vert1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vert2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Wait for start
        initAprilTag();
        waitForStart();

        //All code written from Red perspective, toPos automatically flips
        if (opModeIsActive()) {
            runtime.reset();
            //go to specimen mount position. FILL CLAWMOUNTDISTANCE WITH HOW FAR NEEDED TO MOUNT CLAW
            if (!isLeft) {
                toPos(-15, -48 - 10);
                //MOUNT PIECE
                
                //prepare to go to push specimen pieces
                toPos(36, -48 - 10);
                //continue above. 12, not 15, to try and account for error
                toPos(36, -12);
                //position in front of first thing to push
                toPos(48, -12);
                //push
                toPos(48, -70);
                //return
                toPos(48, -12);
                //on to the next
                toPos(56, -12);
                //push
                toPos(56, -70);
                //return
                toPos(56, -12);
                //on to last, might slip but can't center
                toPos(60, -12);
                //push
                toPos(60, -70);
                //return
                toPos(60, -40);
                //I really like using the left border of observation corner tile as a potential grab spot, easier to align for mounter
                //sleep(TIME TO CLIP 3)
                toPos(48, -10);
                toPos(48, -60);
            } else {
                //go to specimen mount position. FILL CLAWMOUNTDISTANCE WITH HOW FAR NEEDED TO MOUNT CLAW
                toPos(-9, 10);
            }
        }
    }

    //Only works for straight lines because I'm lazy and our mecanums are bad at diagonals. Not hard to change, but no reason to now.
    //Maybe check the z value of robot. If off, camera is stupid and bad and should not be trusted.
    public void toPosCam(double xDest, double yDest) {
        //this should work but cast to int in case
        //CAMERAPOS
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        double cameraX = 0;
        double cameraY = 0;
        for (AprilTagDetection detection : currentDetections) {
            cameraX += detection.robotPose.getPosition().x;
            cameraY += detection.robotPose.getPosition().y;
        }   // end for() loop
        cameraX /= currentDetections.size();
        cameraY /= currentDetections.size();
        xPos = cameraX;
        yPos = cameraY;
        /*if (Math.abs(Math.abs(xPos) - Math.abs(cameraX)) < 1 && Math.abs(Math.abs(yPos) - Math.abs(cameraY)) > 1) {

        }*/
        while ((int) xDest != (int) xPos && (int) yDest != (int) yPos) {
            //CAMERAPOS
            if ((int) xDest > (int) xPos) {
                toPos(xPos + 1, yPos);
            } else {
                toPos(xPos - 1, yPos);
            }
            if ((int) yDest > (int) yPos) {
                toPos(xPos, yPos + 1);
            } else {
                toPos(xPos, yPos - 1);
            }
        }
    }
    public void toPos(double xDest, double yDest) {
        // Reset encoders
        frontLeft = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeft = hardwareMap.dcMotor.get("backLeftMotor");
        frontRight = hardwareMap.dcMotor.get("frontRightMotor");
        backRight = hardwareMap.dcMotor.get("backRightMotor");
        horz = hardwareMap.dcMotor.get("Horizontal");
        peck = hardwareMap.crservo.get("peck");
        //claw_horz = hardwareMap.servo.get("sample claw");
        //claw_vertical = hardwareMap.servo.get("specimen claw");
        //claw_horz_rot = hardwareMap.servo.get("rotation");
        vert1 = hardwareMap.dcMotor.get("Vertical 1");
        vert2 = hardwareMap.dcMotor.get("Vertical 2");
        armMotor = hardwareMap.dcMotor.get("armMotor");
        horz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vert1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vert2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        // Set to use encoders
        double x;
        double y;
        if (isRed) {
            yDest *= -1;
        } else {
            xDest *= -1;
        }
        x = xDest - xPos;
        y = yDest - yPos;
        telemetry.addData("XandY X: ", String.valueOf(x) + " Y: " + String.valueOf(y));
        //find distance
        double hyp = Math.hypot(x, y);
        if (x < 0 ^ y < 0) {
            hyp *= -1;
        }
        telemetry.addData("HYPOT: ", String.valueOf(hyp));
        //x is opposite, y is adjacent
        double angle = Math.atan(y/x);
        if (angle <= 0) {
            angle*=-1;
            angle = Math.PI*2 - angle;
        }
        angle -= Math.PI/2;
        telemetry.addData("ANGLE", Math.toDegrees(angle));
        
        telemetry.addData("ANGLEDONE", Math.toDegrees(angle));
        double PIover4 = Math.PI/4;
        telemetry.addData("PIOVER4", PIover4);
        double FRBL = Math.sin(angle - PIover4);
        double FLBR = Math.sin(angle + PIover4);
        telemetry.addData("PRELIMVECT: ", String.valueOf(FRBL) + "  " + String.valueOf(FLBR));
        
        
        //if true, normalize by FRBL, else normalize by FLBR
        double normalize = Math.abs(FLBR);
        if (FLBR > FRBL) {
            normalize = Math.abs(FRBL);
        }
        FLBR = FLBR/normalize;
        FRBL = FRBL/normalize;
        if (Math.abs(FLBR) > 1) {
            FRBL /= Math.abs(FLBR);
            FLBR /= Math.abs(FLBR);
        }
        if (Math.abs(FRBL) > 1) {
            FLBR /= Math.abs(FRBL);
            FRBL /= Math.abs(FRBL);
        }
        //FUCK floating points
        if (Math.abs(FLBR) < .0001) {
            FLBR = 0;
        }
        if (Math.abs(FRBL) < .0001) {
            FRBL = 0;
        }
        telemetry.addData("VECTORS: ", String.valueOf(FLBR) + " " + String.valueOf(FRBL) + String.valueOf(FRBL == 1));
        
        while(opModeIsActive()) {
            telemetry.update();
            //comment break out for telemetry
            break;
        }
        frontLeft.setTargetPosition((int)(ticksPerInch * hyp/FLBR));
        frontRight.setTargetPosition((int)(ticksPerInch * hyp/FRBL));
        backLeft.setTargetPosition((int)(ticksPerInch * hyp/FRBL));
        backRight.setTargetPosition((int)(ticksPerInch * hyp/FLBR));
        
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        frontLeft.setPower(FLBR);
        frontRight.setPower(FRBL);
        backLeft.setPower(FRBL);
        backRight.setPower(FLBR);
        
        while (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy()) {
            telemetry.addData("Motors", 
            frontLeft.getCurrentPosition() + " " +
            frontRight.getCurrentPosition() + " " +
            backLeft.getCurrentPosition() + " " +
            backRight.getCurrentPosition());
            telemetry.update();
        }
        //Stop moving and be at new spot
        xPos = xDest;
        yPos = yDest;
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
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
    
    }
}
// end method initAprilTag()
//https://cad.onshape.com/documents/8984e00f04b05696d8db3ecb/w/967eeac02a8de364065c7c08/e/193021613c0f3adca058692b
/*
* From RED WALL (x,y):
* Close left corner is -72, -72
* Close right corner is 72, -72
* Far left corner is -72, 72
* Far right corner is 72, 72
*/
 