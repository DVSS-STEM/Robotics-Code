package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import java.lang.Math; 

import java.util.List;
import java.util.ArrayList;
    //------------------------------------------------------------------

// tutorial link: youtube.com/watch?v=OZt33z-lyYo
// documentation link: chrome-extension://efaidnbmnnnibpcajpcglclefindmkaj/https://ftc-docs-cdn.ftclive.org/booklets/en/april_tags.pdf

public class CVTEST1_DECODE {
        private AprilTagProcessor aprilTagProcessor; // understands and operates upon apriltag IDs
        private VisionPortal visionPortal; // library to open up visual processors through your webcam
        
        private List<AprilTagDetection> detectedTags = new ArrayList<>(); // this will be a list of apriltags with their respective information attached
        
        private Telemetry telemetry; // for debugging
        
        private Position cameraPosition = new Position(DistanceUnit.CM,
            -5.8, -5.9, 14.5, 0); // position of the camera
        
        private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, 0, 0, 0); // rotation of the camera

        
        public void init(HardwareMap hardwareMap, Telemetry telemetry) {
            this.telemetry = telemetry; // basically when we encapsulate this we want to tell it what telemetry object we want it to use in the main code
            
            aprilTagProcessor = new AprilTagProcessor.Builder() // here we are basically defining the paremeters it passes on through the processor
                //.setDrawTagID(true)
                //.setDrawTagOutline(true)
                //.setDrawAxes(true)
                //.setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();
            
            VisionPortal.Builder builder = new VisionPortal.Builder(); // building our visionPortal
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")); // note this linr may need to be altered based on the names of our camera or hardwareMap attributions
            //builder.setCameraResolution(new Size(1280, 960)); // supposedly the "true" resolution. If it's funky try y = 720
            builder.addProcessor(aprilTagProcessor);
            
            visionPortal = builder.build();
        }
        
        public void update() { // update function
            detectedTags = aprilTagProcessor.getDetections();
        }
        
        public List<AprilTagDetection> getDetectedTags() { // function to return the detected tags we get in the update function
            return detectedTags;
        }
        
        public void displayDetectionTelemetry() { // given an ID print out the metadata to telemetry
            for (AprilTagDetection detection : detectedTags) {
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
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
            }
        }
        /*
        public List<String> allDetections() {
            List<String> strList = new ArrayList<>();
            for (AprilTagDetection detection : detectedTags) {
                strList.add(detection.metadata.name);
            }
            return strList;
        }*/
        
        public AprilTagDetection getTagBySpecificId(int id) { // iterates through the detected tags list for the requested tag
            for (AprilTagDetection detection : detectedTags) {
                if(detection.id == id) {
                    return detection;
                }
            }
            return null;
        }
        
        public void stop() { // closes vision portal to conserve resources
            if (visionPortal != null){
                visionPortal.close();
            }
        }
    }