package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "AprilTag Position by ID", group = "Vision")
public class ZachsCameraPositioning extends LinearOpMode {

    VisionPortal visionPortal;
    AprilTagProcessor aprilTag;

    // CHANGE THIS to the tag you want
    int targetID = 21;

    @Override
    public void runOpMode() {

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setOutputUnits(DistanceUnit.METER,AngleUnit.DEGREES)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        waitForStart();

        while (opModeIsActive()) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            for (AprilTagDetection detection : currentDetections){

                if (detection.id == targetID) {
                    if (detection.ftcPose != null){
                        double x = detection.ftcPose.x;     // meters (left/right)
                        double y = detection.ftcPose.y;     // meters (up/down)
                        double z = detection.ftcPose.z;     // meters (distance)
                        double yaw = detection.ftcPose.yaw; // degrees
        
                        telemetry.addLine("TARGET TAG FOUND");
                        telemetry.addData("ID", detection.id);
                        telemetry.addData("X (left/right)", "%.2f m", x);
                        telemetry.addData("Y (up/down)", "%.2f m", y);
                        telemetry.addData("Z (forward)", "%.2f m", z);
                        telemetry.addData("Yaw (deg)", "%.1f", yaw);
                    }
                    else{
                        telemetry.addLine("Pose not avaible");
                        break;
                    }
                }    
                else{
                    telemetry.addLine("Target tag NOT visible");
                }
            }
            telemetry.update();
        }
    }

    private AprilTagDetection getTagByID(int id) {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        for (AprilTagDetection detection : detections) {
            if (detection.id == id) {
                return detection;
            }
        }
        return null;
    }
}
