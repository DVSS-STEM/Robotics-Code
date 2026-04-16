package org.firstinspires.ftc.teamcode; 
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; 
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp; 
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName; 
import org.firstinspires.ftc.vision.VisionPortal; 
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection; 
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor; 
import java.util.List;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.*;

@TeleOp(name = "AprilTag Position by ID", group = "Vision") 
public class AprilTagPositionByID extends LinearOpMode {
    private Servo cameraServo;
    VisionPortal visionPortal; 
    AprilTagProcessor aprilTag; 
    int TARGET_TAG_ID = 20; 
    @Override 
    public void runOpMode() { 
        cameraServo = hardwareMap.servo.get("cameraServo");
        cameraServo.setPosition(10 * Math.PI / 180);
        aprilTag = new AprilTagProcessor.Builder() 
            .setDrawAxes(true) 
            .setDrawCubeProjection(true) 
            .setDrawTagOutline(true)
            .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)
            .build(); 
        visionPortal = new VisionPortal.Builder() 
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) 
            .addProcessor(aprilTag)
            .build(); 
        waitForStart(); 
        while (opModeIsActive()) { 
            Double yaw = getTagYaw(TARGET_TAG_ID); 
            if (yaw != null) { telemetry.addLine("TARGET TAG FOUND"); 
                telemetry.addData("Yaw needed (deg)", "%.1f", yaw); 
                
            } 
            else { 
                telemetry.addLine("Target tag not visible or pose unavailable"); 
                
            } 
            telemetry.update(); 
            
        } 
        
    } 
/** * Returns the yaw angle (degrees) of the specified AprilTag ID. * Returns null if the tag is not found or pose is unavailable. */ 
private Double getTagYaw(int tagId) { 
    List<AprilTagDetection> detections = aprilTag.getDetections(); 
    for (AprilTagDetection detection : detections) { 
        if (detection.id == tagId && detection.ftcPose != null) {
            double x = detection.ftcPose.x;
            double y = detection.ftcPose.y;
            double z = detection.ftcPose.z;
            double angle = detection.ftcPose.yaw;
            telemetry.addData("x: ",x);
            telemetry.addData("y: ",y);
            telemetry.addData("z: ",z);
            
            return angle+Math.atan(x/y); 
            
            } 
        
        } 
        return null; 
    } 
    
}