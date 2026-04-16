package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp
public class CVPIPELINETEST1 extends LinearOpMode {
    
    CVTEST1_DECODE pipeline = new CVTEST1_DECODE();
    
    public void runOpMode() {
        waitForStart();
        pipeline.init(hardwareMap, telemetry);
        while (opModeIsActive()){
            pipeline.update();
            AprilTagDetection id20 = pipeline.getTagBySpecificId(20);
            pipeline.displayDetectionTelemetry();
            //telemetry.addLine(Integer.toString(pipeline.allDetections().size()));
            //telemetry.addLine(String.join(", ", pipeline.allDetections()));
            telemetry.update();
            sleep(1000);
        }
    }
}
