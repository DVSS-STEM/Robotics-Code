package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import org.openftc.easyopencv.OpenCvPipeline;

import com.acmerobotics.dashboard.config.Config;

import org.checkerframework.checker.signedness.qual.Constant;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.w3c.dom.css.Rect;

@Config
public class CVTEST1_DECODE extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location{
        Close,
        Far
    }
    
    private Location location;
    
    // regions of interest (ROI's). These will be rectangles centered on the points of the zones
    static final Rect closeROI = new Rect(new Point())