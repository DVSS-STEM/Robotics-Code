package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.ArrayList;

public class MotorTPR {
    public MotorTPR(String type, int tpr, int rpm) {
        this.type = type;
        this.tpr = tpr;
        this.rpm = rpm;
    }
    public String getType() {
        return type;
    }
    public int getTPR() {
        return tpr;
    }
    public int getRPM() {
        return rpm;
    }
}