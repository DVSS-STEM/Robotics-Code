package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.ArrayList;

public class MotorTPR {
    public MotorTPR(String type, int tpr) {
        this.type = type;
        this.tpr = tpr;
    }
    public String getType() {
        return this.type;
    }
    public int getTPR() {
        return this.tpr;
    }
}