package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.ArrayList;

public class MotorTPR {
    public MotorTPR(MotorConfigurationType type, int tpr) {
        this.type = type;
        this.tpr = tpr;
    }
    public MotorConfigurationType getType() {
        return this.type;
    }
    public int getTPR() {
        return this.tpr;
    }
}