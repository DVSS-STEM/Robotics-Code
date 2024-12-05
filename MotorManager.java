package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.ArrayList;

public class MotorManager {
    private ArrayList<DcMotor> poweredMotors = new ArrayList<>();

    public void run(DcMotor motor, int ticks, double power) {
        if (!motor.isBusy()) {
            poweredMotors.add(motor);
            vertical_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setTargetPosition(ticks);
            vertical_1.setPower(power);
        }
    }
    public void resetTicks(DcMotor motor) {
        motor.setMode(DcMotor.STOP_AND_RESET_ENCODER);
    }
    public void stopMotor(DcMotor motor) {
        motor.setPower(0);
        poweredMotors.remove(motor);
    }
    public void stopAndReset(DcMotor motor) {
        resetTicks(motor);
        stopMotor(motor);
    }
    public void stopCompleted() {
        for (DcMotor motor : getPoweredMotors()) {
            if (!motor.isBusy()) {
                stopMotor(motor);
            }
        }
    }
    public ArrayList<DcMotor> getPoweredMotors() {
        return new ArrayList<>(poweredMotors);
    }
}
