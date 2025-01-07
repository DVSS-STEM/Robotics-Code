package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.ArrayList;
import java.util.Map;
/*
///public class MotorManager {
    public MotorManager(MotorTPR[] tprList) {
        //Actually use MotorConfigurationType correctly, in time.
        Map<String, Integer[]> ticksMap = new HashMap<>();
        for (MotorTPR entry : tprList) {
            ticksMap.put(entry.getType(), [entry.getTPR(), entry.getRPM()]);
        }
        this.ticksMap = ticksMap;
    }
    private ArrayList<DcMotor> poweredMotors = new ArrayList<>();

    //ticks are how much it runs after a single press, basically
    //a lower threshold value for movement.
    public void switchOn(DcMotor motor) {
        poweredMotors.add(motor);
    }
    public void runTicks(DcMotor motor, int ticks, double power) {
        if (!motor.isBusy()) {
            switchOn(motor);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setTargetPosition(ticks);
            motor.setPower(power);
        }
    }
    public void runMs(DcMotor motor, double ms, double power) {
        String type = motor.getMotorType().getName();
        ticksMap.get(type);

        
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

//useful links
/*
 * https://javadoc.io/doc/org.firstinspires.ftc
 * DCMOTOR DOCUMENTATION (has MotorConfigurationType)
 * https://javadoc.io/doc/org.firstinspires.ftc/RobotCore/latest/com/qualcomm/robotcore/hardware/configuration/typecontainers/MotorConfigurationType.html
 */
*/