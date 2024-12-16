package org.firstinspires.ftc.teamcode;

//imports
    //eventloop components
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

    //hardware components
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

    //utilities
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.*;
import java.lang.Math;

@TeleOp
public class DriveCode extends LinearOpMode {
    boolean claw1 = false;
    boolean claw2 = false;
    double rot_pos = 0.5;
    int vpos = 0;
    int hpos = 0;
    CRServo sub_peck;
    Servo specimen_claw, sample_claw, sub_rotation;

    DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, vertical_1, vertical_2, horizontal, armMotor;



    @Override
    public void runOpMode(){

        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        armMotor = hardwareMap.dcMotor.get("armMotor");
        

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        
        //set zero power behaviour to brake
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //sub_peck.setZeroPowerBehavior(CrServo.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorManager manager = new MotorManager();
        waitForStart();

        while (opModeIsActive()) {
            manager.stopCompleted();
            telemetry.addData("----ACTIVE MOTORS----", "");
            for (DcMotor motor : manager.getPoweredMotors()) {
                telemetry.addData("Motor Name", motor.getDeviceName());
                telemetry.addData("Ticks remaining", motor.getTargetPosition() - motor.getCurrentPosition());
                telemetry.addData(",", "");
            }
            double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            
            MotorConfigurationType flmtype = armMotor.getMotorType();
            telemetry.addData("FLMTYPENAME", flmtype.getName());
            telemetry.addData("FLMTYPETPM", flmtype.getTicksPerRev());
            
            if (gamepad1.right_bumper){
                //manager.runTicks(armMotor, 6, 1);
                telemetry.addData("Ouch", "Owie");
                if (!armMotor.isBusy()) {
                    armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    armMotor.setTargetPosition(100);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(1);
                }
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armMotor.setPower(1);
            } else if (gamepad1.left_bumper){
                manager.runTicks(armMotor, -6, 1);
            }
            
            
            telemetry.update();
            
           
        }
    }
    
    private class MotorTPR {
        String type;
        int tpr;
        int rpm;
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
    private class MotorManager {
        private ArrayList<DcMotor> poweredMotors = new ArrayList<>();
        //ticks are how much it runs after a single press, basically
        //a lower threshold value for movement.
        public void switchOn(DcMotor motor) {
            poweredMotors.add(motor);
        }
        public void runTicks(DcMotor motor, int ticks, double power) {
            if (!motor.isBusy()) {
                switchOn(motor);
                motor.setTargetPosition(ticks);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(power);
            }
        }
        public void resetTicks(DcMotor motor) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
                    stopAndReset(motor);
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

    
}

