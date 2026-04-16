package org.firstinspires.ftc.teamcode;

//imports
    //eventloop components
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="DEV Strafing")
public class DEV_Strafing extends LinearOpMode {
    // Motor and servo declarations
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    public static double power = 0;
    boolean last_up = false;
    boolean last_down = false;
    boolean last_r = false;
    boolean last_l = false;

    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeHardware();

        waitForStart();

        while (opModeIsActive()) {
            double frontLeftPower = 0;
            double backLeftPower = 0;
            double frontRightPower = 0;
            double backRightPower = 0;
            
            
            if (gamepad1.dpad_up && !last_up && power<1){
                power+=0.05;
            } else if (gamepad1.dpad_down && !last_down && power>-1){
                power-=0.05;
            } else if (gamepad1.dpad_right && !last_r && power>-1){
                power+=0.01;
            } else if (gamepad1.dpad_left && !last_l && power>-1){
                power-=0.01;
            }
            
            if (gamepad1.b){
                power = 0;
            }
            last_up = gamepad1.dpad_up;
            last_down = gamepad1.dpad_down;
            last_r = gamepad1.dpad_right;
            last_l = gamepad1.dpad_left;
            telemetry.addData("Power:",power);
            telemetry.update();
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            
            
        }
    }

    private void initializeHardware() {
        // Motor initialization
        frontLeftMotor = hardwareMap.dcMotor.get("backRightMotor");
        backLeftMotor = hardwareMap.dcMotor.get("frontRightMotor");
        frontRightMotor = hardwareMap.dcMotor.get("backLeftMotor");
        backRightMotor = hardwareMap.dcMotor.get("frontLeftMotor");

        // Set motor directions
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Ensure all motors have BRAKE behavior set
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
    }
}