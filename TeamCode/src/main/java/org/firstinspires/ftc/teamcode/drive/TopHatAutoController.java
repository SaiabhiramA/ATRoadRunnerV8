package org.firstinspires.ftc.teamcode.drive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//@TeleOp(name = "TopHatManualController (Blocks to Java)")
public class TopHatAutoController {

    private DcMotor arm;
    private DcMotor elbow;
    private DcMotor turntable;
    private Servo wrist;
    private Servo claw;
    private TouchSensor turntabletouch;
    private TouchSensor armdowntouch;
    private TouchSensor armuptouch;
    private TouchSensor elbowtouch;
    private HardwareMap hwMap;
    private  Telemetry telemetry;
    private Gamepad gamepad1;
    private Gamepad gamepad2;
    double ElbowPosition;
    int ElbowSpeed;
    double TurnTablePosition;
    int TurnTableSpeed;
    double ArmPosition;
    int ArmSpeed;
    double ClawSpeed;
    double ClawPosition;
    double WristSpeed;
    double WristPosition;
    boolean HoldCone= false ;
    int ArmVelocity;
    int TurnTableVelocity;
    int ElbowVelocity;

    public void initializeRobot(HardwareMap hardwareMapAT, Telemetry tl, Gamepad gp1 , Gamepad gp2, String Alliance) {
        hwMap = hardwareMapAT;
        telemetry = tl;
        gamepad1=gp1;
        gamepad2=gp2;
        arm = hwMap.get(DcMotor.class, "arm");
        elbow = hwMap.get(DcMotor.class, "elbow");
        turntable = hwMap.get(DcMotor.class, "turntable");
        wrist = hwMap.get(Servo.class, "wrist");
        claw = hwMap.get(Servo.class, "claw");
        //turntabletouch = hwMap.get(TouchSensor.class, "turntabletouch");
        //armdowntouch = hwMap.get(TouchSensor.class, "armdowntouch");
        //armuptouch = hwMap.get(TouchSensor.class, "armuptouch");
        //elbowtouch = hwMap.get(TouchSensor.class, "elbowtouch");
        // Set servo to mid position
        ElbowPosition = 0;
        ElbowSpeed = 10;
        TurnTablePosition = 0;
        TurnTableSpeed = 10;
        ArmPosition = 0;
        ArmSpeed = 10;
        ClawSpeed = 0.05;
        ClawPosition = 0.1;
        WristSpeed = 0.05;
        WristPosition = 0.1;
        ArmVelocity=5000;
        ElbowVelocity=5000;
        TurnTableVelocity=5000;
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turntable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    public void runTopHat() {

       /*if (HoldCone) {
            ClawPosition = 0 ;
        }*/
            if (gamepad2.dpad_up) {
                WristPosition += WristSpeed;
                telemetry.addData("DpadUp Wrist Up", "Pressed");
            }
            if (gamepad2.dpad_down) {
                WristPosition += -WristSpeed;
                telemetry.addData("DpadDown Wrist Down", "Pressed");
            }
            if (gamepad2.dpad_right) {
                //ClawPosition += ClawSpeed;
                ClawPosition = 0 ;
                HoldCone = true;
                telemetry.addData("DpadRight Claw Close", "Pressed");
            }
            if (gamepad2.dpad_left) {
                //ClawPosition += -ClawSpeed;
                ClawPosition = 1 ;
                HoldCone = false;
                telemetry.addData("DpadLeft Claw Open", "Pressed");
            }
            if (gamepad2.left_stick_y > 0) {
                ArmPosition += gamepad2.left_stick_y*ArmSpeed;
                telemetry.addData("Left Stick -Y Arm Down", "Pressed");
            }
            if (gamepad2.left_stick_y < 0) {
                ArmPosition += gamepad2.left_stick_y*ArmSpeed;
                telemetry.addData("Left Stick Y Arm Up", "Pressed");
            }
            if (gamepad2.right_stick_y < 0) {
                ElbowPosition += gamepad2.right_stick_y*ElbowSpeed;
                telemetry.addData("Right Stick -Y Elbow Down", "Pressed");
            }
            if (gamepad2.right_stick_y > 0) {
                ElbowPosition += gamepad2.right_stick_y*ElbowSpeed;
                telemetry.addData("Right Stick Y Elbow Up", "Pressed");
            }
            if (gamepad2.left_trigger > 0) {
                TurnTablePosition += gamepad2.left_trigger*TurnTableSpeed;
                telemetry.addData("Left Trigger TurnTable Left", "Pressed");
            }
            if (gamepad2.right_trigger > 0) {
                TurnTablePosition += -gamepad2.right_trigger*TurnTableSpeed;
                telemetry.addData("Right Trigger TurnTable Right", "Pressed");
            }
            // Keep Servo position in valid range
            ClawPosition = Math.min(Math.max(ClawPosition, 0), 1);
            WristPosition = Math.min(Math.max(WristPosition, 0), 1);
            wrist.setPosition(WristPosition);
            claw.setPosition(ClawPosition);
            setMotorPosition((int) ArmPosition,arm,ArmVelocity);
            setMotorPosition((int) ElbowPosition,elbow,ElbowVelocity);
            setMotorPosition((int) TurnTablePosition,turntable,TurnTableVelocity);
            telemetry.addData("wrist position", wrist.getPosition());
            telemetry.addData("claw position", claw.getPosition());
            telemetry.addData("arm position", arm.getCurrentPosition());
            telemetry.addData("elbow position", elbow.getCurrentPosition());
            telemetry.addData("turntable position", turntable.getCurrentPosition());
           // telemetry.update();

      //  }
    }
    private void ResetTurnTable() {
        if (!turntabletouch.isPressed()) {
            setMotorPosition(-5000,turntable,TurnTableVelocity);
            while (!turntabletouch.isPressed()) {
                telemetry.addData("Reset Turn Table Position", turntable.getCurrentPosition());
            }
            turntable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setMotorPosition(300,turntable,TurnTableVelocity);
        }
    }
    private void ResetArmUp() {
        if (!armuptouch.isPressed()) {
            arm.setTargetPosition(-5000);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx) arm).setVelocity(ArmVelocity);
            while (!armuptouch.isPressed()) {
                telemetry.addData("Reset Arm Position", arm.getCurrentPosition());
            }
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    private void setMotorPosition(int pos, DcMotor motor, int motorVel){
        motor.setTargetPosition(pos);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) motor).setVelocity(motorVel);
    }
}