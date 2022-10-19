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
    boolean autoMode=false;

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
        turntabletouch = hwMap.get(TouchSensor.class, "turntabletouch");
        armdowntouch = hwMap.get(TouchSensor.class, "armdowntouch");
        armuptouch = hwMap.get(TouchSensor.class, "armuptouch");
        elbowtouch = hwMap.get(TouchSensor.class, "elbowtouch");
        // Set servo to mid position
        ElbowPosition = 0;
        ElbowSpeed = 100;
        TurnTablePosition = 0;
        TurnTableSpeed = 10;
        ArmPosition = 0;
        ArmSpeed = 100;
        ClawSpeed = 0.05;
        ClawPosition = 0;
        WristSpeed = 0.05;
        WristPosition = 1;
        ArmVelocity=5000;
        ElbowVelocity=5000;
        TurnTableVelocity=1000;
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turntable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ResetTopHat();
        telemetry.update();
    }

    private void ResetTopHat(){
        ResetWristNClaw();
        ResetArmUp();
        ResetArmElbow();
        ResetTurnTable();
        ResetArmDown();
     }

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    public void runTopHat() {
            fullManualControl();
            partialManualControl();
            // Keep Servo position in valid range
            ClawPosition = Math.min(Math.max(ClawPosition, 0), 1);
            WristPosition = Math.min(Math.max(WristPosition, 0), 1);

            TurnTablePosition = Math.min(Math.max(TurnTablePosition, 100), 1650);
            ElbowPosition = Math.min(Math.max(ElbowPosition, -8500), -40);
            ArmPosition=Math.min(Math.max(ArmPosition, 0), 4700);

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
    }
    private void ResetTurnTable() {
        //if (!turntabletouch.isPressed()) {
        setMotorPosition(-5000,turntable,TurnTableVelocity);
        while (!turntabletouch.isPressed()) {
            telemetry.addData("Reset Turn Table Position", turntable.getCurrentPosition());
        }
        turntable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //}
        while (turntable.getCurrentPosition() < 200) {
            setMotorPosition(200, turntable, TurnTableVelocity);
        }
        TurnTablePosition=turntable.getCurrentPosition();
      }
    private void ResetArmUp() {
        //if (!armuptouch.isPressed()) {
            setMotorPosition(10000,arm,ArmVelocity);
            while (!armuptouch.isPressed()) {
                telemetry.addData("Reset Arm Position", arm.getCurrentPosition());
            }
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //}
    }
    private void ResetArmDown() {
        //if (!armdowntouch.isPressed()) {
            setMotorPosition(-10000,arm,ArmVelocity);
            while (!armdowntouch.isPressed()) {
                telemetry.addData("Reset Arm Position", arm.getCurrentPosition());
            }
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (arm.getCurrentPosition() < 100) {
            setMotorPosition(100, arm, ArmVelocity);
        }
        //}
        ArmPosition=arm.getCurrentPosition();
    }
    private void ResetArmElbow() {
        //if (!elbowtouch.isPressed()) {
        setMotorPosition(5000,elbow,ElbowVelocity);
        while (!elbowtouch.isPressed()) {
            telemetry.addData("Reset Elbow Position", elbow.getCurrentPosition());
        }
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (elbow.getCurrentPosition() > -20) {
            setMotorPosition(-20, elbow, ElbowVelocity);
        }
        ElbowPosition=elbow.getCurrentPosition();
        //}
    }

    private void ResetWristNClaw(){
        WristPosition = .9;
        wrist.setPosition(WristPosition);
        ClawPosition = 0.3;
        claw.setPosition(ClawPosition);
    }


    public void setMotorPosition(int pos, DcMotor motor, int motorVel){
        motor.setTargetPosition(pos);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) motor).setVelocity(motorVel);
    }
    private void setWristPosition(double pos){
        WristPosition=pos;
        //wrist.setPosition(WristPosition);
    }
    private void openClaw(boolean status){
        if (status) {
            ClawPosition = 1;
        }
        else{
            ClawPosition = 0.3;
        }
    }
    private void rightSidePickup(){
        double desiredWristPosition = 0.49;
        double desiredClawPosition = 1.0;
        int desiredArmPosition = 1100;
        int desiredElbowPosition = -4392;
        int desiredTurnTablePosition = 242;
        ArmPosition = desiredArmPosition;
        TurnTablePosition = desiredTurnTablePosition;
        ElbowPosition = desiredElbowPosition;
        WristPosition = desiredWristPosition;
        openClaw(true);

    }
    private void leftSideHighDrop(){
        double desiredWristPosition = 0.799999;
        double desiredClawPosition = 0.3;
        int desiredArmPosition = 4284;
        int desiredElbowPosition = -5325;
        int desiredTurnTablePosition = 1514;
        ArmPosition=desiredArmPosition;
        TurnTablePosition=desiredTurnTablePosition;
        ElbowPosition=desiredElbowPosition;
        WristPosition=desiredWristPosition;
        openClaw(false);

    }

    private void fullManualControl(){
        if (gamepad2.dpad_up) {
            WristPosition += WristSpeed;
            telemetry.addData("DpadUp Wrist Up", "Pressed");
        }
        if (gamepad2.dpad_down) {
            WristPosition += -WristSpeed;
            telemetry.addData("DpadDown Wrist Down", "Pressed");
        }
        if (gamepad2.dpad_right) {
            ClawPosition = 0.3 ;
            HoldCone = true;
            telemetry.addData("DpadRight Claw Close", "Pressed");
        }
        if (gamepad2.dpad_left) {
            ClawPosition = 1 ;
            HoldCone = false;
            telemetry.addData("DpadLeft Claw Open", "Pressed");
        }
        if (gamepad2.left_stick_y != 0) {
            ArmPosition += -gamepad2.left_stick_y*ArmSpeed;
        }
        if (gamepad2.right_stick_y != 0) {
            ElbowPosition += gamepad2.right_stick_y*ElbowSpeed;
        }
        if (gamepad2.left_trigger > 0) {
            TurnTablePosition += gamepad2.left_trigger*TurnTableSpeed;
            telemetry.addData("Left Trigger TurnTable Left", "Pressed");
        }
        if (gamepad2.right_trigger > 0) {
            TurnTablePosition += -gamepad2.right_trigger*TurnTableSpeed;
            telemetry.addData("Right Trigger TurnTable Right", "Pressed");
        }
    }
    private void partialManualControl(){
        if (gamepad2.right_bumper && gamepad2.x){
            rightSidePickup();
        }
        if (gamepad2.left_bumper && gamepad2.y){
            leftSideHighDrop();
        }
    }

}