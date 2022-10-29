package org.firstinspires.ftc.teamcode.drive;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
    int testCounter = 0 ;

    double desiredWristPosition;
    double desiredClawPosition ;
    int desiredArmPosition ;
    int desiredElbowPosition ;
    int desiredTurnTablePosition ;
    int step = 0 ;
    int noOfCones = 0 ;
    MecanumDriveAT drive;
    ATRobotMode robotMode=ATRobotMode.RESET;

    public void initializeRobot(HardwareMap hardwareMapAT, MecanumDriveAT driveAT, Telemetry tl, Gamepad gp1 , Gamepad gp2, String Alliance, ATRobotMode rMode) {
        drive=driveAT;
        robotMode=rMode;
        telemetry = tl;
        gamepad1=gp1;
        gamepad2=gp2;
        arm = hardwareMapAT.get(DcMotor.class, "arm");
        elbow = hardwareMapAT.get(DcMotor.class, "elbow");
        turntable = hardwareMapAT.get(DcMotor.class, "turntable");
        wrist = hardwareMapAT.get(Servo.class, "wrist");
        claw = hardwareMapAT.get(Servo.class, "claw");
        turntabletouch = hardwareMapAT.get(TouchSensor.class, "turntabletouch");
        armdowntouch = hardwareMapAT.get(TouchSensor.class, "armdowntouch");
        armuptouch = hardwareMapAT.get(TouchSensor.class, "armuptouch");
        elbowtouch = hardwareMapAT.get(TouchSensor.class, "elbowtouch");
        // Set servo to mid position
        ElbowPosition = 0;
        ElbowSpeed = 10;
        TurnTablePosition = 0;
        TurnTableSpeed = 10;
        ArmPosition = 0;
        ArmSpeed = 10;
        ClawSpeed = 0.05;
        ClawPosition = 0;
        WristSpeed = 0.01;
        WristPosition = 1;
        ArmVelocity=500;//5000;
        ElbowVelocity=500;//5000;
        TurnTableVelocity=1000;
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turntable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ResetTopHat();
      }
    public void ResetTopHat(){
        robotMode=ATRobotMode.RESET;
        ResetWristNClaw();
        ResetArmUp();
        ResetArmElbow();
        ResetTurnTable();
        ResetArmDown();
     }

     public void ResetTopHatStop(){
        robotMode=ATRobotMode.STOP;
        ResetWristNClaw();
        ResetArmUptoStop();
        ResetArmElbowtoStop();
        ResetTurnTabletoStop();
        ResetArmDowntoStop();
     }

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    public void runTopHat() {
            fullManualControl();
            partialManualControl();
            fullAutoControl();

            setMotorNServoMaximums();

            moveTopHatMotors();

            telemetry.addData("wrist position", wrist.getPosition());
            telemetry.addData("wrist position while moving", wrist.getController().getServoPosition(0));
            telemetry.addData("claw position", claw.getPosition());
            telemetry.addData("arm position", arm.getCurrentPosition());
            telemetry.addData("elbow position", elbow.getCurrentPosition());
            telemetry.addData("turntable position", turntable.getCurrentPosition());
            telemetry.addData("testCounter", testCounter);

    }

    private void moveTopHatMotors(){
        setMotorPosition((int) ArmPosition,arm,ArmVelocity);
        setMotorPosition((int) ElbowPosition,elbow,ElbowVelocity);
        setMotorPosition((int) TurnTablePosition,turntable,TurnTableVelocity);
    }

    private void setMotorNServoMaximums(){
        ClawPosition = Math.min(Math.max(ClawPosition, 0), 1);
        WristPosition = Math.min(Math.max(WristPosition, 0), 1);

        TurnTablePosition = Math.min(Math.max(TurnTablePosition, 100), 1900);
        ElbowPosition = Math.min(Math.max(ElbowPosition, -8500), -400);
        ArmPosition=Math.min(Math.max(ArmPosition, 0), 4700);
    }

    public void setTopHatMotorsVelocity(int TTVel, int ArmVel, int ElbowVel){

        TurnTableVelocity=TTVel;
        ArmVelocity=ArmVel;
        ElbowVelocity=ElbowVel;

    }

    private void ResetTurnTable() {
        setMotorPosition(-5000,turntable,TurnTableVelocity);
        while (!turntabletouch.isPressed()) {
            telemetry.addData("Reset Turn Table Position", turntable.getCurrentPosition());
        }
        turntable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (turntable.getCurrentPosition() < 200) {
            setMotorPosition(200, turntable, TurnTableVelocity);
        }
        TurnTablePosition=turntable.getCurrentPosition();
      }

    private void ResetTurnTabletoStop() {
        while (turntable.getCurrentPosition() > 200 || !turntabletouch.isPressed())  {
            setMotorPosition(200, turntable, 1000);
        }
    }

    private void ResetArmUp() {
           setMotorPosition(10000,arm,ArmVelocity);
            while (!armuptouch.isPressed()) {
                telemetry.addData("Reset Arm Position", arm.getCurrentPosition());
            }
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void ResetArmUptoStop() {
        while (arm.getCurrentPosition() < 500 || !armuptouch.isPressed())  {
            setMotorPosition(500, arm, 1000);
        }
    }

    private void ResetArmDown() {
            setMotorPosition(-10000,arm,ArmVelocity);
            while (!armdowntouch.isPressed()) {
                telemetry.addData("Reset Arm Position", arm.getCurrentPosition());
            }
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (arm.getCurrentPosition() < 100) {
            setMotorPosition(100, arm, ArmVelocity);
        }
        ArmPosition=arm.getCurrentPosition();
    }

    private void ResetArmDowntoStop() {
        while (arm.getCurrentPosition() > 100 || !armdowntouch.isPressed())  {
            setMotorPosition(100, arm, 1000);
        }
    }

    private void ResetArmElbow() {
        setMotorPosition(5000,elbow,ElbowVelocity);
        while (!elbowtouch.isPressed()) {
            telemetry.addData("Reset Elbow Position", elbow.getCurrentPosition());
        }
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (elbow.getCurrentPosition() > -20) {
            setMotorPosition(-20, elbow, ElbowVelocity);
        }
        ElbowPosition=elbow.getCurrentPosition();
    }

    private void ResetArmElbowtoStop() {
        while (elbow.getCurrentPosition() < -100 || !elbowtouch.isPressed())  {
            setMotorPosition(-100, elbow, 1000);
        }
    }

    private void ResetWristNClaw(){

        setWristPosition(.9);
        openClaw(false);
        sleep(2000);

    }
    public void setMotorPosition(int pos, DcMotor motor, int motorVel){
        motor.setTargetPosition(pos);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) motor).setVelocity(motorVel);
    }
    private void setWristPosition(double pos){
        WristPosition=pos;
        wrist.setPosition(WristPosition);
    }
    private void openClaw(boolean status){
        if (status) {
            ClawPosition = 1;
        }
        else{
            ClawPosition = 0.3;
        }
        claw.setPosition(ClawPosition);
    }

    private void fullAutoControl( )
    {
      if (this.robotMode==ATRobotMode.AUTO_RED_RIGHT_HIGH_PICK_CONE) {
          redAllianceRightAutonHigh();
        }
      if (this.robotMode==ATRobotMode.AUTO_RED_RIGHT_MEDIUM_PICK_CONE){
          redAllianceRightAutonMedium();
      }

    }
    // THis method is to lift the arm after the pole if picked up..
    private void setTopHatPosition(double desiredWristPosition, boolean desiredClawPosition , int desiredArmPosition , int desiredElbowPosition , double desiredTurnTablePosition ){
        ArmPosition = desiredArmPosition;
        TurnTablePosition = desiredTurnTablePosition;
        ElbowPosition = desiredElbowPosition;
    }
    private void moveTopHatPosition(double desiredWristPosition, boolean desiredClawPosition , int desiredArmPosition , int desiredElbowPosition , double desiredTurnTablePosition ){
        ArmPosition = desiredArmPosition;
        TurnTablePosition = desiredTurnTablePosition;
        ElbowPosition = desiredElbowPosition;
        moveTopHatMotors();
        if (desiredWristPosition==-1){
            wrist.getController().pwmDisable();
            claw.getController().pwmDisable();
        }
        else {
            setWristPosition(desiredWristPosition);
            openClaw(desiredClawPosition);
        }

    }

    public void setRobotMode(ATRobotMode rMode){
        robotMode=rMode;
    }

    public ATRobotMode getRobotMode(){
        return robotMode;
    }


    public void redAllianceRightAutonHigh(){
        if (this.robotMode==ATRobotMode.AUTO_RED_RIGHT_HIGH_PICK_CONE) {
            if (step == 0) {
                if (noOfCones == 0) {
                    setWristPosition(.55);
                    desiredElbowPosition = -3431;
                    desiredArmPosition = 473;
                } else if (noOfCones == 1) {
                    setWristPosition(.54);
                    desiredElbowPosition = -3709;
                    desiredArmPosition = 526;
                } else if (noOfCones == 2) {
                    setWristPosition(.49);
                    desiredElbowPosition = -3927;
                    desiredArmPosition = 562;
                } else if (noOfCones == 3) {
                    setWristPosition(.49);
                    desiredElbowPosition = -4068;
                    desiredArmPosition = 604;
                } else if (noOfCones == 4) {
                    setWristPosition(.44);
                    desiredElbowPosition = -4375;
                    desiredArmPosition = 737;
                }
                setTopHatPosition(.55, true, desiredArmPosition, desiredElbowPosition, 215);
                openClaw(true);
                step = 1;
            } else if (step == 1 && isInRange(ElbowPosition, elbow.getCurrentPosition())
                    && isInRange(ArmPosition, arm.getCurrentPosition())
                    && isInRange(TurnTablePosition, turntable.getCurrentPosition())) {
                telemetry.addData(" LOWER WRIST", "Pressed");

                openClaw(false);
                sleep(800);
                setWristPosition(.15);
                sleep(500);

                setTopHatPosition(.55, false, 4405, -3427, 215);
                step = 2;

            } else if (step == 2 && isInRange(-3427, elbow.getCurrentPosition())
                    && isInRange(4405, arm.getCurrentPosition())
                    && isInRange(215, turntable.getCurrentPosition())) {
                setTopHatPosition(.59, true, 4405, -3427, 1505);
                setWristPosition(.59);
                //sleep(500);
                step = 3;

            } else if (step == 3 && isInRange(-3427, elbow.getCurrentPosition())
                    && isInRange(4405, arm.getCurrentPosition())
                    && isInRange(1505, turntable.getCurrentPosition())) {
                setTopHatPosition(.59, true, 4113, -4900, 1522);
                openClaw(true);

                step = 4;
            } else if (step == 4 && isInRange(-4900, elbow.getCurrentPosition())
                    && isInRange(4113, arm.getCurrentPosition())
                    && isInRange(1522, turntable.getCurrentPosition())) {
                setTopHatPosition(.59, true, 4113, -3388, 1522);

                step = 5;
            } else if (step == 5 && isInRange(-3388, elbow.getCurrentPosition())
                    && isInRange(4113, arm.getCurrentPosition())
                    && isInRange(1522, turntable.getCurrentPosition())) {
                setTopHatPosition(.59, true, 4113, -3388, 215);

            } else if (step == 5 && isInRange(-3388, elbow.getCurrentPosition())
                    && isInRange(4113, arm.getCurrentPosition())
                    && isInRange(215, turntable.getCurrentPosition())) {

                noOfCones = noOfCones + 1;
                step = 0;
            }

            if (noOfCones == 5) {
                this.robotMode = ATRobotMode.RESET;
                noOfCones = 0;

            }
        }
    }
    public boolean areFiveConesDone(){
        if (noOfCones>4) {
            return true;
        }
        else
        {
            return false;
        }
    }
    public void redAllianceRightAutonMedium(){
        if (this.robotMode==ATRobotMode.AUTO_RED_RIGHT_MEDIUM_SETUP) {
            moveTopHatPosition(-1, false, 4700, -50, 1800);
            noOfCones=0;
        }
        if (this.robotMode==ATRobotMode.AUTO_RED_RIGHT_MEDIUM_PICK_CONE) {
                if (step == 0) {
                    if (noOfCones == 0) {
                        desiredElbowPosition = -6241;
                        desiredArmPosition = 2189;
                        desiredTurnTablePosition=1800;
                    } else if (noOfCones == 1) {
                        desiredElbowPosition = -6160;
                        desiredArmPosition = 2040;
                        desiredTurnTablePosition=1800;
                    } else if (noOfCones == 2) {
                        desiredElbowPosition = -6057;
                        desiredArmPosition = 1866;
                        desiredTurnTablePosition=1800;
                    } else if (noOfCones == 3) {
                        desiredElbowPosition = -5801;
                        desiredArmPosition = 1544;
                        desiredTurnTablePosition=1800;
                    } else if (noOfCones == 4) {
                        desiredElbowPosition = -5779;
                        desiredArmPosition = 1483;
                        desiredTurnTablePosition=1800;
                    }
                    moveTopHatPosition(0, true, desiredArmPosition, desiredElbowPosition, desiredTurnTablePosition);
                    openClaw(true);
                    step = 1;
                } else if (step == 1 && isInRange(desiredElbowPosition, elbow.getCurrentPosition())
                        && isInRange(desiredArmPosition, arm.getCurrentPosition())
                        && isInRange(desiredTurnTablePosition, turntable.getCurrentPosition())) {
                    telemetry.addData(" LOWER WRIST", "Pressed");
                    setWristPosition(.3);
                    sleep(500);
                    openClaw(false);
                    sleep(800);
                    setWristPosition(.10);
                    sleep(500);

                    moveTopHatPosition(0, false, 4242, -5895, 533);
                    step = 2;

                } else if (step == 2 && isInRange(-5895, elbow.getCurrentPosition())
                        && isInRange(4242, arm.getCurrentPosition())
                        && isInRange(533, turntable.getCurrentPosition())) {
                    setWristPosition(.45);
                    sleep(500);
                    openClaw(true);
                    sleep(500);
                    moveTopHatPosition(0, false, 4500, -6163, 900);
                    step = 3;
                }else if (step == 3 && isInRange(-6163, elbow.getCurrentPosition())
                        && isInRange(4500, arm.getCurrentPosition())
                        && isInRange(900, turntable.getCurrentPosition())) {
                    noOfCones = noOfCones + 1;
                    step = 0;
                }
                if (noOfCones>4){
                    robotMode=ATRobotMode.AUTO_RED_RIGHT_MEDIUM_PARK;
                }
        }
        telemetry.addData("wrist position", wrist.getPosition());
        telemetry.addData("wrist position while moving", wrist.getController().getServoPosition(0));
        telemetry.addData("claw position", claw.getPosition());
        telemetry.addData("arm position", arm.getCurrentPosition());
        telemetry.addData("elbow position", elbow.getCurrentPosition());
        telemetry.addData("turntable position", turntable.getCurrentPosition());
        telemetry.addData("noOfCones", noOfCones);
    }

    private boolean isInRange(double desiredValue, double inputValue){
        return (Math.abs(inputValue)>=Math.abs(desiredValue)-5) && (Math.abs(inputValue)<=Math.abs(desiredValue)+5);
    }

    private void fullManualControl(){
        if (gamepad2.dpad_up) {
            robotMode=ATRobotMode.MANUAL;
            WristPosition += WristSpeed;
            setWristPosition(WristPosition);
            telemetry.addData("DpadUp Wrist Up", "Pressed");

        }
        if (gamepad2.dpad_down) {
            robotMode=ATRobotMode.MANUAL;
            WristPosition += -WristSpeed;
            setWristPosition(WristPosition);
            telemetry.addData("DpadDown Wrist Down", "Pressed");
        }
        if (gamepad2.dpad_right) {
            robotMode=ATRobotMode.MANUAL;
            openClaw(false);
            HoldCone = true;
            telemetry.addData("DpadRight Claw Close", "Pressed");
        }
        if (gamepad2.dpad_left) {
            robotMode=ATRobotMode.MANUAL;
            openClaw(true);
            HoldCone = false;
            telemetry.addData("DpadLeft Claw Open", "Pressed");
        }
        if (gamepad2.left_stick_y != 0) {
            robotMode=ATRobotMode.MANUAL;
            ArmPosition += -gamepad2.left_stick_y*ArmSpeed;
        }
        if (gamepad2.right_stick_y != 0) {
            robotMode=ATRobotMode.MANUAL;
            ElbowPosition += gamepad2.right_stick_y*ElbowSpeed;
        }
        if (gamepad2.left_trigger > 0) {
            robotMode=ATRobotMode.MANUAL;
            TurnTablePosition += gamepad2.left_trigger*TurnTableSpeed;
            telemetry.addData("Left Trigger TurnTable Left", "Pressed");
        }
        if (gamepad2.right_trigger > 0) {
            robotMode=ATRobotMode.MANUAL;
            TurnTablePosition += -gamepad2.right_trigger*TurnTableSpeed;
            telemetry.addData("Right Trigger TurnTable Right", "Pressed");
        }
    }

    private void partialManualControl(){
        if (gamepad2.right_bumper && gamepad2.x){
             this.robotMode = ATRobotMode.AUTO_RED_RIGHT_HIGH_SETUP;
        }
        if (gamepad2.left_bumper && gamepad2.y){
            robotMode=ATRobotMode.MANUAL;
        }
        if (gamepad2.b){
            robotMode=ATRobotMode.MANUAL;
            wrist.getController().pwmEnable();
        }
        if (gamepad2.a){
            robotMode=ATRobotMode.MANUAL;
            wrist.getController().pwmDisable();
        }
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

}