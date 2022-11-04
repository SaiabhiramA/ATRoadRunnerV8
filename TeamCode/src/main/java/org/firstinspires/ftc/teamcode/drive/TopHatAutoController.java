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
    boolean desiredClawPosition ;
    double desiredArmPosition ;
    double desiredElbowPosition ;
    double desiredTurnTablePosition ;
    int step = 0 ;
    int teleOpStep=0;
    int noOfCones = 0 ;
    MecanumDriveAT drive;
    ATRobotEnumeration robotMode= ATRobotEnumeration.RESET;

    public void fullyInitializeRobot(Telemetry tl, Gamepad gp1, Gamepad gp2, ATRobotEnumeration rMode, HardwareMap hardwareMapAT) {
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
        ElbowSpeed = 100;
        TurnTablePosition = 0;
        TurnTableSpeed = 10;
        ArmPosition = 0;
        ArmSpeed = 100;
        ClawSpeed = 0.05;
        ClawPosition = 0;
        WristSpeed = 0.01;
        WristPosition = 1;
        ArmVelocity=5000;
        ElbowVelocity=5000;
        TurnTableVelocity=1000;
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turntable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ResetTopHat();
      }

    public void basicInitializeRobot(HardwareMap hardwareMapAT, Telemetry tl, Gamepad gp1, Gamepad gp2, ATRobotEnumeration rMode) {
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
        ElbowPosition = elbow.getCurrentPosition();
        ElbowSpeed = 100;
        TurnTablePosition = elbow.getCurrentPosition();
        TurnTableSpeed = 10;
        ArmPosition = elbow.getCurrentPosition();
        ArmSpeed = 100;
        ClawSpeed = 0.05;
        ClawPosition = claw.getPosition();
        WristSpeed = 0.01;
        WristPosition = wrist.getPosition();
        ArmVelocity=5000;
        ElbowVelocity=5000;
        TurnTableVelocity=1000;

        telemetry.addData("wrist position", wrist.getPosition());
        telemetry.addData("wrist position while moving", wrist.getController().getServoPosition(0));
        telemetry.addData("claw position", claw.getPosition());
        telemetry.addData("arm position", arm.getCurrentPosition());
        telemetry.addData("elbow position", elbow.getCurrentPosition());
        telemetry.addData("turntable position", turntable.getCurrentPosition());
        telemetry.addData("testCounter", testCounter);

    }

    public void ResetTopHat(){
        robotMode= ATRobotEnumeration.RESET;
        ResetWristNClaw();
        ResetArmUp();
        ResetArmElbow();
        ResetTurnTable();
        ResetArmDown();
     }

     public void ResetTopHatStop(){
        robotMode= ATRobotEnumeration.STOP;
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
        if (robotMode==ATRobotEnumeration.TELE_OP_AUTO){
            moveTopHatPosition(-1,false,arm.getCurrentPosition(),elbow.getCurrentPosition(),turntable.getCurrentPosition());
        }
        if (robotMode == ATRobotEnumeration.MANUAL) {
            setTopHatMotorsVelocity(2000, 5000, 5000);
        }
        if (robotMode== ATRobotEnumeration.AUTO_TOPHAT_MOVE_BEGIN){
            if (teleOpStep==0){
                openClaw(desiredClawPosition);
                sleep(1000);
                moveTopHatPosition(.1,desiredClawPosition,4300,-2600,turntable.getCurrentPosition());
                teleOpStep=1;
            }

            if (teleOpStep == 1 && isInRange(-2600, elbow.getCurrentPosition())
                    && isInRange(4300, arm.getCurrentPosition())){
                moveTopHatPosition(.1,desiredClawPosition,4300,-2600,desiredTurnTablePosition);
                teleOpStep=2;
            }
            if (teleOpStep == 2 && isInRange(desiredTurnTablePosition, turntable.getCurrentPosition())){
                moveTopHatPosition(.1,desiredClawPosition,desiredArmPosition,desiredElbowPosition,desiredTurnTablePosition);
                teleOpStep=3;

            }
            if (teleOpStep==3 && isInRange(desiredElbowPosition, elbow.getCurrentPosition())
                    && isInRange(desiredArmPosition, arm.getCurrentPosition())
                    && isInRange(desiredTurnTablePosition, turntable.getCurrentPosition()) ){
                setWristPosition(desiredWristPosition);
                sleep(500);
                teleOpStep=0;
                robotMode= ATRobotEnumeration.AUTO_TOPHAT_MOVE_END;
            }

        }
        fullManualControl();
        partialManualControl();
        moveTopHatMotors();
        setMotorNServoMaximums();
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
        ElbowPosition = Math.min(Math.max(ElbowPosition, -8500), -100);
        ArmPosition=Math.min(Math.max(ArmPosition, 100), 4300);
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
        while (arm.getCurrentPosition() > -200) {
            setMotorPosition(-200, arm, ArmVelocity);
        }
        ArmPosition=arm.getCurrentPosition();
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
        sleep(500);

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
      if (this.robotMode== ATRobotEnumeration.AUTO_RED_RIGHT_HIGH_PICK_CONE) {
          redAllianceRightAutonHigh();
        }
      if (this.robotMode== ATRobotEnumeration.AUTO_RED_RIGHT_MEDIUM_PICK_CONE){
          redAllianceRightAutonMedium();
      }

    }
    // This method is used to have TopHat move in a controlled manner from current to desired position
    private void setTopHatPosition(double desiredWristPos, boolean desiredClawPos , int desiredArmPos , int desiredElbowPos , double desiredTurnTablePos ){
        desiredArmPosition = desiredArmPos;
        desiredTurnTablePosition = desiredTurnTablePos;
        desiredElbowPosition = desiredElbowPos;
        desiredWristPosition=desiredWristPos;
        desiredClawPosition=desiredClawPos;
    }
    public void moveTopHatPosition(double desiredWristPosition, boolean desiredClawPosition , double desiredArmPosition , double desiredElbowPosition , double desiredTurnTablePosition ){
        ArmPosition = desiredArmPosition;
        TurnTablePosition = desiredTurnTablePosition;
        ElbowPosition = desiredElbowPosition;
        if (desiredWristPosition==-1){
            wrist.getController().pwmDisable();
            claw.getController().pwmDisable();
        }
        else {
            setWristPosition(desiredWristPosition);
            openClaw(desiredClawPosition);
        }
        moveTopHatMotors();
    }

    public void stopTopHatMovement(){
        moveTopHatPosition(-1,false,arm.getCurrentPosition(),elbow.getCurrentPosition(), turntable.getCurrentPosition());
    }

    public void setRobotMode(ATRobotEnumeration rMode){
        robotMode=rMode;
    }

    public ATRobotEnumeration getRobotMode(){
        return robotMode;
    }


    public void redAllianceRightAutonHigh(){
        if (this.robotMode== ATRobotEnumeration.AUTO_RED_RIGHT_HIGH_SETUP) {
            moveTopHatPosition(-1, false, 4300, -20, 200);
            noOfCones=0;
        }
        if (this.robotMode== ATRobotEnumeration.AUTO_RED_RIGHT_HIGH_PICK_CONE) {
            if (step == 0) {
                if (noOfCones == 0) {
                    desiredWristPosition=.3894;
                    desiredElbowPosition = -5065;
                    desiredArmPosition = 1558;
                    desiredTurnTablePosition=197;
                } else if (noOfCones == 1) {
                    desiredWristPosition=.3794;
                    desiredElbowPosition = -4576;
                    desiredArmPosition = 1063;
                    desiredTurnTablePosition=197;
                } else if (noOfCones == 2) {
                    desiredWristPosition=.4694;
                    desiredElbowPosition = -4242;
                    desiredArmPosition = 855;
                    desiredTurnTablePosition=197;
                } else if (noOfCones == 3) {
                    desiredWristPosition=.3694;
                    desiredElbowPosition = -4452;
                    desiredArmPosition = 779;
                    desiredTurnTablePosition=197;
                } else if (noOfCones == 4) {
                    desiredWristPosition=.4194;
                    desiredElbowPosition = -4450;
                    desiredArmPosition = 799;
                    desiredTurnTablePosition=197;
                }
                moveTopHatPosition(0.1, false, desiredArmPosition, desiredElbowPosition, desiredTurnTablePosition);
                openClaw(true);
                step = 1;
            }
            //Pickup cone
            else if (step == 1 && isInRange(desiredElbowPosition, elbow.getCurrentPosition())
                    && isInRange(desiredArmPosition, arm.getCurrentPosition())
                    && isInRange(desiredTurnTablePosition, turntable.getCurrentPosition())) {
                telemetry.addData(" LOWER WRIST", "Pressed");
                setWristPosition(desiredWristPosition);
                sleep(500);
                openClaw(false);
                sleep(1000);
                setWristPosition(.10);
                sleep(500);
                moveTopHatPosition(0.1, false, 4200, -3500, 197);
                step = 2;
            }
            else if (step == 2 && isInRange(-3500, elbow.getCurrentPosition())
                    && isInRange(4200, arm.getCurrentPosition()) && isInRange(197, turntable.getCurrentPosition())) {
                moveTopHatPosition(0.1, false, 4200, -3500, 1507);
                step = 3;
            }

            else if (step == 3 && isInRange(-3500, elbow.getCurrentPosition())
                    && isInRange(4200, arm.getCurrentPosition()) && isInRange(1507, turntable.getCurrentPosition())) {
                moveTopHatPosition(0.6094, false, 4005, -4789, 1507);
                step = 4;
            }
            else if (step == 4 && isInRange(-4789, elbow.getCurrentPosition())
                    && isInRange(4005, arm.getCurrentPosition())
                    && isInRange(1507, turntable.getCurrentPosition())) {
                //setWristPosition(.64);
                //sleep(500);
                openClaw(true);
                sleep(500);
                step = 5;
                moveTopHatPosition(0.1, false, 4200, -3500, 1507);
            }
            else if (step == 5 && isInRange(-3500, elbow.getCurrentPosition())
                    && isInRange(4200, arm.getCurrentPosition())
                    && isInRange(1507, turntable.getCurrentPosition())) {
                moveTopHatPosition(0.1, false, 4200, -3500, 197);
                step = 6;
            }
            else if (step == 6 && isInRange(-3500, elbow.getCurrentPosition())
                    && isInRange(4200, arm.getCurrentPosition())
                    && isInRange(197, turntable.getCurrentPosition())) {
                noOfCones = noOfCones + 1;
                step = 0;
            }

            if (noOfCones>2){
                robotMode= ATRobotEnumeration.AUTO_RED_RIGHT_MEDIUM_PARK;
                moveTopHatPosition(0.9, false, 4224, -200, 216);
            }
        }
    }
    public boolean areFiveConesDone(){
        if (noOfCones>2) {
            telemetry.addData("No Of COnes", noOfCones);
            return true;
        }
        else
        {
            telemetry.addData("No Of COnes", noOfCones);
            return false;
        }
    }
    public void redAllianceRightAutonMedium(){
        if (this.robotMode== ATRobotEnumeration.AUTO_RED_RIGHT_MEDIUM_SETUP) {
            moveTopHatPosition(-1, false, 4300, -20, 200);
            noOfCones=0;
        }
        if (this.robotMode== ATRobotEnumeration.AUTO_RED_RIGHT_MEDIUM_PICK_CONE) {
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
                    robotMode= ATRobotEnumeration.AUTO_RED_RIGHT_MEDIUM_PARK;
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
            robotMode= ATRobotEnumeration.MANUAL;
            WristPosition += WristSpeed;
            setWristPosition(WristPosition);
            telemetry.addData("DpadUp Wrist Up", "Pressed");

        }
        if (gamepad2.dpad_down) {
            robotMode= ATRobotEnumeration.MANUAL;
            WristPosition += -WristSpeed;
            setWristPosition(WristPosition);
            telemetry.addData("DpadDown Wrist Down", "Pressed");
        }
        if (gamepad2.dpad_right) {
            robotMode= ATRobotEnumeration.MANUAL;
            openClaw(false);
            HoldCone = true;
            telemetry.addData("DpadRight Claw Close", "Pressed");
        }
        if (gamepad2.dpad_left) {
            robotMode= ATRobotEnumeration.MANUAL;
            openClaw(true);
            HoldCone = false;
            telemetry.addData("DpadLeft Claw Open", "Pressed");
        }
        if (gamepad2.left_stick_y != 0) {
            robotMode= ATRobotEnumeration.MANUAL;
            ArmPosition += -gamepad2.left_stick_y*ArmSpeed;
        }
        if (gamepad2.right_stick_y != 0) {
            robotMode= ATRobotEnumeration.MANUAL;
            ElbowPosition += gamepad2.right_stick_y*ElbowSpeed;
        }
        if (gamepad2.left_trigger > 0) {
            robotMode= ATRobotEnumeration.MANUAL;
            TurnTablePosition += gamepad2.left_trigger*TurnTableSpeed;
            telemetry.addData("Left Trigger TurnTable Left", "Pressed");
        }
        if (gamepad2.right_trigger > 0) {
            robotMode= ATRobotEnumeration.MANUAL;
            TurnTablePosition += -gamepad2.right_trigger*TurnTableSpeed;
            telemetry.addData("Right Trigger TurnTable Right", "Pressed");
        }
    }

    private void partialManualControl(){
        if (!gamepad2.right_bumper && gamepad2.left_bumper && gamepad2.x){
             robotMode= ATRobotEnumeration.AUTO_TOPHAT_MOVE_BEGIN;
             /*ArmPosition=1590;
             TurnTablePosition=1665;
             ElbowPosition=-5936;
             setWristPosition(.3094);
             openClaw(true);*/
             setTopHatPosition(.3999,true,1359,-5347,1649);
        }

        if (gamepad2.right_bumper && gamepad2.left_bumper && gamepad2.x){
            robotMode= ATRobotEnumeration.AUTO_TOPHAT_MOVE_BEGIN;
            /*openClaw(false);
            sleep(1000);
            setWristPosition(.02);
            ArmPosition=3565;
            TurnTablePosition=471;
            ElbowPosition=-3714;
            setWristPosition(.6599);*/
            setTopHatPosition(.3999,true,1359,-5347,1649);

            if (isInRange(-5347, elbow.getCurrentPosition())
                    && isInRange(1359, arm.getCurrentPosition())
                    && isInRange(1649, turntable.getCurrentPosition()) ) {
                setTopHatPosition(.1, false, 4300, -2600, 1000);
            }
        }


        if (gamepad2.right_bumper && gamepad2.y){
            robotMode= ATRobotEnumeration.AUTO_TOPHAT_MOVE_BEGIN;
            /*openClaw(false);
            sleep(1000);
            setWristPosition(.02);
            ArmPosition=3565;
            TurnTablePosition=471;
            ElbowPosition=-3714;
            setWristPosition(.6599);*/

            setTopHatPosition(.6777,false,3946,-4309,605);

         }



        if (gamepad2.right_bumper && gamepad2.b){
            robotMode= ATRobotEnumeration.AUTO_TOPHAT_MOVE_BEGIN;
            /*openClaw(false);
            sleep(1000);
            setWristPosition(.02);
            ArmPosition=3565;
            TurnTablePosition=471;
            ElbowPosition=-3714;
            setWristPosition(.6599);*/

            setTopHatPosition(.54,false,4302,-5618,614);

        }

        if (gamepad2.left_bumper && gamepad2.a){
            robotMode= ATRobotEnumeration.AUTO_TOPHAT_MOVE_BEGIN;
            /*openClaw(false);
            sleep(1000);
            setWristPosition(.02);
            ArmPosition=3565;
            TurnTablePosition=471;
            ElbowPosition=-3714;
            setWristPosition(.6599);*/

            setTopHatPosition(.2694,false,4302,-7901,1280);

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