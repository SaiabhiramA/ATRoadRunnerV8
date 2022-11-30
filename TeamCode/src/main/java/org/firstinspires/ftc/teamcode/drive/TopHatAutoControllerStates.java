package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TopHatAutoControllerStates {

    private DcMotor arm;
    private DcMotor elbow;
    private DcMotor turntable;
    private Servo wrist;
    private Servo rightClaw;
    private Servo leftClaw;
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
    double RightClawPosition;
    double LeftClawPosition;
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
    int desiredNoOfConesToPick=1;
    double robotMidPointWristPosition=.1;
    boolean robotMidPointClawPosition=false ;
    public final double armMultiplier=.51282; // 60 RPM to 117 RPM motor conversion
    public final double elbowMultiplier=.26923;// 84 RPM to 312 RPM motor conversion
    public final double turnTableMultiplier = .9; // this is adjusted to compensate new turntable gear ratio & centering
    public final double armMultiplier_N=1;
    public final double elbowMultiplier_N=1;
    public final double turnTableMultiplier_N =1;

    double robotMidPointArmPosition=4200*armMultiplier;
    double robotMidPointElbowPosition=-3500*elbowMultiplier;
    double robotMidPointTurnTablePosition=945*turnTableMultiplier;
    int step = 0 ;
    double teleOpStep=0;
    int noOfCones = 0 ;
    MecanumDriveAT drive;
    ATRobotEnumeration tophatAction;

    ATRobotEnumeration tophatMode;
    ATRobotEnumeration substationHSide;
    ATRobotEnumeration substationMSide;
    ATRobotEnumeration substationPickConfig;
    double subTTHighPickupPos;
    double subArmHighPickupPos;
    double subElbowHighPickupPos;
    double subWristHighPickupPos;

    double subTTHighDropPos;
    double subArmHighDropPos;
    double subElbowHighDropPos;
    double subWristHighDropPos;

    double subTTMedPickupPos;
    double subArmMedPickupPos;
    double subElbowMedPickupPos;
    double subWristMedPickupPos;

    double subTTMedDropPos;
    double subArmMedDropPos;
    double subElbowMedDropPos;
    double subWristMedDropPos;
    public double sleepMSecReq;
    public ATRobotEnumeration sleepMode;



    public void fullyInitializeRobot(Telemetry tl, Gamepad gp1, Gamepad gp2, ATRobotEnumeration rMode, HardwareMap hardwareMapAT) {
        tophatAction =rMode;
        telemetry = tl;
        gamepad1=gp1;
        gamepad2=gp2;
        arm = hardwareMapAT.get(DcMotor.class, "arm");
        elbow = hardwareMapAT.get(DcMotor.class, "elbow");
        turntable = hardwareMapAT.get(DcMotor.class, "turntable");
        wrist = hardwareMapAT.get(Servo.class, "wrist");
        rightClaw = hardwareMapAT.get(Servo.class, "rightclaw");
        leftClaw = hardwareMapAT.get(Servo.class, "leftclaw");
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
        RightClawPosition = 0;
        LeftClawPosition = 1;
        WristSpeed = 0.01;
        WristPosition = 1;
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turntable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turntable.setDirection(DcMotorSimple.Direction.REVERSE);
        setTopHatMotorPowers(1000, 1000,1000);
        ResetTopHat();
        setTopHatMotorPowers(1000, 1000,1000);
      }

    public void basicInitializeRobot(HardwareMap hardwareMapAT, Telemetry tl, Gamepad gp1, Gamepad gp2, ATRobotEnumeration rMode) {
        tophatAction =rMode;
        telemetry = tl;
        gamepad1=gp1;
        gamepad2=gp2;
        arm = hardwareMapAT.get(DcMotor.class, "arm");
        elbow = hardwareMapAT.get(DcMotor.class, "elbow");
        turntable = hardwareMapAT.get(DcMotor.class, "turntable");
        wrist = hardwareMapAT.get(Servo.class, "wrist");
        rightClaw = hardwareMapAT.get(Servo.class, "rightclaw");
        leftClaw = hardwareMapAT.get(Servo.class, "leftclaw");
        turntabletouch = hardwareMapAT.get(TouchSensor.class, "turntabletouch");
        armdowntouch = hardwareMapAT.get(TouchSensor.class, "armdowntouch");
        armuptouch = hardwareMapAT.get(TouchSensor.class, "armuptouch");
        elbowtouch = hardwareMapAT.get(TouchSensor.class, "elbowtouch");
        turntable.setDirection(DcMotorSimple.Direction.REVERSE);
        // Set servo to mid position
        ElbowPosition = elbow.getCurrentPosition();
        ElbowSpeed = 10;
        TurnTablePosition = turntable.getCurrentPosition();
        TurnTableSpeed = 10;
        ArmPosition = arm.getCurrentPosition();
        ArmSpeed = 10;
        ClawSpeed = 0.05;
        RightClawPosition = rightClaw.getPosition();
        LeftClawPosition = leftClaw.getPosition();
        WristSpeed = 0.01;
        WristPosition = wrist.getPosition();
        setTopHatMotorPowers(1000, 1000,1000);

        switch (ATGlobalStorage.autonModeName) {
            case RED_RIGHT_HIGH_DROP: {
                subTTHighPickupPos =1737*turnTableMultiplier;
                subArmHighPickupPos =1160*armMultiplier;
                subElbowHighPickupPos =-5345*elbowMultiplier;
                subWristHighPickupPos =.3483;

                subTTHighDropPos=519*turnTableMultiplier;
                subArmHighDropPos =3683*armMultiplier;
                subElbowHighDropPos =-4321*elbowMultiplier;
                subWristHighDropPos =.6555;

                subTTMedPickupPos=175*turnTableMultiplier;
                subArmMedPickupPos=1339*armMultiplier;
                subElbowMedPickupPos=-5615*elbowMultiplier;
                subWristMedPickupPos=.34;

                subTTMedDropPos=1579*turnTableMultiplier;
                subArmMedDropPos=2045*armMultiplier;
                subElbowMedDropPos=-2520*elbowMultiplier;
                subWristMedDropPos=.7794;
                substationHSide=ATRobotEnumeration.SUBSTATION_LEFT;
                substationMSide=ATRobotEnumeration.SUBSTATION_RIGHT;
            }
            break;
            case RED_LEFT_HIGH_DROP: {
                subTTHighPickupPos =266*turnTableMultiplier;
                subArmHighPickupPos =1041*armMultiplier;
                subElbowHighPickupPos =-5139*elbowMultiplier;
                subWristHighPickupPos =.3483;

                subTTHighDropPos=1526*turnTableMultiplier;
                subArmHighDropPos =3874*armMultiplier;
                subElbowHighDropPos =-4605*elbowMultiplier;
                subWristHighDropPos =.6255;

                subTTMedPickupPos=1834*turnTableMultiplier;
                subArmMedPickupPos=911*armMultiplier;
                subElbowMedPickupPos=-4692*elbowMultiplier;
                subWristMedPickupPos=.4294;

                subTTMedDropPos=503*turnTableMultiplier;
                subArmMedDropPos=2018*armMultiplier;
                subElbowMedDropPos=-2429*elbowMultiplier;
                subWristMedDropPos=.7794;
                substationHSide=ATRobotEnumeration.SUBSTATION_RIGHT;
                substationMSide=ATRobotEnumeration.SUBSTATION_LEFT;
            }
            break;
            case BLUE_RIGHT_HIGH_DROP: {

                subTTHighPickupPos =1696*turnTableMultiplier;
                subArmHighPickupPos =767*armMultiplier;
                subElbowHighPickupPos =-4667*elbowMultiplier;
                subWristHighPickupPos =.3883;

                subTTHighDropPos=527*turnTableMultiplier;
                subArmHighDropPos =3373*armMultiplier;
                subElbowHighDropPos =-3694*elbowMultiplier;
                subWristHighDropPos =.6655;

                subTTMedPickupPos=150*turnTableMultiplier;
                subArmMedPickupPos=782*armMultiplier;
                subElbowMedPickupPos=-4759*elbowMultiplier;
                subWristMedPickupPos=.36;

                subTTMedDropPos=1507*turnTableMultiplier;
                subArmMedDropPos=2042*armMultiplier;
                subElbowMedDropPos=-2544*elbowMultiplier;
                subWristMedDropPos=.7694;
                substationHSide=ATRobotEnumeration.SUBSTATION_LEFT;
                substationMSide=ATRobotEnumeration.SUBSTATION_RIGHT;
            }
            break;
            case BLUE_LEFT_HIGH_DROP: {

                subTTHighPickupPos =281*turnTableMultiplier;
                subArmHighPickupPos =1155*armMultiplier;
                subElbowHighPickupPos =-5345*elbowMultiplier;
                subWristHighPickupPos =.3383;

                subTTHighDropPos=1550*turnTableMultiplier;
                subArmHighDropPos =3516*armMultiplier;
                subElbowHighDropPos =-3763*elbowMultiplier;
                subWristHighDropPos =.7155;

                subTTMedPickupPos=1866*turnTableMultiplier;
                subArmMedPickupPos=749*armMultiplier;
                subElbowMedPickupPos=-4602*elbowMultiplier;
                subWristMedPickupPos=.3999;

                subTTMedDropPos=513*turnTableMultiplier;
                subArmMedDropPos=1668*armMultiplier;
                subElbowMedDropPos=-2128*elbowMultiplier;
                subWristMedDropPos=.7894;
                substationHSide=ATRobotEnumeration.SUBSTATION_RIGHT;
                substationMSide=ATRobotEnumeration.SUBSTATION_LEFT;
            }
            break;
        }

        telemetry.addData("wrist position", wrist.getPosition());
        telemetry.addData("wrist position while moving", wrist.getController().getServoPosition(0));
        telemetry.addData("rightClaw position", rightClaw.getPosition());
        telemetry.addData("leftClaw position", leftClaw.getPosition());
        telemetry.addData("arm position", arm.getCurrentPosition());
        telemetry.addData("elbow position", elbow.getCurrentPosition());
        telemetry.addData("turntable position", turntable.getCurrentPosition());
        telemetry.addData("testCounter", testCounter);
    }

    public void ResetTopHat(){
        tophatAction = ATRobotEnumeration.RESET;
        tophatMode = ATRobotEnumeration.AUTO;
        ResetWristNClaw();
        ResetArmUp();
        ResetArmElbow();
        ResetTurnTable();
        ResetArmDown();
     }

     public void ResetTopHatStop(){
        tophatAction = ATRobotEnumeration.STOP;
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
        fullManualControl_New();
        partialManualControl();
        if (tophatAction == ATRobotEnumeration.MANUAL) {
            tophatMode = ATRobotEnumeration.MANUAL;
            sleepMode = ATRobotEnumeration.SLEEP_MODE_OFF;
            teleOpStep=0;
        }
        if (sleepMode !=ATRobotEnumeration.SLEEP_MODE_ON) {
            if (tophatAction == ATRobotEnumeration.AUTO_TOPHAT_ONEWAY_MOVE_BEGIN) {
                tophatMode = ATRobotEnumeration.FULL_AUTO;
                moveTopHatOneWayInOrder();
            }

            if (tophatAction == ATRobotEnumeration.PICK_CONE_READY_TO_NAVIGATE) {
                tophatMode = ATRobotEnumeration.FULL_AUTO;
                conePickupToNavigate();
            }

            if (tophatAction == ATRobotEnumeration.PICK_CONE_DROP_HIGH_IN_LOOP) {
                tophatMode = ATRobotEnumeration.FULL_AUTO;
                pickFromSubstationDropInHighJunction();
            }

            if (tophatAction == ATRobotEnumeration.PICK_CONE_DROP_MEDIUM_IN_LOOP) {
                tophatMode = ATRobotEnumeration.FULL_AUTO;
                pickFromSubstationDropInMedJunction();
            }

            if (tophatAction == ATRobotEnumeration.PICK_CONE_SET_TO_DRPOP_HIGH) {
                tophatMode = ATRobotEnumeration.FULL_AUTO;
                pickNSetforHighDrop();
            }

            if (tophatAction == ATRobotEnumeration.PICK_CONE_SET_TO_DRPOP_MEDIUM) {
                tophatMode = ATRobotEnumeration.FULL_AUTO;
                pickNSetforMediumDrop();
            }
        }
        moveTopHatMotors();
        setMotorNServoMaximums();
        telemetry.addData("wrist position", wrist.getPosition());
        telemetry.addData("wrist position while moving", wrist.getController().getServoPosition(0));
        telemetry.addData("Right Claw position", rightClaw.getPosition());
        telemetry.addData("Left Claw position", leftClaw.getPosition());
        telemetry.addData("arm position", arm.getCurrentPosition());
        telemetry.addData("elbow position", elbow.getCurrentPosition());
        telemetry.addData("turntable position", turntable.getCurrentPosition());
        telemetry.addData("testCounter", testCounter);
        telemetry.addData("TopHat robot mode", tophatAction);
        telemetry.addData("TopHat Step # in Execution", teleOpStep);

    }

    /**
     * This is the method can be used to compensate any motor changes instead of making changes everywhere
     * This can be planned later but not now given amount of testing needed before the competition
     */
    private void moveTopHatMotors(){
        if (!armuptouch.isPressed()) {
            setMotorPosition((int) ArmPosition, arm, ArmVelocity);
        }
        else{
            setMotorPosition(2200, arm, ArmVelocity);
        }
        if (!armdowntouch.isPressed()) {
            setMotorPosition((int) ArmPosition, arm, ArmVelocity);
        }
        else{
            setMotorPosition(100, arm, ArmVelocity);
        }

        if (!elbowtouch.isPressed()) {
            setMotorPosition((int) ElbowPosition, elbow, ElbowVelocity);
        }
        else{
            setMotorPosition( -100, elbow, ElbowVelocity);
        }
        if (!turntabletouch.isPressed()) {
            setMotorPosition((int) (TurnTablePosition), turntable, TurnTableVelocity);
        }
        else{
            setMotorPosition(100, turntable, TurnTableVelocity);
        }

    }

    private void setMotorNServoMaximums(){
        RightClawPosition = Math.min(Math.max(RightClawPosition, 0), 1);
        LeftClawPosition = Math.min(Math.max(LeftClawPosition, 0), 1);
        WristPosition = Math.min(Math.max(WristPosition, 0), 1);

        TurnTablePosition = Math.min(Math.max(TurnTablePosition, 100), 1900*turnTableMultiplier);
        ElbowPosition = Math.min(Math.max(ElbowPosition, -8500*elbowMultiplier), -100);
        ArmPosition=Math.min(Math.max(ArmPosition, 100), 4300*armMultiplier);
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

        while (turntable.getCurrentPosition() < 10) {
            setMotorPosition(10, turntable, TurnTableVelocity);
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
        autonSleep(500);

    }
    private void setMotorPosition(int pos, DcMotor motor, int motorVel){
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
            RightClawPosition = .4;
            LeftClawPosition=.5;
        }
        else{
            RightClawPosition = 0;
            LeftClawPosition=1;
        }
        rightClaw.setPosition(RightClawPosition);
        leftClaw.setPosition(LeftClawPosition);
    }

    // This method is used to have TopHat move in a controlled manner from current to desired position
    public void setTopHatPosition(double desiredWristPos, boolean desiredClawPos , double desiredArmPos , double desiredElbowPos , double desiredTurnTablePos){
        desiredArmPosition = desiredArmPos;
        desiredTurnTablePosition = desiredTurnTablePos;
        desiredElbowPosition = desiredElbowPos;
        desiredWristPosition=desiredWristPos;
        desiredClawPosition=desiredClawPos;
    }
    public void moveTopHatPosition(double desiredWristPos, boolean desiredClawPos , double desiredArmPos , double desiredElbowPos , double desiredTurnTablePos){
        ArmPosition = desiredArmPos;
        TurnTablePosition = desiredTurnTablePos;
        ElbowPosition = desiredElbowPos;
        if (desiredWristPos==-1){
            wrist.getController().pwmDisable();
            rightClaw.getController().pwmDisable();
            leftClaw.getController().pwmDisable();
        }
        else {
            setWristPosition(desiredWristPos);
            openClaw(desiredClawPos);
        }
        moveTopHatMotors();
    }

    public void stopTopHatMovement(){
        tophatAction = ATRobotEnumeration.KILL_ALL_ACTIONS;
        tophatMode = ATRobotEnumeration.AUTO;
        moveTopHatPosition(-1,false,arm.getCurrentPosition(),elbow.getCurrentPosition(), turntable.getCurrentPosition());
    }
    private void setTopHatMotorPowers(int armVel, int elbowVel, int turntableVel){
        ArmVelocity=armVel;
        ElbowVelocity=elbowVel;
        TurnTableVelocity=turntableVel;
    }

    public void setTophatAction(ATRobotEnumeration rMode){
        tophatAction =rMode;
    }

    public ATRobotEnumeration getTophatAction(){
        return tophatAction;
    }




    private boolean isInRange(double desiredValue, double inputValue){
        return (Math.abs(inputValue)>=Math.abs(desiredValue)-5) && (Math.abs(inputValue)<=Math.abs(desiredValue)+5);
    }

    public ATRobotEnumeration getTophatMode() {
        return tophatMode;
    }

    public void setTophatMode(ATRobotEnumeration tophatMode) {
        this.tophatMode = tophatMode;
    }

    private void fullManualControl(){
        if (gamepad2.dpad_up &&
                !gamepad2.dpad_down && !gamepad2.dpad_left && !gamepad2.dpad_right && gamepad2.left_trigger==0
                && gamepad2.right_trigger==0 && !gamepad2.left_bumper && !gamepad2.right_bumper)
        {
            tophatAction = ATRobotEnumeration.MANUAL;
            WristPosition += WristSpeed;
            setWristPosition(WristPosition);
            telemetry.addData("DpadUp Wrist Up", "Pressed");
        }
        if (gamepad2.dpad_down &&
                !gamepad2.dpad_up && !gamepad2.dpad_left && !gamepad2.dpad_right && gamepad2.left_trigger==0
                && gamepad2.right_trigger==0 && !gamepad2.left_bumper && !gamepad2.right_bumper)
        {
            tophatAction = ATRobotEnumeration.MANUAL;
            WristPosition += -WristSpeed;
            setWristPosition(WristPosition);
            telemetry.addData("DpadDown Wrist Down", "Pressed");
        }
        if (gamepad2.dpad_right &&
                !gamepad2.dpad_up && !gamepad2.dpad_left && !gamepad2.dpad_down && gamepad2.left_trigger==0
                && gamepad2.right_trigger==0 && !gamepad2.left_bumper && !gamepad2.right_bumper)
         {
            tophatAction = ATRobotEnumeration.MANUAL;
            openClaw(false);
            HoldCone = true;
            telemetry.addData("DpadRight Claw Close", "Pressed");
        }
        if (gamepad2.dpad_left &&
                !gamepad2.dpad_up && !gamepad2.dpad_right && !gamepad2.dpad_down && gamepad2.left_trigger==0
                && gamepad2.right_trigger==0 && !gamepad2.left_bumper && !gamepad2.right_bumper)
        {
            tophatAction = ATRobotEnumeration.MANUAL;
            openClaw(true);
            HoldCone = false;
            telemetry.addData("DpadLeft Claw Open", "Pressed");
        }

        if (gamepad2.left_stick_y != 0) {
            tophatAction = ATRobotEnumeration.MANUAL;
            ArmPosition += -gamepad2.left_stick_y*ArmSpeed;
            telemetry.addData("left stick y Arm Up or Down", "Pressed");
        }
        if (gamepad2.right_stick_y != 0) {
            tophatAction = ATRobotEnumeration.MANUAL;
            ElbowPosition += gamepad2.right_stick_y*ElbowSpeed;
            telemetry.addData("right stick y Elbow Up or Down", "Pressed");
        }

        if (gamepad2.left_trigger > 0 &&
                !gamepad2.dpad_up && !gamepad2.dpad_right && !gamepad2.dpad_left && !gamepad2.dpad_down
                && gamepad2.right_trigger==0 && !gamepad2.left_bumper && !gamepad2.right_bumper)
        {
            tophatAction = ATRobotEnumeration.MANUAL;
            TurnTablePosition += gamepad2.left_trigger*TurnTableSpeed;
            telemetry.addData("Left Trigger TurnTable Left", "Pressed");
        }

        if (gamepad2.right_trigger > 0 &&
                !gamepad2.dpad_up && !gamepad2.dpad_right && !gamepad2.dpad_left && !gamepad2.dpad_down
                && gamepad2.left_trigger==0 && !gamepad2.left_bumper && !gamepad2.right_bumper)
        {
            tophatAction = ATRobotEnumeration.MANUAL;
            TurnTablePosition += -gamepad2.right_trigger*TurnTableSpeed;
            telemetry.addData("Right Trigger TurnTable Right", "Pressed");
        }
    }

    private void fullManualControl_New(){
        if (gamepad2.dpad_up && gamepad2.left_bumper && gamepad2.right_bumper)
        {
            tophatAction = ATRobotEnumeration.MANUAL;
            WristPosition += WristSpeed;
            setWristPosition(WristPosition);
            telemetry.addData("DpadUp Wrist Up", "Pressed");
        }
        if (gamepad2.dpad_down && gamepad2.left_bumper && gamepad2.right_bumper)
        {
            tophatAction = ATRobotEnumeration.MANUAL;
            WristPosition += -WristSpeed;
            setWristPosition(WristPosition);
            telemetry.addData("DpadDown Wrist Down", "Pressed");
        }
        if (gamepad2.dpad_right && gamepad2.left_bumper && gamepad2.right_bumper)
        {
            tophatAction = ATRobotEnumeration.MANUAL;
            openClaw(false);
            HoldCone = true;
            telemetry.addData("DpadRight Claw Close", "Pressed");
        }
        if (gamepad2.dpad_left && gamepad2.left_bumper && gamepad2.right_bumper)
        {
            tophatAction = ATRobotEnumeration.MANUAL;
            openClaw(true);
            HoldCone = false;
            telemetry.addData("DpadLeft Claw Open", "Pressed");
        }

        if (gamepad2.left_stick_y != 0 && gamepad2.left_bumper && gamepad2.right_bumper) {
            tophatAction = ATRobotEnumeration.MANUAL;
            ArmPosition += -gamepad2.left_stick_y*ArmSpeed;
            telemetry.addData("left stick y Arm Up or Down", "Pressed");
        }
        if (gamepad2.right_stick_y != 0 && gamepad2.left_bumper && gamepad2.right_bumper) {
            tophatAction = ATRobotEnumeration.MANUAL;
            ElbowPosition += gamepad2.right_stick_y*ElbowSpeed;
            telemetry.addData("right stick y Elbow Up or Down", "Pressed");
        }

        if (gamepad2.left_trigger > 0  && gamepad2.left_bumper && gamepad2.right_bumper)
        {
            tophatAction = ATRobotEnumeration.MANUAL;
            TurnTablePosition += gamepad2.left_trigger*TurnTableSpeed;
            telemetry.addData("Left Trigger TurnTable Left", "Pressed");
        }

        if (gamepad2.right_trigger > 0  && gamepad2.left_bumper && gamepad2.right_bumper)
        {
            tophatAction = ATRobotEnumeration.MANUAL;
            TurnTablePosition += -gamepad2.right_trigger*TurnTableSpeed;
            telemetry.addData("Right Trigger TurnTable Right", "Pressed");
        }
    }


    private void partialManualControl(){

        /**
         * This is to preset TopHat for left ground junction drop         *
         */
        if (!gamepad2.right_bumper && gamepad2.left_bumper && gamepad2.x){
             tophatAction = ATRobotEnumeration.AUTO_TOPHAT_ONEWAY_MOVE_BEGIN;
             teleOpStep=0;
             setTopHatPosition(.14,false,1902*armMultiplier,-7394*elbowMultiplier,1382*turnTableMultiplier);
        }
        /**
         * This is to preset TopHat for right ground junction drop
         */
        if (gamepad2.right_bumper && !gamepad2.left_bumper && gamepad2.x){
            tophatAction = ATRobotEnumeration.AUTO_TOPHAT_ONEWAY_MOVE_BEGIN;
            teleOpStep=0;
            setTopHatPosition(.14,false,1902*armMultiplier,-7394*elbowMultiplier,494*turnTableMultiplier);
        }
       /**
         * This is to preset TopHat for left high junction drop
         */
        if (!gamepad2.right_bumper && gamepad2.left_bumper && gamepad2.y){
            tophatAction = ATRobotEnumeration.AUTO_TOPHAT_ONEWAY_MOVE_BEGIN;
            teleOpStep=0;
            setTopHatPosition(subWristHighDropPos,false, 3874*armMultiplier, -4605*elbowMultiplier,1700*turnTableMultiplier);
        }
        /**
         * This is to preset TopHat for right high junction drop
         */
        if (gamepad2.right_bumper && !gamepad2.left_bumper && gamepad2.y){
            tophatAction = ATRobotEnumeration.AUTO_TOPHAT_ONEWAY_MOVE_BEGIN;
            teleOpStep=0;
            setTopHatPosition(subWristHighDropPos,false,  3874*armMultiplier, -4605*elbowMultiplier,280*turnTableMultiplier);
        }

        /**
         * This is to preset TopHat for right medium junction drop
         */
        if (gamepad2.right_bumper && !gamepad2.left_bumper && gamepad2.b){
            tophatAction = ATRobotEnumeration.AUTO_TOPHAT_ONEWAY_MOVE_BEGIN;
            teleOpStep=0;
            setTopHatPosition(.87,false,2018*armMultiplier,-1888*elbowMultiplier,607*turnTableMultiplier);
        }
        /**
         * This is to preset TopHat for left medium junction drop
         */
        if (gamepad2.left_bumper && !gamepad2.right_bumper && gamepad2.b){
            tophatAction = ATRobotEnumeration.AUTO_TOPHAT_ONEWAY_MOVE_BEGIN;
            teleOpStep=0;
            setTopHatPosition(.87,false,2018*armMultiplier,-1888*elbowMultiplier,1507*turnTableMultiplier);
        }

        /**
         * This is to preset TopHat for left low junction drop
         */
        if (!gamepad2.right_bumper && gamepad2.left_bumper && gamepad2.a){
            tophatAction = ATRobotEnumeration.AUTO_TOPHAT_ONEWAY_MOVE_BEGIN;
            teleOpStep=0;
            setTopHatPosition(.8,false,391*armMultiplier,-1080*elbowMultiplier,1406*turnTableMultiplier);
        }
        /**
         * This is to preset TopHat for right low junction drop
         */
        if (gamepad2.right_bumper && !gamepad2.left_bumper && gamepad2.a){
            tophatAction = ATRobotEnumeration.AUTO_TOPHAT_ONEWAY_MOVE_BEGIN;
            teleOpStep=0;
            setTopHatPosition(.8,false,391*armMultiplier,-1080*elbowMultiplier,506*turnTableMultiplier);
        }

        /**
         * This is to preset TopHat to pickup cone from Alliance side specific substation for Medium Junction Drop
         * This needs to be added to the instruction sheet and not to analyze this as left or right instead focus on High vs
         * Medium Junction
         */

        if (gamepad2.left_trigger == 0 && gamepad2.right_trigger > 0 && gamepad2.x){
            tophatAction = ATRobotEnumeration.AUTO_TOPHAT_ONEWAY_MOVE_BEGIN;
            teleOpStep=0;
            setTopHatPosition(subWristMedPickupPos, true, subArmMedPickupPos, subElbowMedPickupPos, subTTMedPickupPos);
        }

        /**
         * This is to preset TopHat to pickup cone from Alliance side specific substation for High Junction Drop
         * This needs to be added to the instruction sheet and not to analyze this as left or right instead focus on High vs
         * Medium Junction
          */
        if (gamepad2.left_trigger > 0 && gamepad2.right_trigger == 0 && gamepad2.x){
            tophatAction = ATRobotEnumeration.AUTO_TOPHAT_ONEWAY_MOVE_BEGIN;
            teleOpStep=0;
            setTopHatPosition(subWristHighPickupPos, true, subArmHighPickupPos, subElbowHighPickupPos, subTTHighPickupPos);
        }

        /**
         * This is to preset TopHat to pickup cone from Alliance side specific substation and
         * have TopHat stay at medium junction specific height
         * this can be used to pickup cone and navigate anywhere within the filed to either own a junction
         * or complete the circuit
         * *#################### NOT WORKING ####################
         */
        if (gamepad2.right_trigger>0 && gamepad2.left_trigger>0 && gamepad2.x){
            tophatAction = ATRobotEnumeration.PICK_CONE_READY_TO_NAVIGATE;
            teleOpStep=0;
        }

        /**
         * This is to pickup cone from Alliance side specific substation and have it ready to drop
         * in high junction
         */
        if (gamepad2.left_bumper && gamepad2.right_bumper && gamepad2.y){
            tophatAction = ATRobotEnumeration.PICK_CONE_SET_TO_DRPOP_HIGH;
            teleOpStep=0;
            substationPickConfig =ATRobotEnumeration.PICK_CONE_SET_TO_DRPOP_HIGH;
        }

        /**
         * This is to preset TopHat to pickup cone from Alliance side specific substation and
         * have TopHat continue dropping cones till action being interrupted by pressing any additional key
         */
        if (gamepad2.right_trigger>0 && gamepad2.left_trigger>0 && gamepad2.y){
            tophatAction = ATRobotEnumeration.PICK_CONE_DROP_HIGH_IN_LOOP;
            teleOpStep=0;
        }

        /**
         * This is to pickup cone from Alliance side specific substation and have it ready to drop
         * in medium junction
         */
        if (gamepad2.left_bumper && gamepad2.right_bumper && gamepad2.b){
            tophatAction = ATRobotEnumeration.PICK_CONE_SET_TO_DRPOP_MEDIUM;
            teleOpStep=0;
            substationPickConfig =ATRobotEnumeration.PICK_CONE_SET_TO_DRPOP_MEDIUM;
        }

        /**
         * This is to preset TopHat to pickup cone from Alliance side specific substation and
         * have TopHat continue dropping cones in Medium Junction till action being interrupted by pressing any additional key
         */
        if (gamepad2.right_trigger>0 && gamepad2.left_trigger>0 && gamepad2.b){
            tophatAction = ATRobotEnumeration.PICK_CONE_DROP_MEDIUM_IN_LOOP;
            teleOpStep=0;
        }
    }

    private final void teleOpSleep(long milliseconds) {
        sleepMSecReq=milliseconds/1000;
        sleepMode=ATRobotEnumeration.SLEEP_MODE_REQUEST;
    }

    private void moveTopHatOneWayInOrder(){
        if (teleOpStep==0){
            openClaw(desiredClawPosition);
            teleOpSleep(500);
            teleOpStep = 0.1;
        }
        else if (teleOpStep==0.1){
            moveTopHatPosition(.3, desiredClawPosition, 4300 * armMultiplier, -2600 * elbowMultiplier, turntable.getCurrentPosition());
            teleOpStep = 1;
        }
        else if (teleOpStep == 1 && isInRange(-2600*elbowMultiplier, elbow.getCurrentPosition())
                && isInRange(4300*armMultiplier, arm.getCurrentPosition())){
            moveTopHatPosition(.3,desiredClawPosition,4300*armMultiplier,-2600*elbowMultiplier,desiredTurnTablePosition);
            teleOpStep=2;
        }
        else if (teleOpStep == 2 && isInRange(desiredTurnTablePosition, turntable.getCurrentPosition())){
            moveTopHatPosition(.3,desiredClawPosition,desiredArmPosition,desiredElbowPosition,desiredTurnTablePosition);
            teleOpStep=3;
     }
        else if (teleOpStep==3 && isInRange(desiredElbowPosition, elbow.getCurrentPosition())
                && isInRange(desiredArmPosition, arm.getCurrentPosition())
                && isInRange(desiredTurnTablePosition, turntable.getCurrentPosition()) ) {
            setWristPosition(desiredWristPosition);
            teleOpSleep(500);
            teleOpStep=3.1;
        }
        else if (teleOpStep==3.1 && isInRange(desiredElbowPosition, elbow.getCurrentPosition())
                && isInRange(desiredArmPosition, arm.getCurrentPosition())
                && isInRange(desiredTurnTablePosition, turntable.getCurrentPosition()) ) {
            teleOpStep = 0;
            tophatAction = ATRobotEnumeration.AUTO_TOPHAT_ONEWAY_MOVE_END;
        }
    }

    private void conePickupToNavigate(){
        if (teleOpStep==0){
            openClaw(true);
            //teleOpSleep(500);
            teleOpStep = 0.1;
        }
        else if (teleOpStep==0.1){
            moveTopHatPosition(.1, false, 4300 * armMultiplier, -2600 * elbowMultiplier, turntable.getCurrentPosition());
            teleOpStep = 1;
        }
        else if (teleOpStep == 1 && isInRange(-2600*elbowMultiplier, elbow.getCurrentPosition())
                && isInRange(4300*armMultiplier, arm.getCurrentPosition())){
            moveTopHatPosition(subWristHighPickupPos,true,4300*armMultiplier,-2600*elbowMultiplier, subTTHighPickupPos);
            teleOpStep=2;
        }
        else if (teleOpStep == 2 && isInRange(subTTHighPickupPos, turntable.getCurrentPosition())){
            moveTopHatPosition(subWristHighPickupPos,true, subArmHighPickupPos, subElbowHighPickupPos, subTTHighPickupPos);
            teleOpStep=3;
        }
        else if (teleOpStep==3 && isInRange(subElbowHighPickupPos, elbow.getCurrentPosition())
                && isInRange(subArmHighPickupPos, arm.getCurrentPosition())
                && isInRange(subTTHighPickupPos, turntable.getCurrentPosition()) ){
            setWristPosition(subWristHighPickupPos);
            //teleOpSleep(500);
            teleOpStep=3.1;
        }
        else if (teleOpStep==3.1 && isInRange(subElbowHighPickupPos, elbow.getCurrentPosition())
                && isInRange(subArmHighPickupPos, arm.getCurrentPosition())
                && isInRange(subTTHighPickupPos, turntable.getCurrentPosition()) ) {
            openClaw(false);
            teleOpSleep(1000);
            teleOpStep=3.2;
        }
        else if (teleOpStep==3.2 && isInRange(subElbowHighPickupPos, elbow.getCurrentPosition())
                && isInRange(subArmHighPickupPos, arm.getCurrentPosition())
                && isInRange(subTTHighPickupPos, turntable.getCurrentPosition()) ) {
            setWristPosition(.1);
            teleOpSleep(500);
            teleOpStep=3.3;
        }
        else if (teleOpStep==3.3 && isInRange(subElbowHighPickupPos, elbow.getCurrentPosition())
                && isInRange(subArmHighPickupPos, arm.getCurrentPosition())
                && isInRange(subTTHighPickupPos, turntable.getCurrentPosition()) ) {
            teleOpStep=4;
            moveTopHatPosition(.1,false,4300*armMultiplier,-1800*elbowMultiplier, subTTHighPickupPos);
        }
        else if (teleOpStep==4 && isInRange(-1800*elbowMultiplier, elbow.getCurrentPosition())
                && isInRange(4300*armMultiplier, arm.getCurrentPosition())){
            teleOpStep=5;
            moveTopHatPosition(.1,false,4300*armMultiplier,-1800*elbowMultiplier,945*turnTableMultiplier);
        }
        else if (teleOpStep==5 && isInRange(945*turnTableMultiplier, turntable.getCurrentPosition())){
            teleOpStep=0;
            tophatAction = ATRobotEnumeration.FULL_AUTO;
        }
    }

    private void pickNSetforHighDrop(){
         if (teleOpStep==0){
             openClaw(false);
             teleOpSleep(500);
             teleOpStep=0.1;
        }
         else if (teleOpStep==0.1){
             setWristPosition(.1);
             teleOpSleep(500);
             teleOpStep=1;
         }
        else if (teleOpStep == 1){
            teleOpStep=0;
            tophatAction = ATRobotEnumeration.AUTO_TOPHAT_ONEWAY_MOVE_BEGIN;
            setTopHatPosition(subWristHighDropPos,false, subArmHighDropPos, subElbowHighDropPos,subTTHighDropPos);
        }
    }

    private void pickNSetforMediumDrop(){
        if (teleOpStep==0){
            openClaw(false);
            teleOpSleep(500);
            teleOpStep=0.1;
        }
        else if (teleOpStep==0.1){
            setWristPosition(.1);
            teleOpSleep(500);
            teleOpStep=1;
        }
        else if (teleOpStep == 1){
            teleOpStep=0;
            tophatAction = ATRobotEnumeration.AUTO_TOPHAT_ONEWAY_MOVE_BEGIN;
            setTopHatPosition(subWristMedDropPos,false, subArmMedDropPos, subElbowMedDropPos,subTTMedDropPos);
        }
    }

    private void pickFromSubstationDropInHighJunction() {
        if (teleOpStep == 0) {
            openClaw(true);
            //teleOpSleep(500);
            teleOpStep = 0.1;
        } else if (teleOpStep == 0.1) {
            moveTopHatPosition(.1, false, 4300 * armMultiplier, -2600 * elbowMultiplier, turntable.getCurrentPosition());
            teleOpStep = 1;
        } else if (teleOpStep == 1 && isInRange(-2600 * elbowMultiplier, elbow.getCurrentPosition())
                && isInRange(4300 * armMultiplier, arm.getCurrentPosition())) {
            moveTopHatPosition(subWristHighPickupPos, true, 4300 * armMultiplier, -2600 * elbowMultiplier, subTTHighPickupPos);
            teleOpStep = 2;
        } else if (teleOpStep == 2 && isInRange(subTTHighPickupPos, turntable.getCurrentPosition())) {
            moveTopHatPosition(subWristHighPickupPos, true, subArmHighPickupPos, subElbowHighPickupPos, subTTHighPickupPos);
            teleOpStep = 3;
        } else if (teleOpStep == 3 && isInRange(subElbowHighPickupPos, elbow.getCurrentPosition())
                && isInRange(subArmHighPickupPos, arm.getCurrentPosition())
                && isInRange(subTTHighPickupPos, turntable.getCurrentPosition())) {
            setWristPosition(subWristHighPickupPos);
            //teleOpSleep(1000);
            teleOpStep = 3.1;
        } else if (teleOpStep == 3.1 && isInRange(subElbowHighPickupPos, elbow.getCurrentPosition())
                && isInRange(subArmHighPickupPos, arm.getCurrentPosition())
                && isInRange(subTTHighPickupPos, turntable.getCurrentPosition())) {
            openClaw(false);
            teleOpSleep(1000);
            teleOpStep = 3.2;
        } else if (teleOpStep == 3.2 && isInRange(subElbowHighPickupPos, elbow.getCurrentPosition())
                && isInRange(subArmHighPickupPos, arm.getCurrentPosition())
                && isInRange(subTTHighPickupPos, turntable.getCurrentPosition())) {
            setWristPosition(.1);
            teleOpSleep(500);
            teleOpStep = 3.3;
        } else if (teleOpStep == 3.3 && isInRange(subElbowHighPickupPos, elbow.getCurrentPosition())
                && isInRange(subArmHighPickupPos, arm.getCurrentPosition())
                && isInRange(subTTHighPickupPos, turntable.getCurrentPosition())) {
            teleOpStep = 4;
            moveTopHatPosition(.1, false, 4300 * armMultiplier, -2600 * elbowMultiplier, subTTHighPickupPos);
        } else if (teleOpStep == 4 && isInRange(-2600 * elbowMultiplier, elbow.getCurrentPosition())
                && isInRange(4300 * armMultiplier, arm.getCurrentPosition())) {
            moveTopHatPosition(.1, false, 4300 * armMultiplier, -2600 * elbowMultiplier, subTTHighDropPos);
            teleOpStep = 5;
        } else if (teleOpStep == 5 && isInRange(subTTHighDropPos, turntable.getCurrentPosition())) {
            moveTopHatPosition(subWristHighDropPos, false, subArmHighDropPos, subElbowHighDropPos, subTTHighDropPos);
            teleOpStep = 6;
        } else if (teleOpStep == 6 && isInRange(subElbowHighDropPos, elbow.getCurrentPosition())
                && isInRange(subArmHighDropPos, arm.getCurrentPosition())
                && isInRange(subTTHighDropPos, turntable.getCurrentPosition())) {
            openClaw(true);
            teleOpSleep(500);
            teleOpStep=6.1;
        } else if (teleOpStep == 6.1 && isInRange(subElbowHighDropPos, elbow.getCurrentPosition())
                && isInRange(subArmHighDropPos, arm.getCurrentPosition())
                && isInRange(subTTHighDropPos, turntable.getCurrentPosition())) {
            tophatAction = ATRobotEnumeration.PICK_CONE_DROP_HIGH_IN_LOOP;
            teleOpStep = 0;
        }
    }
    private void pickFromSubstationDropInMedJunction() {
        if (teleOpStep == 0) {
            openClaw(true);
            //teleOpSleep(500);
            teleOpStep = 0.1;
        } else if (teleOpStep == 0.1) {
            moveTopHatPosition(.1, false, 4300 * armMultiplier, -2600 * elbowMultiplier, turntable.getCurrentPosition());
            teleOpStep = 1;
        } else if (teleOpStep == 1 && isInRange(-2600 * elbowMultiplier, elbow.getCurrentPosition())
                && isInRange(4300 * armMultiplier, arm.getCurrentPosition())) {
            moveTopHatPosition(subWristMedPickupPos, true, 4300 * armMultiplier, -2600 * elbowMultiplier, subTTMedPickupPos);
            teleOpStep = 2;
        } else if (teleOpStep == 2 && isInRange(subTTMedPickupPos, turntable.getCurrentPosition())) {
            moveTopHatPosition(subWristMedPickupPos, true, subArmMedPickupPos, subElbowMedPickupPos, subTTMedPickupPos);
            teleOpStep = 3;
        } else if (teleOpStep == 3 && isInRange(subElbowMedPickupPos, elbow.getCurrentPosition())
                && isInRange(subArmMedPickupPos, arm.getCurrentPosition())
                && isInRange(subTTMedPickupPos, turntable.getCurrentPosition())) {
            setWristPosition(subWristMedPickupPos);
            //teleOpSleep(1000);
            teleOpStep = 3.1;
        } else if (teleOpStep == 3.1 && isInRange(subElbowMedPickupPos, elbow.getCurrentPosition())
                && isInRange(subArmMedPickupPos, arm.getCurrentPosition())
                && isInRange(subTTMedPickupPos, turntable.getCurrentPosition())) {
            openClaw(false);
            teleOpSleep(500);
            teleOpStep = 3.2;
        } else if (teleOpStep == 3.2 && isInRange(subElbowMedPickupPos, elbow.getCurrentPosition())
                && isInRange(subArmMedPickupPos, arm.getCurrentPosition())
                && isInRange(subTTMedPickupPos, turntable.getCurrentPosition())) {
            setWristPosition(.1);
            teleOpSleep(500);
            teleOpStep = 3.3;
        } else if (teleOpStep == 3.3 && isInRange(subElbowMedPickupPos, elbow.getCurrentPosition())
                && isInRange(subArmMedPickupPos, arm.getCurrentPosition())
                && isInRange(subTTMedPickupPos, turntable.getCurrentPosition())) {
            teleOpStep = 4;
            moveTopHatPosition(.1, false, 4300 * armMultiplier, -2600 * elbowMultiplier, subTTMedPickupPos);
        } else if (teleOpStep == 4 && isInRange(-2600 * elbowMultiplier, elbow.getCurrentPosition())
                && isInRange(4300 * armMultiplier, arm.getCurrentPosition())) {
            moveTopHatPosition(.1, false, 4300 * armMultiplier, -2600 * elbowMultiplier, subTTMedDropPos);
            teleOpStep = 5;
        } else if (teleOpStep == 5 && isInRange(subTTMedDropPos, turntable.getCurrentPosition())) {
            moveTopHatPosition(subWristMedDropPos, false, subArmMedDropPos, subElbowMedDropPos, subTTMedDropPos);
            teleOpStep = 6;
        } else if (teleOpStep == 6 && isInRange(subElbowMedDropPos, elbow.getCurrentPosition())
                && isInRange(subArmMedDropPos, arm.getCurrentPosition())
                && isInRange(subTTMedDropPos, turntable.getCurrentPosition())) {
            openClaw(true);
            teleOpSleep(500);
            teleOpStep=6.1;
        } else if (teleOpStep == 6.1 && isInRange(subElbowMedDropPos, elbow.getCurrentPosition())
                && isInRange(subArmMedDropPos, arm.getCurrentPosition())
                && isInRange(subTTMedDropPos, turntable.getCurrentPosition())) {
            tophatAction = ATRobotEnumeration.PICK_CONE_DROP_MEDIUM_IN_LOOP;
            teleOpStep = 0;
        }
    }
    public boolean isTopHatMoveCompleted(double desiredArmPos , double desiredElbowPos , double desiredTurnTablePos){
        if (isInRange(desiredElbowPos, elbow.getCurrentPosition())
                && isInRange(desiredArmPos, arm.getCurrentPosition())
                && isInRange(desiredTurnTablePos, turntable.getCurrentPosition())){
            return true;
        }
        return false;
    }

/**
 *  ##### AUTON Only Methods from below and only called from Auton Modes
 *  ######################################################################################
 */

    /**
     * Following Method only called in Auton Mode
     */
    private final void autonSleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /**
     * Following Method only called in Auton Mode
     */
    public void redAllianceRightAutonHigh(){
        if (this.tophatAction == ATRobotEnumeration.AUTO_RED_RIGHT_HIGH_SETUP) {
            moveTopHatPosition(-1, false, 4300*armMultiplier, -20*elbowMultiplier, 200*turnTableMultiplier);
            noOfCones=0;
        }
        if (this.tophatAction == ATRobotEnumeration.SET_RED_RIGHT_PRELOADED_CONE) {
            //This is for medium drop with preloaded
            //moveTopHatPosition(.51, false, 4240, -1539, 1651);
            moveTopHatPosition(.55, false, 4161*armMultiplier, -1134*elbowMultiplier, 401*turnTableMultiplier);
            noOfCones=0;
        }
        if (this.tophatAction == ATRobotEnumeration.DROP_RED_RIGHT_PRELOADED_CONE) {
            openClaw(true);
            noOfCones=0;
        }
        // 13th Nov 2022
        if (this.tophatAction == ATRobotEnumeration.AUTO_RED_RIGHT_HIGH_PICK_CONE) {
            if (step == 0) {
                if (noOfCones == 0) {
                    desiredWristPosition=.37;
                    desiredElbowPosition = -5620*elbowMultiplier;
                    desiredArmPosition = 1817*armMultiplier;
                    desiredTurnTablePosition=209*turnTableMultiplier;
                } else if (noOfCones == 1) {
                    desiredWristPosition=.34;
                    desiredElbowPosition = -5430*elbowMultiplier;
                    desiredArmPosition = 1576*armMultiplier;
                    desiredTurnTablePosition=211*turnTableMultiplier;
                } /*else if (noOfCones == 2) {
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
                }*/
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
                autonSleep(500);
                openClaw(false);
                autonSleep(800);
                setWristPosition(.10);
                autonSleep(500);
                moveTopHatPosition(0.1, false, 4200*armMultiplier, -3500*elbowMultiplier, desiredTurnTablePosition);
                step = 2;
            }
            else if (step == 2 && isInRange(-3500*elbowMultiplier, elbow.getCurrentPosition())
                    && isInRange(4200*armMultiplier, arm.getCurrentPosition()) && isInRange(desiredTurnTablePosition, turntable.getCurrentPosition())) {
                moveTopHatPosition(0.1, false, 4200*armMultiplier, -3500*elbowMultiplier, 1548*turnTableMultiplier);
                step = 3;
            }

            else if (step == 3 && isInRange(-3500*elbowMultiplier, elbow.getCurrentPosition())
                    && isInRange(4200*armMultiplier, arm.getCurrentPosition()) && isInRange(1548*turnTableMultiplier, turntable.getCurrentPosition())) {
                moveTopHatPosition(.64, false, 4257*armMultiplier, -4995*elbowMultiplier, 1548*turnTableMultiplier);
                step = 4;
            }
            else if (step == 4 && isInRange(-4995*elbowMultiplier, elbow.getCurrentPosition())
                    && isInRange(4257*armMultiplier, arm.getCurrentPosition())
                    && isInRange(1548*turnTableMultiplier, turntable.getCurrentPosition())) {
                //setWristPosition(.64);
                //sleep(500);
                openClaw(true);
                autonSleep(500);
                step = 5;
                moveTopHatPosition(0.1, false, 4200*armMultiplier, -3500*elbowMultiplier, 1548*turnTableMultiplier);
            }
            else if (step == 5 && isInRange(-3500*elbowMultiplier, elbow.getCurrentPosition())
                    && isInRange(4200*armMultiplier, arm.getCurrentPosition())
                    && isInRange(1548*turnTableMultiplier, turntable.getCurrentPosition())) {
                moveTopHatPosition(0.1, false, 4200*armMultiplier, -3500*elbowMultiplier, desiredTurnTablePosition);
                step = 6;
            }
            else if (step == 6 && isInRange(-3500*elbowMultiplier, elbow.getCurrentPosition())
                    && isInRange(4200*armMultiplier, arm.getCurrentPosition())
                    && isInRange(desiredTurnTablePosition, turntable.getCurrentPosition())) {
                noOfCones = noOfCones + 1;
                step = 0;
            }

            if (!(noOfCones<desiredNoOfConesToPick)){
                tophatAction = ATRobotEnumeration.AUTO_RED_RIGHT_HIGH_PARK;
                //moveTopHatPosition(-1, false, 4224, -200, desiredTurnTablePosition);
                //***************** This is important to set the position in navigation mode
                moveTopHatPosition(-1, false, 4250*armMultiplier, -1800*elbowMultiplier, 945*turnTableMultiplier);
            }
        }
    }


    /**
     * Following Method only called in Auton Mode
     */
    public void redAllianceRightAutonMedium(){
        if (this.tophatAction == ATRobotEnumeration.AUTO_RED_RIGHT_MEDIUM_SETUP) {
            moveTopHatPosition(-1, false, 4300*armMultiplier, -20*elbowMultiplier, 200*turnTableMultiplier);
            noOfCones=0;
        }
        if (this.tophatAction == ATRobotEnumeration.AUTO_RED_RIGHT_MEDIUM_PICK_CONE) {
            if (step == 0) {
                if (noOfCones == 0) {
                    desiredElbowPosition = -6241*elbowMultiplier;
                    desiredArmPosition = 2189*armMultiplier;
                    desiredTurnTablePosition=1800*turnTableMultiplier;
                } else if (noOfCones == 1) {
                    desiredElbowPosition = -6160*elbowMultiplier;
                    desiredArmPosition = 2040*armMultiplier;
                    desiredTurnTablePosition=1800*turnTableMultiplier;
                } else if (noOfCones == 2) {
                    desiredElbowPosition = -6057*elbowMultiplier;
                    desiredArmPosition = 1866*armMultiplier;
                    desiredTurnTablePosition=1800*turnTableMultiplier;
                } else if (noOfCones == 3) {
                    desiredElbowPosition = -5801*elbowMultiplier;
                    desiredArmPosition = 1544*armMultiplier;
                    desiredTurnTablePosition=1800*turnTableMultiplier;
                } else if (noOfCones == 4) {
                    desiredElbowPosition = -5779*elbowMultiplier;
                    desiredArmPosition = 1483*armMultiplier;
                    desiredTurnTablePosition=1800*turnTableMultiplier;
                }
                moveTopHatPosition(0, true, desiredArmPosition, desiredElbowPosition, desiredTurnTablePosition);
                //openClaw(true);
                step = 1;
            } else if (step == 1 && isInRange(desiredElbowPosition, elbow.getCurrentPosition())
                    && isInRange(desiredArmPosition, arm.getCurrentPosition())
                    && isInRange(desiredTurnTablePosition, turntable.getCurrentPosition())) {
                telemetry.addData(" LOWER WRIST", "Pressed");
                setWristPosition(.3);
                autonSleep(500);
                openClaw(false);
                autonSleep(800);
                setWristPosition(.10);
                autonSleep(500);

                moveTopHatPosition(0, false, 4242*armMultiplier, -5895*elbowMultiplier, 533*turnTableMultiplier);
                step = 2;

            } else if (step == 2 && isInRange(-5895*elbowMultiplier, elbow.getCurrentPosition())
                    && isInRange(4242*armMultiplier, arm.getCurrentPosition())
                    && isInRange(533*turnTableMultiplier, turntable.getCurrentPosition())) {
                setWristPosition(.45);
                autonSleep(500);
                openClaw(true);
                //sleep(500);
                moveTopHatPosition(0, false, 4500*armMultiplier, -6163*elbowMultiplier, 900*turnTableMultiplier);
                step = 3;
            }else if (step == 3 && isInRange(-6163*elbowMultiplier, elbow.getCurrentPosition())
                    && isInRange(4500*armMultiplier, arm.getCurrentPosition())
                    && isInRange(900*turnTableMultiplier, turntable.getCurrentPosition())) {
                noOfCones = noOfCones + 1;
                step = 0;
            }
            if (!(noOfCones<desiredNoOfConesToPick)){
                tophatAction = ATRobotEnumeration.AUTO_RED_RIGHT_MEDIUM_PARK;
                moveTopHatPosition(-1, false, 4224*armMultiplier, -200*elbowMultiplier, desiredTurnTablePosition);
            }
        }
        telemetry.addData("wrist position", wrist.getPosition());
        telemetry.addData("wrist position while moving", wrist.getController().getServoPosition(0));
        telemetry.addData("Right Claw position", rightClaw.getPosition());
        telemetry.addData("Left Claw position", leftClaw.getPosition());
        telemetry.addData("arm position", arm.getCurrentPosition());
        telemetry.addData("elbow position", elbow.getCurrentPosition());
        telemetry.addData("turntable position", turntable.getCurrentPosition());
        telemetry.addData("noOfCones", noOfCones);
    }
    /**
     * Following Method only called in Auton Mode
     */

    public void blueAllianceLeftAutonHigh(){
        if (this.tophatAction == ATRobotEnumeration.AUTO_BLUE_LEFT_HIGH_SETUP) {
            moveTopHatPosition(.9, false, robotMidPointArmPosition, -20*elbowMultiplier, 1600*turnTableMultiplier);
            noOfCones=0;
        }
        if (this.tophatAction == ATRobotEnumeration.SET_BLUE_LEFT_PRELOADED_CONE) {
            //This is for medium drop with preloaded
            moveTopHatPosition(.55, false, 4300*armMultiplier, -1366*elbowMultiplier, 1548*turnTableMultiplier);
            //moveTopHatPosition(.54, false, 4305, -2153, 1902);
            noOfCones=0;
        }
        if (this.tophatAction == ATRobotEnumeration.DROP_BLUE_LEFT_PRELOADED_CONE) {
            openClaw(true);
            noOfCones=0;
        }
        if (this.tophatAction == ATRobotEnumeration.AUTO_BLUE_LEFT_HIGH_PICK_CONE) {
            if (step == 0) {
                if (noOfCones == 0) {
                    desiredWristPosition=.41;
                    desiredElbowPosition = -4637*elbowMultiplier;
                    desiredArmPosition = 1216*armMultiplier;
                    desiredTurnTablePosition=1780*turnTableMultiplier;
                } else if (noOfCones == 1) {
                    desiredWristPosition=.41;
                    desiredElbowPosition = -4765*elbowMultiplier;
                    desiredArmPosition = 1218*armMultiplier;
                    desiredTurnTablePosition=1780*turnTableMultiplier;

                }/*else if (noOfCones == 2) {
                    desiredWristPosition=.46;
                    desiredElbowPosition = -4173;
                    desiredArmPosition = 1338;
                    desiredTurnTablePosition=1786;
                } else if (noOfCones == 3) {
                    desiredWristPosition=.44;
                    desiredElbowPosition = -4520;
                    desiredArmPosition = 1466;
                    desiredTurnTablePosition=1786;
                } else if (noOfCones == 4) {
                    desiredWristPosition=.48;
                    desiredElbowPosition = -4520;
                    desiredArmPosition = 1478;
                    desiredTurnTablePosition=1786;
                }*/
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
                autonSleep(500);
                openClaw(false);
                autonSleep(800);
                setWristPosition(.10);
                autonSleep(500);
                moveTopHatPosition(0.1, false, robotMidPointArmPosition, robotMidPointElbowPosition, desiredTurnTablePosition);
                step = 2;
            }
            else if (step == 2 && isInRange(robotMidPointElbowPosition, elbow.getCurrentPosition())
                    && isInRange(robotMidPointArmPosition, arm.getCurrentPosition())) {
                moveTopHatPosition(0.1, false, robotMidPointArmPosition, robotMidPointElbowPosition, 508*turnTableMultiplier);
                step = 3;
            }

            else if (step == 3 && isInRange(508*turnTableMultiplier, turntable.getCurrentPosition())) {
                moveTopHatPosition(.55, false, 4300*armMultiplier, -5298*elbowMultiplier, 508*turnTableMultiplier);
                step = 4;
            }
            else if (step == 4 && isInRange(-5298*elbowMultiplier, elbow.getCurrentPosition())
                    && isInRange(4300*armMultiplier, arm.getCurrentPosition())
                    && isInRange(508*turnTableMultiplier, turntable.getCurrentPosition())) {
                openClaw(true);
                autonSleep(500);
                step = 5;
                moveTopHatPosition(0.1, false, robotMidPointArmPosition, robotMidPointElbowPosition, 508*turnTableMultiplier);
            }
            else if (step == 5 && isInRange(robotMidPointElbowPosition, elbow.getCurrentPosition())
                    && isInRange(robotMidPointArmPosition, arm.getCurrentPosition())) {
                moveTopHatPosition(0.1, false, robotMidPointArmPosition, robotMidPointElbowPosition, desiredTurnTablePosition);
                step = 6;
            }
            else if (step == 6 && isInRange(robotMidPointElbowPosition, elbow.getCurrentPosition())
                    && isInRange(robotMidPointArmPosition, arm.getCurrentPosition())
                    && isInRange(desiredTurnTablePosition, turntable.getCurrentPosition())) {
                noOfCones = noOfCones + 1;
                step = 0;
            }
            if (!(noOfCones<desiredNoOfConesToPick)){
                tophatAction = ATRobotEnumeration.AUTO_BLUE_LEFT_HIGH_PARK;
                //moveTopHatPosition(-1, false, 4224, -200, desiredTurnTablePosition);
                //***************** This is important to set the position in navigation mode
                moveTopHatPosition(.9, false, 4250*armMultiplier, -1800*elbowMultiplier, 945*turnTableMultiplier);

            }
        }

    }
    /**
     * Following Method only called in Auton Mode
     */
    public void blueAllianceRightAutonHigh(){
        if (this.tophatAction == ATRobotEnumeration.AUTO_BLUE_RIGHT_HIGH_SETUP) {
            moveTopHatPosition(-1, false, robotMidPointArmPosition, -20*elbowMultiplier, 1600*turnTableMultiplier);
            noOfCones=0;
        }
        if (this.tophatAction == ATRobotEnumeration.SET_BLUE_RIGHT_PRELOADED_CONE) {
            //This is for medium drop with preloaded
            //moveTopHatPosition(.51, false, 4240, -1539, 1651);
            moveTopHatPosition(.56, false, 4251*armMultiplier, -1130*elbowMultiplier, 361*turnTableMultiplier);
            noOfCones=0;
        }
        if (this.tophatAction == ATRobotEnumeration.DROP_BLUE_RIGHT_PRELOADED_CONE) {
            openClaw(true);
            noOfCones=0;
        }
        if (this.tophatAction == ATRobotEnumeration.AUTO_BLUE_RIGHT_HIGH_PICK_CONE) {
            if (step == 0) {
                if (noOfCones == 0) {
                    desiredWristPosition=.39;
                    desiredElbowPosition = -5279*elbowMultiplier;
                    desiredArmPosition = 1609*armMultiplier;
                    desiredTurnTablePosition=254*turnTableMultiplier;
                } else if (noOfCones == 1) {
                    desiredWristPosition=.48;
                    desiredElbowPosition = -3906*elbowMultiplier;
                    desiredArmPosition = 647*armMultiplier;
                    desiredTurnTablePosition=250*turnTableMultiplier;
                } /*else if (noOfCones == 2) {
                    desiredWristPosition=.46;
                    desiredElbowPosition = -4173;
                    desiredArmPosition = 1338;
                    desiredTurnTablePosition=1786;
                } else if (noOfCones == 3) {
                    desiredWristPosition=.44;
                    desiredElbowPosition = -4520;
                    desiredArmPosition = 1466;
                    desiredTurnTablePosition=1786;
                } else if (noOfCones == 4) {
                    desiredWristPosition=.48;
                    desiredElbowPosition = -4520;
                    desiredArmPosition = 1478;
                    desiredTurnTablePosition=1786;
                }*/
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
                autonSleep(500);
                openClaw(false);
                autonSleep(800);
                setWristPosition(.10);
                autonSleep(500);
                moveTopHatPosition(0.1, false, robotMidPointArmPosition, robotMidPointElbowPosition, desiredTurnTablePosition);
                step = 2;
            }
            else if (step == 2 && isInRange(robotMidPointElbowPosition, elbow.getCurrentPosition())
                    && isInRange(robotMidPointArmPosition, arm.getCurrentPosition())) {
                moveTopHatPosition(0.1, false, robotMidPointArmPosition, robotMidPointElbowPosition, 1529*turnTableMultiplier);
                step = 3;
            }

            else if (step == 3 && isInRange(1529*turnTableMultiplier, turntable.getCurrentPosition())) {
                moveTopHatPosition(.59, false, 4116*armMultiplier, -4917*elbowMultiplier, 1529*turnTableMultiplier);
                step = 4;
            }
            else if (step == 4 && isInRange(-4917*elbowMultiplier, elbow.getCurrentPosition())
                    && isInRange(4116*armMultiplier, arm.getCurrentPosition())
                    && isInRange(1529*turnTableMultiplier, turntable.getCurrentPosition())) {
                //setWristPosition(.64);
                //sleep(500);
                openClaw(true);
                autonSleep(500);
                step = 5;
                moveTopHatPosition(0.1, false, robotMidPointArmPosition, robotMidPointElbowPosition, 1529*turnTableMultiplier);
            }
            else if (step == 5 && isInRange(robotMidPointElbowPosition, elbow.getCurrentPosition())
                    && isInRange(robotMidPointArmPosition, arm.getCurrentPosition())) {
                moveTopHatPosition(0.1, false, robotMidPointArmPosition, robotMidPointElbowPosition, desiredTurnTablePosition);
                step = 6;
            }
            else if (step == 6 && isInRange(robotMidPointElbowPosition, elbow.getCurrentPosition())
                    && isInRange(robotMidPointArmPosition, arm.getCurrentPosition())
                    && isInRange(desiredTurnTablePosition, turntable.getCurrentPosition())) {
                noOfCones = noOfCones + 1;
                step = 0;
            }
            if (!(noOfCones<desiredNoOfConesToPick)){
                tophatAction = ATRobotEnumeration.AUTO_BLUE_RIGHT_HIGH_PARK;
                //moveTopHatPosition(-1, false, 4224, -200, desiredTurnTablePosition);
                //***************** This is important to set the position in navigation mode
                moveTopHatPosition(-1, false, 4250*armMultiplier, -1800*elbowMultiplier, 945*turnTableMultiplier);

            }
        }

    }

    /**
     Following Methods Only called from Auton. These may be moved in to a separate class but for now everything is in one place for ease of use

     */

    public void redAllianceLeftAutonHigh(){
        if (this.tophatAction == ATRobotEnumeration.AUTO_RED_LEFT_HIGH_SETUP) {
            moveTopHatPosition(-1, false, robotMidPointArmPosition, -20*elbowMultiplier, 1600*turnTableMultiplier);
            noOfCones=0;
        }
        if (this.tophatAction == ATRobotEnumeration.SET_RED_LEFT_PRELOADED_CONE) {
            //This is for medium drop with preloaded
            moveTopHatPosition(.58, false, 4300*armMultiplier, -1219*elbowMultiplier, 1475*turnTableMultiplier);

            //This is for high drop with preloaded
            //moveTopHatPosition(.53, false, 4300, -2152, 1900);
            noOfCones=0;
        }
        if (this.tophatAction == ATRobotEnumeration.DROP_RED_LEFT_PRELOADED_CONE) {
            openClaw(true);
            noOfCones=0;
        }
        if (this.tophatAction == ATRobotEnumeration.AUTO_RED_LEFT_HIGH_PICK_CONE) {
            if (step == 0) {
                if (noOfCones == 0) {
                    desiredWristPosition=.44;
                    desiredElbowPosition = -4176*elbowMultiplier;
                    desiredArmPosition = 937*armMultiplier;
                    desiredTurnTablePosition=1794*turnTableMultiplier;
                } else if (noOfCones == 1) {
                    desiredWristPosition=.46;
                    desiredElbowPosition = -4211*elbowMultiplier;
                    desiredArmPosition = 909*armMultiplier;
                    desiredTurnTablePosition=1805*turnTableMultiplier;
                } /*else if (noOfCones == 2) {
                    desiredWristPosition=.46;
                    desiredElbowPosition = -4173;
                    desiredArmPosition = 1338;
                    desiredTurnTablePosition=1786;
                } else if (noOfCones == 3) {
                    desiredWristPosition=.44;
                    desiredElbowPosition = -4520;
                    desiredArmPosition = 1466;
                    desiredTurnTablePosition=1786;
                } else if (noOfCones == 4) {
                    desiredWristPosition=.48;
                    desiredElbowPosition = -4520;
                    desiredArmPosition = 1478;
                    desiredTurnTablePosition=1786;
                }*/
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
                autonSleep(500);
                openClaw(false);
                autonSleep(800);
                setWristPosition(.10);
                autonSleep(500);
                moveTopHatPosition(0.1, false, robotMidPointArmPosition, robotMidPointElbowPosition, desiredTurnTablePosition);
                step = 2;
            }
            else if (step == 2 && isInRange(robotMidPointElbowPosition, elbow.getCurrentPosition())
                    && isInRange(robotMidPointArmPosition, arm.getCurrentPosition())) {
                moveTopHatPosition(0.1, false, robotMidPointArmPosition, robotMidPointElbowPosition, 494*turnTableMultiplier);
                step = 3;
            }

            else if (step == 3 && isInRange(494*turnTableMultiplier, turntable.getCurrentPosition())) {
                moveTopHatPosition(.62, false, 4300*armMultiplier, -5024*elbowMultiplier, 494*turnTableMultiplier);
                step = 4;
            }
            else if (step == 4 && isInRange(-5024*elbowMultiplier, elbow.getCurrentPosition())
                    && isInRange(4300*armMultiplier, arm.getCurrentPosition())
                    && isInRange(494*turnTableMultiplier, turntable.getCurrentPosition())) {
                openClaw(true);
                autonSleep(500);
                step = 5;
                moveTopHatPosition(0.1, false, robotMidPointArmPosition, robotMidPointElbowPosition, 494*turnTableMultiplier);
            }
            else if (step == 5 && isInRange(robotMidPointElbowPosition, elbow.getCurrentPosition())
                    && isInRange(robotMidPointArmPosition, arm.getCurrentPosition())) {
                moveTopHatPosition(0.1, false, robotMidPointArmPosition, robotMidPointElbowPosition, desiredTurnTablePosition);
                step = 6;
            }
            else if (step == 6 && isInRange(robotMidPointElbowPosition, elbow.getCurrentPosition())
                    && isInRange(robotMidPointArmPosition, arm.getCurrentPosition())
                    && isInRange(desiredTurnTablePosition, turntable.getCurrentPosition())) {
                noOfCones = noOfCones + 1;
                step = 0;
            }
            if (!(noOfCones<desiredNoOfConesToPick)){
                tophatAction = ATRobotEnumeration.AUTO_RED_LEFT_HIGH_PARK;
                //moveTopHatPosition(-1, false, 4224, -200, desiredTurnTablePosition);
                //***************** This is important to set the position in navigation mode
                moveTopHatPosition(-1, false, 4250*armMultiplier, -1800*elbowMultiplier, 945*turnTableMultiplier);
            }
        }

    }
    /**
     * Following Method only called in Auton Mode
     */
    public boolean areFiveConesDone(){
        if (!(noOfCones<desiredNoOfConesToPick)) {
            telemetry.addData("No Of COnes", noOfCones);
            return true;
        }
        else
        {
            telemetry.addData("No Of COnes", noOfCones);
            return false;
        }
    }

}