package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TopHatAutoControllerStatesV2 {

    private DcMotorEx arm;
    private DcMotorEx elbow;
    private DcMotorEx turntable;
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
    public double parkingTurnTablePosition=250;


    int desiredNoOfConesToPick=2;
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
    public double teleOpStep=0;
    int noOfCones = 0 ;
    MecanumDriveAT drive;
    public ATRobotEnumeration tophatAction;

    ATRobotEnumeration tophatMode;
    ATRobotEnumeration substationHSide;
    ATRobotEnumeration substationMSide;
    public ATRobotEnumeration substationPickConfig;
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

    double subTTMedPickupPos_Loop=1705;
    double subArmMedPickupPos_Loop=755;
    double subElbowMedPickupPos_Loop=-1590;
    double subWristMedPickupPos_Loop=.3583;

    double subTTMedDropPos_Loop=450;
    double subArmMedDropPos_Loop=1030;
    double subElbowMedDropPos_Loop=-604;
    double subWristMedDropPos_Loop=.81;

    double subTTHighPickupPos_Loop=188;
    double subArmHighPickupPos_Loop=833;
    double subElbowHighPickupPos_Loop=-1666;
    double subWristHighPickupPos_Loop=.3583;

    double subTTHighDropPos_Loop=1371;
    double subArmHighDropPos_Loop=2203;
    double subElbowHighDropPos_Loop=-1259;
    double subWristHighDropPos_Loop=.68;

    public double frontPickTTPos=884;
    public double leftDiagPickTTPos=1225;
    public double rightDiagPickTTPos=525;
    public double leftPickTTPos;
    public double rightPickTTPos;

    public double highJunctionArmPos=2205;
    public double highJunctionElbowPos=-1017;
    public double highJunctionWristPos=.855;

    public double mediumJunctionArmPos=2018*armMultiplier;
    public double mediumJunctionElbowPos=-1888*elbowMultiplier;
    public double mediumJunctionWristPos=.87;

    public double lowJunctionArmPos=391*armMultiplier;
    public double lowJunctionElbowPos=-1080*elbowMultiplier;
    public double lowJunctionWristPos=.85;

    public double groundJunctionArmPos=857;
    public double groundJunctionElbowPos=-1806;
    public double groundJunctionWristPos=.235;

    public ATRobotEnumeration topHatSpeed = ATRobotEnumeration.TOPHAT_MEDIUM_SPEED;
    public ATRobotEnumeration selectedTopHatSpeed = ATRobotEnumeration.TOPHAT_MEDIUM_SPEED;
    public ATRobotEnumeration robotMode=ATRobotEnumeration.TELE_OP_AUTO;


    public void fullyInitializeRobot(Telemetry tl, Gamepad gp1, Gamepad gp2, ATRobotEnumeration rMode, HardwareMap hardwareMapAT) {
        tophatAction =rMode;
        telemetry = tl;
        gamepad1=gp1;
        gamepad2=gp2;
        arm = hardwareMapAT.get(DcMotorEx.class, "arm");
        elbow = hardwareMapAT.get(DcMotorEx.class, "elbow");
        turntable = hardwareMapAT.get(DcMotorEx.class, "turntable");
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
        ResetTopHat();
      }
    public int getDesiredNoOfConesToPick() {
        return desiredNoOfConesToPick;
    }

    public void setDesiredNoOfConesToPick(int desiredNoOfConesToPick) {
        this.desiredNoOfConesToPick = desiredNoOfConesToPick;
    }

    public void basicInitializeRobot(HardwareMap hardwareMapAT, Telemetry tl, Gamepad gp1, Gamepad gp2, ATRobotEnumeration rMode) {
        tophatAction =rMode;
        telemetry = tl;
        gamepad1=gp1;
        gamepad2=gp2;
        arm = hardwareMapAT.get(DcMotorEx.class, "arm");
        elbow = hardwareMapAT.get(DcMotorEx.class, "elbow");
        turntable = hardwareMapAT.get(DcMotorEx.class, "turntable");
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
        //setTopHatMotorsVelocity(2000, 2000,1000);
        setTopHatSpeed(ATRobotEnumeration.TOPHAT_MEDIUM_SPEED);

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

                subTTMedPickupPos=108;
                subArmMedPickupPos=771;
                subElbowMedPickupPos=-1603;
                subWristMedPickupPos=.34;

                subTTMedDropPos=1416;
                subArmMedDropPos=945;
                subElbowMedDropPos=-508;
                subWristMedDropPos=.91;


                substationHSide=ATRobotEnumeration.SUBSTATION_LEFT;
                substationMSide=ATRobotEnumeration.SUBSTATION_RIGHT;

                /**
                 * Following needs to be tuned well for specific substation and this is very generic
                 */
                subTTMedPickupPos_Loop=188; //
                subArmMedPickupPos_Loop=755;
                subElbowMedPickupPos_Loop=-1590;
                subWristMedPickupPos_Loop=.2988;

                subTTMedDropPos_Loop=1371; //
                subArmMedDropPos_Loop=1030;
                subElbowMedDropPos_Loop=-604;
                subWristMedDropPos_Loop=.81;

                subTTHighPickupPos_Loop=1705;//
                subArmHighPickupPos_Loop=833;
                subElbowHighPickupPos_Loop=-1666;
                subWristHighPickupPos_Loop=.26;

                subTTHighDropPos_Loop=450; //
                subArmHighDropPos_Loop=2203;
                subElbowHighDropPos_Loop=-1259;
                subWristHighDropPos_Loop=.68;
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

                subTTMedPickupPos=1710;
                subArmMedPickupPos=790;
                subElbowMedPickupPos=-1606;
                subWristMedPickupPos=.38;

                subTTMedDropPos=442;
                subArmMedDropPos=912;
                subElbowMedDropPos=-517;
                subWristMedDropPos=.91;
                substationHSide=ATRobotEnumeration.SUBSTATION_RIGHT;
                substationMSide=ATRobotEnumeration.SUBSTATION_LEFT;

                subTTMedPickupPos_Loop=1705;
                subArmMedPickupPos_Loop=755;
                subElbowMedPickupPos_Loop=-1590;
                subWristMedPickupPos_Loop=.2988;

                subTTMedDropPos_Loop=450;
                subArmMedDropPos_Loop=1030;
                subElbowMedDropPos_Loop=-604;
                subWristMedDropPos_Loop=.81;

                subTTHighPickupPos_Loop=188;
                subArmHighPickupPos_Loop=833;
                subElbowHighPickupPos_Loop=-1666;
                subWristHighPickupPos_Loop=.26;

                subTTHighDropPos_Loop=1371;
                subArmHighDropPos_Loop=2203;
                subElbowHighDropPos_Loop=-1259;
                subWristHighDropPos_Loop=.68;


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

                subTTMedPickupPos=108;
                subArmMedPickupPos=771;
                subElbowMedPickupPos=-1603;
                subWristMedPickupPos=.34;

                subTTMedDropPos=1416;
                subArmMedDropPos=945;
                subElbowMedDropPos=-508;
                subWristMedDropPos=.91;

                substationHSide=ATRobotEnumeration.SUBSTATION_LEFT;
                substationMSide=ATRobotEnumeration.SUBSTATION_RIGHT;

                /**
                 * Following needs to be tuned well for specific substation and this is very generic
                 */
                subTTMedPickupPos_Loop=188; //
                subArmMedPickupPos_Loop=755;
                subElbowMedPickupPos_Loop=-1590;
                subWristMedPickupPos_Loop=.2988;

                subTTMedDropPos_Loop=1371; //
                subArmMedDropPos_Loop=1030;
                subElbowMedDropPos_Loop=-604;
                subWristMedDropPos_Loop=.81;

                subTTHighPickupPos_Loop=1705;//
                subArmHighPickupPos_Loop=833;
                subElbowHighPickupPos_Loop=-1666;
                subWristHighPickupPos_Loop=.26;

                subTTHighDropPos_Loop=450; //
                subArmHighDropPos_Loop=2203;
                subElbowHighDropPos_Loop=-1259;
                subWristHighDropPos_Loop=.68;

            }
            break;
            case BLUE_LEFT_HIGH_DROP: {

                subTTMedPickupPos_Loop=1705;
                subArmMedPickupPos_Loop=755;
                subElbowMedPickupPos_Loop=-1590;
                subWristMedPickupPos_Loop=.2988;

                subTTMedDropPos_Loop=450;
                subArmMedDropPos_Loop=1030;
                subElbowMedDropPos_Loop=-604;
                subWristMedDropPos_Loop=.81;

                subTTHighPickupPos_Loop=188;
                subArmHighPickupPos_Loop=833;
                subElbowHighPickupPos_Loop=-1666;
                subWristHighPickupPos_Loop=.26;

                subTTHighDropPos_Loop=1371;
                subArmHighDropPos_Loop=2203;
                subElbowHighDropPos_Loop=-1259;
                subWristHighDropPos_Loop=.68;


                subTTHighPickupPos =281*turnTableMultiplier;
                subArmHighPickupPos =1155*armMultiplier;
                subElbowHighPickupPos =-5345*elbowMultiplier;
                subWristHighPickupPos =.3383;

                subTTHighDropPos=1550*turnTableMultiplier;
                subArmHighDropPos =3516*armMultiplier;
                subElbowHighDropPos =-3763*elbowMultiplier;
                subWristHighDropPos =.7155;

                subTTMedPickupPos=1710;
                subArmMedPickupPos=790;
                subElbowMedPickupPos=-1606;
                subWristMedPickupPos=.38;

                subTTMedDropPos=442;
                subArmMedDropPos=912;
                subElbowMedDropPos=-517;
                subWristMedDropPos=.91;

                substationHSide=ATRobotEnumeration.SUBSTATION_RIGHT;
                substationMSide=ATRobotEnumeration.SUBSTATION_LEFT;
            }
            break;
        }

        telemetry.addData("wrist position", wrist.getPosition());
        telemetry.addData("arm position", arm.getCurrentPosition());
        telemetry.addData("elbow position", elbow.getCurrentPosition());
        telemetry.addData("turntable position", turntable.getCurrentPosition());
        telemetry.addData("rightClaw position", rightClaw.getPosition());
        telemetry.addData("leftClaw position", leftClaw.getPosition());
        telemetry.addData("testCounter", testCounter);
    }

    public void ResetTopHat(){
        setTopHatSpeed(ATRobotEnumeration.TOPHAT_LOW_SPEED);
        tophatAction = ATRobotEnumeration.RESET;
        tophatMode = ATRobotEnumeration.AUTO;
        ResetWristNClaw();
        ResetArmUp();
        ResetArmElbow();
        ResetTurnTable();
        ResetArmDown();
        setTopHatSpeed(ATRobotEnumeration.TOPHAT_MEDIUM_SPEED);
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
        //tophatDriverActionsNovi();
        this.tophatDriverActionsStateChamp();
        if (tophatAction == ATRobotEnumeration.MANUAL) {
            tophatMode = ATRobotEnumeration.MANUAL;
            sleepMode = ATRobotEnumeration.SLEEP_MODE_OFF;
            resetSelectedTopHatSpeed();
            teleOpStep=0;
        }
        if (sleepMode !=ATRobotEnumeration.SLEEP_MODE_ON) {
            if (tophatAction == ATRobotEnumeration.AUTO_TOPHAT_ONEWAY_MOVE_BEGIN) {
                tophatMode = ATRobotEnumeration.FULL_AUTO;
                moveTopHatOneWayInOrder(); //reset enabled
            }
            if (tophatAction == ATRobotEnumeration.AUTO_TOPHAT_GROUND_MOVE_BEGIN) {
                //This is enabled with coordinated speed
                tophatMode = ATRobotEnumeration.FULL_AUTO;
                moveTopHatGroundPickup();
            }
            if (tophatAction == ATRobotEnumeration.PICK_CONE_READY_TO_NAVIGATE) {
                tophatMode = ATRobotEnumeration.FULL_AUTO;
                conePickupToNavigateStates(); //reset enabled
            }

            if (tophatAction == ATRobotEnumeration.PICK_CONE_DROP_HIGH_IN_LOOP) {
                tophatMode = ATRobotEnumeration.FULL_AUTO;
                //This is enabled with coordinated speed
                pickFromSubstationDropInHighJunctionStates();
            }

            if (tophatAction == ATRobotEnumeration.PICK_CONE_DROP_MEDIUM_IN_LOOP) {
                tophatMode = ATRobotEnumeration.FULL_AUTO;
                //This is enabled with coordinated speed
                pickFromSubstationDropInMedJunctionStates();

            }

            if (tophatAction == ATRobotEnumeration.PICK_CONE_SET_TO_DRPOP_HIGH) {
                tophatMode = ATRobotEnumeration.FULL_AUTO;
                pickNSetforHighDrop(); //reset enabled
            }

            if (tophatAction == ATRobotEnumeration.PICK_CONE_SET_TO_DRPOP_MEDIUM) {
                tophatMode = ATRobotEnumeration.FULL_AUTO;
                pickNSetforMediumDrop(); //reset enabled
            }
        }
        moveTopHatMotors();
        setMotorNServoMaximums();
        telemetry.addData("wrist position", wrist.getPosition());
        telemetry.addData("arm position", arm.getCurrentPosition());
        telemetry.addData("elbow position", elbow.getCurrentPosition());
        telemetry.addData("turntable position", turntable.getCurrentPosition());
        telemetry.addData("TopHat robot mode", tophatAction);
        telemetry.addData("TopHat Step # in Execution", teleOpStep);
        telemetry.addData("Existing TopHat Speed", topHatSpeed);
        telemetry.addData("Selected TopHat Speed", selectedTopHatSpeed);
        telemetry.addData("*** Elbow Speed ***", elbow.getVelocity());
        telemetry.addData("*** Arm Speed ***", arm.getVelocity());
        telemetry.addData("*** TurnTable Speed ***", turntable.getVelocity());

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
        WristPosition = Math.min(Math.max(WristPosition, .1), .9);

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
    private void setMotorPosition(int pos, DcMotorEx motor, int motorVel){
        motor.setTargetPosition(pos);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setVelocity(motorVel);
    }
    private void setWristPosition(double pos){
        if (tophatAction==ATRobotEnumeration.MANUAL || robotMode==ATRobotEnumeration.FULL_AUTON){
            WristPosition = pos;
        }
        else {
            WristPosition = pos * 1.06; // Compenstate change in wrist movement ######## CAUTION #####
        }
        wrist.setPosition(WristPosition);
    }
    private void openClaw(boolean status){
        if (status) {
            RightClawPosition = .5;
            LeftClawPosition=.5;
        }
        else{
            RightClawPosition = .19;
            LeftClawPosition=.81;
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
        resetSelectedTopHatSpeed();
        if (teleOpStep==0){
            openClaw(desiredClawPosition);
            teleOpSleep(500);
            teleOpStep = 0.1;
        }
        else if (teleOpStep==0.1){
            moveTopHatPosition(.3, desiredClawPosition, highJunctionArmPos, highJunctionElbowPos, turntable.getCurrentPosition());
            teleOpStep = 1;
        }
        else if (teleOpStep == 1 && isInRange(highJunctionElbowPos, elbow.getCurrentPosition())
                && isInRange(highJunctionArmPos, arm.getCurrentPosition())){
            moveTopHatPosition(.3,desiredClawPosition,highJunctionArmPos,highJunctionElbowPos,desiredTurnTablePosition);
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
            teleOpSleep(1000);
            teleOpStep=3.1;
        }
        else if (teleOpStep==3.1 && isInRange(desiredElbowPosition, elbow.getCurrentPosition())
                && isInRange(desiredArmPosition, arm.getCurrentPosition())
                && isInRange(desiredTurnTablePosition, turntable.getCurrentPosition()) ) {
            teleOpStep = 0;
            tophatAction = ATRobotEnumeration.AUTO_TOPHAT_ONEWAY_MOVE_END;
        }
    }

    private void moveTopHatGroundPickup(){
        if (teleOpStep==3){
            setTopHatSpeed(ATRobotEnumeration.TOPHAT_MEDIUM_SPEED);
        }
        else{
            resetSelectedTopHatSpeed();
        }

        if (teleOpStep==0){
            openClaw(desiredClawPosition);
            setWristPosition(0.2);
            teleOpSleep(1000);
            teleOpStep = 0.1;
        }
        else if (teleOpStep==0.1){
            moveTopHatPosition(0.2, desiredClawPosition, mediumJunctionArmPos, mediumJunctionElbowPos, turntable.getCurrentPosition());
            teleOpStep = 1;
        }
        else if (teleOpStep == 1 && isInRange(mediumJunctionElbowPos, elbow.getCurrentPosition())
                && isInRange(mediumJunctionArmPos, arm.getCurrentPosition())){
            moveTopHatPosition(0.2,desiredClawPosition,mediumJunctionArmPos,mediumJunctionElbowPos,desiredTurnTablePosition);
            teleOpStep=2;
        }
        else if (teleOpStep == 2 && isInRange(desiredTurnTablePosition, turntable.getCurrentPosition())){
            moveTopHatPosition(0.2,desiredClawPosition,desiredArmPosition,desiredElbowPosition,desiredTurnTablePosition);
            teleOpStep=3;
        }
        else if (teleOpStep==3 && isInRange(desiredElbowPosition, elbow.getCurrentPosition())
                && isInRange(desiredArmPosition, arm.getCurrentPosition())
                && isInRange(desiredTurnTablePosition, turntable.getCurrentPosition()) ) {
            setWristPosition(desiredWristPosition);
            teleOpSleep(1000);
            teleOpStep=3.1;
        }
        else if (teleOpStep==3.1 && isInRange(desiredElbowPosition, elbow.getCurrentPosition())
                && isInRange(desiredArmPosition, arm.getCurrentPosition())
                && isInRange(desiredTurnTablePosition, turntable.getCurrentPosition()) ) {
            teleOpStep = 0;
            tophatAction = ATRobotEnumeration.AUTO_TOPHAT_GROUND_MOVE_END;
        }
    }

    private void conePickupToNavigate(){
        resetSelectedTopHatSpeed();
        if (teleOpStep==0){
            openClaw(true);
            //teleOpSleep(500);
            teleOpStep = 0.1;
        }
        else if (teleOpStep==0.1){
            moveTopHatPosition(.1, false,highJunctionArmPos , highJunctionElbowPos, turntable.getCurrentPosition());
            teleOpStep = 1;
        }
        else if (teleOpStep == 1 && isInRange(highJunctionElbowPos, elbow.getCurrentPosition())
                && isInRange(highJunctionArmPos, arm.getCurrentPosition())){
            moveTopHatPosition(subWristHighPickupPos,true,highJunctionArmPos,highJunctionElbowPos, subTTHighPickupPos);
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
            teleOpSleep(1000);
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
            setWristPosition(0);
            teleOpSleep(1000);
            teleOpStep=3.3;
        }
        else if (teleOpStep==3.3 && isInRange(subElbowHighPickupPos, elbow.getCurrentPosition())
                && isInRange(subArmHighPickupPos, arm.getCurrentPosition())
                && isInRange(subTTHighPickupPos, turntable.getCurrentPosition()) ) {
            teleOpStep=4;
            moveTopHatPosition(.1,false,highJunctionArmPos,-1800*elbowMultiplier, subTTHighPickupPos);
        }
        else if (teleOpStep==4 && isInRange(-1800*elbowMultiplier, elbow.getCurrentPosition())
                && isInRange(highJunctionArmPos, arm.getCurrentPosition())){
            teleOpStep=5;
            moveTopHatPosition(.1,false,highJunctionArmPos,-1800*elbowMultiplier,frontPickTTPos);
        }
        else if (teleOpStep==5 && isInRange(frontPickTTPos, turntable.getCurrentPosition())){
            teleOpStep=0;
            tophatAction = ATRobotEnumeration.FULL_AUTO;
        }
    }

    private void conePickupToNavigateStates(){
        resetSelectedTopHatSpeed();
        if (teleOpStep==0){
            openClaw(false);
            teleOpSleep(500);
            teleOpStep=0.1;
        }
        else if (teleOpStep==0.1){
            setWristPosition(0);
            teleOpSleep(1000);
            teleOpStep=1;
        }
        else if (teleOpStep == 1){
            teleOpStep=0;
            tophatAction = ATRobotEnumeration.AUTO_TOPHAT_ONEWAY_MOVE_BEGIN;
            setTopHatPosition(.1,false, highJunctionArmPos, -1800*elbowMultiplier,frontPickTTPos);
        }
    }

    private void pickNSetforHighDrop(){
        resetSelectedTopHatSpeed();
         if (teleOpStep==0){
             openClaw(false);
             teleOpSleep(1000);
             teleOpStep=0.1;
        }
         else if (teleOpStep==0.1){
             setWristPosition(0);
             teleOpSleep(1000);
             teleOpStep=1;
         }
        else if (teleOpStep == 1){
            teleOpStep=0;
            tophatAction = ATRobotEnumeration.AUTO_TOPHAT_ONEWAY_MOVE_BEGIN;
            setTopHatPosition(subWristHighDropPos,false, subArmHighDropPos, subElbowHighDropPos,subTTHighDropPos);
        }
    }

    private void pickNSetforMediumDrop(){
        resetSelectedTopHatSpeed();
        if (teleOpStep==0){
            openClaw(false);
            teleOpSleep(1000);
            teleOpStep=0.1;
        }
        else if (teleOpStep==0.1){
            setWristPosition(0);
            teleOpSleep(1000);
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
            teleOpSleep(1000);
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
            setWristPosition(0);
            teleOpSleep(1000);
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
    private void pickFromSubstationDropInHighJunctionStates() {
        resetSelectedTopHatSpeed();
        if (teleOpStep==2){
            setTopHatSpeed(ATRobotEnumeration.TOPHAT_LOW_SPEED);
        }
        else{
            resetSelectedTopHatSpeed();
        }
        if (teleOpStep == 0) {
            openClaw(true);
            //teleOpSleep(500);
            teleOpStep = 0.1;
        } else if (teleOpStep == 0.1) {
            moveTopHatPosition(0, false, highJunctionArmPos, highJunctionElbowPos, turntable.getCurrentPosition());
            teleOpStep = 1;
        } else if (teleOpStep == 1 && isInRange(highJunctionElbowPos, elbow.getCurrentPosition())
                && isInRange(highJunctionArmPos, arm.getCurrentPosition())) {
            moveTopHatPosition(0, true, highJunctionArmPos, highJunctionElbowPos, subTTHighPickupPos_Loop);
            teleOpStep = 2;
        } else if (teleOpStep == 2 && isInRange(subTTHighPickupPos_Loop, turntable.getCurrentPosition())) {
            moveTopHatPosition(0, true, subArmHighPickupPos_Loop, subElbowHighPickupPos_Loop, subTTHighPickupPos_Loop);
            teleOpStep = 3;
        } else if (teleOpStep == 3 && isInRange(subElbowHighPickupPos_Loop, elbow.getCurrentPosition())
                && isInRange(subArmHighPickupPos_Loop, arm.getCurrentPosition())
                && isInRange(subTTHighPickupPos_Loop, turntable.getCurrentPosition())) {
            setWristPosition(subWristHighPickupPos_Loop);
            teleOpSleep(1000);
            teleOpStep = 3.1;
        } else if (teleOpStep == 3.1 && isInRange(subElbowHighPickupPos_Loop, elbow.getCurrentPosition())
                && isInRange(subArmHighPickupPos_Loop, arm.getCurrentPosition())
                && isInRange(subTTHighPickupPos_Loop, turntable.getCurrentPosition())) {
            openClaw(false);
            teleOpSleep(1000);
            teleOpStep = 3.2;
        } else if (teleOpStep == 3.2 && isInRange(subElbowHighPickupPos_Loop, elbow.getCurrentPosition())
                && isInRange(subArmHighPickupPos_Loop, arm.getCurrentPosition())
                && isInRange(subTTHighPickupPos_Loop, turntable.getCurrentPosition())) {
            setWristPosition(0);
            teleOpSleep(1000);
            teleOpStep = 3.3;
        } else if (teleOpStep == 3.3 && isInRange(subElbowHighPickupPos_Loop, elbow.getCurrentPosition())
                && isInRange(subArmHighPickupPos_Loop, arm.getCurrentPosition())
                && isInRange(subTTHighPickupPos_Loop, turntable.getCurrentPosition())) {
            teleOpStep = 4;
            moveTopHatPosition(.1, false, highJunctionArmPos, highJunctionElbowPos, subTTHighPickupPos_Loop);
        } else if (teleOpStep == 4 && isInRange(highJunctionElbowPos, elbow.getCurrentPosition())
                && isInRange(highJunctionArmPos, arm.getCurrentPosition())) {
            moveTopHatPosition(.1, false, highJunctionArmPos, highJunctionElbowPos, subTTHighDropPos_Loop);
            teleOpStep = 5;
        } else if (teleOpStep == 5 && isInRange(subTTHighDropPos_Loop, turntable.getCurrentPosition())) {
            moveTopHatPosition(subWristHighDropPos_Loop, false, subArmHighDropPos_Loop, subElbowHighDropPos_Loop, subTTHighDropPos_Loop);
            teleOpStep = 6;
        } else if (teleOpStep == 6 && isInRange(subElbowHighDropPos_Loop, elbow.getCurrentPosition())
                && isInRange(subArmHighDropPos_Loop, arm.getCurrentPosition())
                && isInRange(subTTHighDropPos_Loop, turntable.getCurrentPosition())) {
            openClaw(true);
            teleOpSleep(500);
            teleOpStep=6.1;
        } else if (teleOpStep == 6.1 && isInRange(subElbowHighDropPos_Loop, elbow.getCurrentPosition())
                && isInRange(subArmHighDropPos_Loop, arm.getCurrentPosition())
                && isInRange(subTTHighDropPos_Loop, turntable.getCurrentPosition())) {
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
            teleOpSleep(1000);
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
            setWristPosition(0);
            teleOpSleep(1000);
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

    private void pickFromSubstationDropInMedJunctionStates() {
        //resetSelectedTopHatSpeed();
       // if (teleOpStep==2){
            setTopHatSpeed(ATRobotEnumeration.TOPHAT_LOW_AUTON_SPEED_MED_DROP);
       // }
       // else{
        //    resetSelectedTopHatSpeed();
       // }
        if (teleOpStep == 0) {
            openClaw(true);
            //teleOpSleep(500);
            teleOpStep = 0.1;
        } else if (teleOpStep == 0.1) {
            moveTopHatPosition(0, false, mediumJunctionArmPos, mediumJunctionElbowPos, turntable.getCurrentPosition());
            teleOpStep = 1;
        } else if (teleOpStep == 1 && isInRange(mediumJunctionElbowPos, elbow.getCurrentPosition())
                && isInRange(mediumJunctionArmPos, arm.getCurrentPosition())) {
            moveTopHatPosition(0, true, mediumJunctionArmPos, mediumJunctionElbowPos, subTTMedPickupPos);
            teleOpStep = 2;
        } else if (teleOpStep == 2 && isInRange(subTTMedPickupPos, turntable.getCurrentPosition())) {
            moveTopHatPosition(0, true, subArmMedPickupPos, subElbowMedPickupPos, subTTMedPickupPos);
            teleOpStep = 3;
        } else if (teleOpStep == 3 && isInRange(subElbowMedPickupPos, elbow.getCurrentPosition())
                && isInRange(subArmMedPickupPos, arm.getCurrentPosition())
                && isInRange(subTTMedPickupPos, turntable.getCurrentPosition())) {
            setWristPosition(subWristMedPickupPos);
            teleOpSleep(1000);
            teleOpStep = 3.1;
        } else if (teleOpStep == 3.1 && isInRange(subElbowMedPickupPos, elbow.getCurrentPosition())
                && isInRange(subArmMedPickupPos, arm.getCurrentPosition())
                && isInRange(subTTMedPickupPos, turntable.getCurrentPosition())) {
            openClaw(false);
            teleOpSleep(1000);
            teleOpStep = 3.2;
        } else if (teleOpStep == 3.2 && isInRange(subElbowMedPickupPos, elbow.getCurrentPosition())
                && isInRange(subArmMedPickupPos, arm.getCurrentPosition())
                && isInRange(subTTMedPickupPos, turntable.getCurrentPosition())) {
            setWristPosition(0);
            teleOpSleep(1000);
            teleOpStep = 3.3;
        } else if (teleOpStep == 3.3 && isInRange(subElbowMedPickupPos, elbow.getCurrentPosition())
                && isInRange(subArmMedPickupPos, arm.getCurrentPosition())
                && isInRange(subTTMedPickupPos, turntable.getCurrentPosition())) {
            teleOpStep = 4;
            moveTopHatPosition(.1, false, mediumJunctionArmPos, mediumJunctionElbowPos, subTTMedPickupPos);
        } else if (teleOpStep == 4 && isInRange(mediumJunctionElbowPos, elbow.getCurrentPosition())
                && isInRange(mediumJunctionArmPos, arm.getCurrentPosition())) {
            moveTopHatPosition(.1, false, mediumJunctionArmPos, mediumJunctionElbowPos, subTTMedDropPos);
            teleOpStep = 5.1;
        }
        else if (teleOpStep == 5.1 && isInRange(subTTMedDropPos, turntable.getCurrentPosition())) {
            setWristPosition(subWristMedDropPos);
            teleOpSleep(1000);
            teleOpStep = 5.2;
        }
        else if (teleOpStep == 5.2 && isInRange(subTTMedDropPos, turntable.getCurrentPosition())) {
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
            moveTopHatPosition(.6294, false, 2139, -309, 458);
            noOfCones=0;
        }
        if (this.tophatAction == ATRobotEnumeration.DROP_RED_RIGHT_PRELOADED_CONE) {
            openClaw(true);
            noOfCones=0;
        }
        // 13th Nov 2022
        if (this.tophatAction == ATRobotEnumeration.AUTO_RED_RIGHT_HIGH_PICK_CONE) {
            if (step==1 || step==4){
                setTopHatSpeed(ATRobotEnumeration.TOPHAT_MEDIUM_AUTON_SPEED);
            }
            else{
                setTopHatSpeed(ATRobotEnumeration.TOPHAT_MEDIUM_SPEED);
            }
            if (step == 0) {
                if (noOfCones == 0) {
                    desiredWristPosition=.35;
                    desiredElbowPosition = -1632;
                    desiredArmPosition = 1053;
                    desiredTurnTablePosition=184;
                } else if (noOfCones == 1) {
                    desiredWristPosition=.38;
                    desiredElbowPosition = -1596;
                    desiredArmPosition = 1024;
                    desiredTurnTablePosition=181;
                } else if (noOfCones == 2) {
                    desiredWristPosition=.36;
                    desiredElbowPosition = -1565;
                    desiredArmPosition = 884;
                    desiredTurnTablePosition=162;
                } /*else if (noOfCones == 3) {
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
                autonSleep(500);
                setWristPosition(.10);
                autonSleep(500);
                moveTopHatPosition(0.1, false, highJunctionArmPos, highJunctionElbowPos, desiredTurnTablePosition);
                step = 2;
            }
            else if (step == 2 && isInRange(highJunctionElbowPos, elbow.getCurrentPosition())
                    && isInRange(highJunctionArmPos, arm.getCurrentPosition()) && isInRange(desiredTurnTablePosition, turntable.getCurrentPosition())) {
                moveTopHatPosition(.76, false, highJunctionArmPos, highJunctionElbowPos, 1402);
                step = 3;
            }

            else if (step == 3 && isInRange(highJunctionElbowPos, elbow.getCurrentPosition())
                    && isInRange(highJunctionArmPos, arm.getCurrentPosition()) && isInRange(1402, turntable.getCurrentPosition())) {
                moveTopHatPosition(.76, false, 2208, -1302, 1402);
                step = 4;
            }
            else if (step == 4 && isInRange(-1302, elbow.getCurrentPosition())
                    && isInRange(2208, arm.getCurrentPosition())
                    && isInRange(1402, turntable.getCurrentPosition())) {
                autonSleep(500);
                openClaw(true);
                autonSleep(500);
                noOfCones = noOfCones + 1;
                if (!(noOfCones<desiredNoOfConesToPick)){
                    tophatAction = ATRobotEnumeration.AUTO_RED_RIGHT_HIGH_PARK;
                    step=0;
                }
                else{
                    step = 5;
                    moveTopHatPosition(0.1, false, highJunctionArmPos, highJunctionElbowPos, 1402);
                }
        }
            else if (step == 5 && isInRange(highJunctionElbowPos, elbow.getCurrentPosition())
                    && isInRange(highJunctionArmPos, arm.getCurrentPosition())
                    && isInRange(1402, turntable.getCurrentPosition())) {
                moveTopHatPosition(0.1, false, highJunctionArmPos, highJunctionElbowPos, desiredTurnTablePosition);
                step = 6;
            }
            else if (step == 6 && isInRange(highJunctionElbowPos, elbow.getCurrentPosition())
                    && isInRange(highJunctionArmPos, arm.getCurrentPosition())
                    && isInRange(desiredTurnTablePosition, turntable.getCurrentPosition())) {
                    step = 0;
            }
        }
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
            moveTopHatPosition(.6994, false, 2204, -248, 1376); // NEW MOTOR
            if (isInRange(-1366*elbowMultiplier, elbow.getCurrentPosition())
                    && isInRange(4300*armMultiplier, arm.getCurrentPosition())
                    && isInRange(1548*turnTableMultiplier, turntable.getCurrentPosition())){
                tophatAction = ATRobotEnumeration.DROP_BLUE_LEFT_PRELOADED_CONE;
            }
            noOfCones=0;
        }
        if (this.tophatAction == ATRobotEnumeration.DROP_BLUE_LEFT_PRELOADED_CONE) {
            openClaw(true);
            noOfCones=0;
        }
        if (this.tophatAction == ATRobotEnumeration.AUTO_BLUE_LEFT_HIGH_PICK_CONE) {
            if (step==1 || step==4){
                setTopHatSpeed(ATRobotEnumeration.TOPHAT_MEDIUM_AUTON_SPEED);
            }
            else{
                setTopHatSpeed(ATRobotEnumeration.TOPHAT_MEDIUM_SPEED);
            }

            if (step == 0) {
                if (noOfCones == 0) {
                    desiredWristPosition=.3999;
                    desiredElbowPosition = -1696;//NEW MOTOR-4637*elbowMultiplier;
                    desiredArmPosition = 1135;//NEW MOTOR1216*armMultiplier;
                    desiredTurnTablePosition=1646;//NEW MOTOR1800*turnTableMultiplier;
                } else if (noOfCones == 1) {
                    desiredWristPosition=.36;
                    desiredElbowPosition = -1681;//NEW MOTOR-4765*elbowMultiplier;
                    desiredArmPosition = 1077;//NEW MOTOR1218*armMultiplier;
                    desiredTurnTablePosition=1648;//NEW MOTOR1800*turnTableMultiplier;

                }else if (noOfCones == 2) {
                    desiredWristPosition=.3961;
                    desiredElbowPosition = -1574;
                    desiredArmPosition = 899;
                    desiredTurnTablePosition=1660;
                } /*else if (noOfCones == 3) {
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
                moveTopHatPosition(0.05, false, desiredArmPosition, desiredElbowPosition, desiredTurnTablePosition);
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
                autonSleep(500);
                setWristPosition(.10);
                autonSleep(500);
                moveTopHatPosition(0.05, false, highJunctionArmPos, highJunctionElbowPos, desiredTurnTablePosition);
                step = 2;
            }
            else if (step == 2 && isInRange(highJunctionElbowPos, elbow.getCurrentPosition())
                    && isInRange(highJunctionArmPos, arm.getCurrentPosition())) {
                moveTopHatPosition(.830, false, highJunctionArmPos, highJunctionElbowPos, 464);//NEW MOTOR
                step = 3;
            }

            else if (step == 3 && isInRange(466, turntable.getCurrentPosition())) {//NEW MOTOR
                moveTopHatPosition(.830, false, 1620, -913, 464); //NEW MOTOR
                step = 4;
            }
            else if (step == 4 && isInRange(-913, elbow.getCurrentPosition())
                    && isInRange(1620, arm.getCurrentPosition())
                    && isInRange(464, turntable.getCurrentPosition())) {
                autonSleep(500);
                openClaw(true);
                autonSleep(500);
                noOfCones = noOfCones + 1;
                if (!(noOfCones<desiredNoOfConesToPick)){
                    tophatAction = ATRobotEnumeration.AUTO_BLUE_LEFT_HIGH_PARK;
                    step=0;
                }
                else{
                    step = 5;
                    moveTopHatPosition(0.05, false, highJunctionArmPos, highJunctionElbowPos, 464);
                }
            }
            else if (step == 5 && isInRange(highJunctionElbowPos, elbow.getCurrentPosition())
                    && isInRange(highJunctionArmPos, arm.getCurrentPosition())) {
                moveTopHatPosition(0.05, false, highJunctionArmPos, highJunctionElbowPos, desiredTurnTablePosition);
                step = 6;
            }
            else if (step == 6 && isInRange(highJunctionElbowPos, elbow.getCurrentPosition())
                    && isInRange(highJunctionArmPos, arm.getCurrentPosition())
                    && isInRange(desiredTurnTablePosition, turntable.getCurrentPosition())) {
                step = 0;
            }
        }
    }
    public void blueAllianceLeftAutonMedium(){
        if (this.tophatAction == ATRobotEnumeration.AUTO_BLUE_LEFT_MEDIUM_SETUP) {
            moveTopHatPosition(.9, false, robotMidPointArmPosition, -20*elbowMultiplier, 1600*turnTableMultiplier);
            noOfCones=0;
        }
        if (this.tophatAction == ATRobotEnumeration.SET_BLUE_LEFT_PRELOADED_CONE) {
            moveTopHatPosition(.6994, false, 2204, -248, 1376); // NEW MOTOR
            if (isInRange(-1366*elbowMultiplier, elbow.getCurrentPosition())
                    && isInRange(4300*armMultiplier, arm.getCurrentPosition())
                    && isInRange(1548*turnTableMultiplier, turntable.getCurrentPosition())){
                tophatAction = ATRobotEnumeration.DROP_BLUE_LEFT_PRELOADED_CONE;
            }
            noOfCones=0;
        }
        if (this.tophatAction == ATRobotEnumeration.DROP_BLUE_LEFT_PRELOADED_CONE) {
            openClaw(true);
            noOfCones=0;
        }
        if (this.tophatAction == ATRobotEnumeration.AUTO_BLUE_LEFT_MEDIUM_PICK_CONE) {
            if (step == 0) {
                if (noOfCones == 0) {
                    desiredWristPosition=.38;
                    desiredElbowPosition = -1596;//NEW MOTOR-4637*elbowMultiplier;
                    desiredArmPosition = 997;//NEW MOTOR1216*armMultiplier;
                    desiredTurnTablePosition=159;//NEW MOTOR1800*turnTableMultiplier;
                } else if (noOfCones == 1) {
                    desiredWristPosition=.3999;
                    desiredElbowPosition = -1565;//NEW MOTOR-4765*elbowMultiplier;
                    desiredArmPosition = 933;//NEW MOTOR1218*armMultiplier;
                    desiredTurnTablePosition=147;//NEW MOTOR1800*turnTableMultiplier;

                }else if (noOfCones == 2) {
                    desiredWristPosition=.37;
                    desiredElbowPosition = -1564;
                    desiredArmPosition = 879;
                    desiredTurnTablePosition=147;
                } else if (noOfCones == 3) {
                    desiredWristPosition=.3999;
                    desiredElbowPosition = -1540;
                    desiredArmPosition = 837;
                    desiredTurnTablePosition=147;
                } /* else if (noOfCones == 4) {
                    desiredWristPosition=.48;
                    desiredElbowPosition = -4520;
                    desiredArmPosition = 1478;
                    desiredTurnTablePosition=1786;
                }*/
                moveTopHatPosition(0.05, false, desiredArmPosition, desiredElbowPosition, desiredTurnTablePosition);
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
                moveTopHatPosition(0.05, false, mediumJunctionArmPos, mediumJunctionElbowPos, desiredTurnTablePosition);
                step = 2;
            }
            else if (step == 2 && isInRange(mediumJunctionElbowPos, elbow.getCurrentPosition())
                    && isInRange(mediumJunctionArmPos, arm.getCurrentPosition())) {
                moveTopHatPosition(0.05, false, mediumJunctionArmPos, mediumJunctionElbowPos, 1403);//NEW MOTOR
                step = 3;
            }

            else if (step == 3 && isInRange(1403, turntable.getCurrentPosition())) {//NEW MOTOR
                moveTopHatPosition(.89, false, 990, -558, 1403); //NEW MOTOR
                step = 4;
            }
            else if (step == 4 && isInRange(-558, elbow.getCurrentPosition())
                    && isInRange(990, arm.getCurrentPosition())
                    && isInRange(1403, turntable.getCurrentPosition())) {
                openClaw(true);
                autonSleep(500);
                noOfCones = noOfCones + 1;
                if (!(noOfCones<desiredNoOfConesToPick)){
                    tophatAction = ATRobotEnumeration.AUTO_BLUE_LEFT_MEDIUM_PARK;
                    step=0;
                }
                else{
                    step = 5;
                    moveTopHatPosition(0.05, false, mediumJunctionArmPos, mediumJunctionElbowPos, 1403);
                }
            }
            else if (step == 5 && isInRange(mediumJunctionElbowPos, elbow.getCurrentPosition())
                    && isInRange(mediumJunctionArmPos, arm.getCurrentPosition())) {
                moveTopHatPosition(0.05, false, mediumJunctionArmPos, mediumJunctionElbowPos, desiredTurnTablePosition);
                step = 6;
            }
            else if (step == 6 && isInRange(mediumJunctionElbowPos, elbow.getCurrentPosition())
                    && isInRange(mediumJunctionArmPos, arm.getCurrentPosition())
                    && isInRange(desiredTurnTablePosition, turntable.getCurrentPosition())) {
                step = 0;
            }
        }
    }
    public boolean isTopHatInParkingPosition(){
        if (step==0){
            setWristPosition(1);
//            autonSleep(500);
            moveTopHatPosition(1, false, 4300*armMultiplier, -1800*elbowMultiplier, turntable.getCurrentPosition());
            step=1;
        }
        else if (step == 1&& isInRange(-1800*elbowMultiplier, elbow.getCurrentPosition())
                && isInRange(4300*armMultiplier, arm.getCurrentPosition())){
            moveTopHatPosition(1, false, 4300*armMultiplier, -1800*elbowMultiplier, parkingTurnTablePosition);
            step=2;
        }
        else if (step==2 && isInRange(parkingTurnTablePosition, turntable.getCurrentPosition())){
            step=0;
            return true;
        }
        return false;
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
            moveTopHatPosition(.6794, false, 2013, -216, 402);
            noOfCones=0;
        }
        if (this.tophatAction == ATRobotEnumeration.DROP_BLUE_RIGHT_PRELOADED_CONE) {
            openClaw(true);
            noOfCones=0;
        }
        if (this.tophatAction == ATRobotEnumeration.AUTO_BLUE_RIGHT_HIGH_PICK_CONE) {
            if (step==1 || step==4){
                setTopHatSpeed(ATRobotEnumeration.TOPHAT_MEDIUM_AUTON_SPEED);
            }
            else{
                setTopHatSpeed(ATRobotEnumeration.TOPHAT_MEDIUM_SPEED);
            }
            if (step == 0) {
                if (noOfCones == 0) {
                    desiredWristPosition=.37;
                    desiredElbowPosition = -1655;
                    desiredArmPosition = 1126;
                    desiredTurnTablePosition=153;
                } else if (noOfCones == 1) {
                    desiredWristPosition=.41;
                    desiredElbowPosition = -1604;
                    desiredArmPosition = 996;
                    desiredTurnTablePosition=139;
                } else if (noOfCones == 2) {
                    desiredWristPosition=.41;
                    desiredElbowPosition = -1569;
                    desiredArmPosition = 905;
                    desiredTurnTablePosition=139;
                } /*else if (noOfCones == 3) {
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
                autonSleep(500);
                setWristPosition(.10);
                autonSleep(500);
                moveTopHatPosition(0.1, false, highJunctionArmPos, highJunctionElbowPos, desiredTurnTablePosition);
                step = 2;
            }
            else if (step == 2 && isInRange(highJunctionElbowPos, elbow.getCurrentPosition())
                    && isInRange(highJunctionArmPos, arm.getCurrentPosition())) {
                moveTopHatPosition(0.1, false, highJunctionArmPos, highJunctionElbowPos, 1416);
                step = 3;
            }

            else if (step == 3 && isInRange(1416, turntable.getCurrentPosition())) {
                moveTopHatPosition(.84, false, 2047, -1165,1416);
                step = 4;
            }
            else if (step == 4 && isInRange(-1165, elbow.getCurrentPosition())
                    && isInRange(2047, arm.getCurrentPosition())
                    && isInRange(1416, turntable.getCurrentPosition())) {
                autonSleep(500);
                openClaw(true);
                autonSleep(500);
                noOfCones = noOfCones + 1;
                if (!(noOfCones<desiredNoOfConesToPick)){
                    tophatAction = ATRobotEnumeration.AUTO_BLUE_RIGHT_HIGH_PARK;
                    step=0;
                }
                else{
                    step = 5;
                    moveTopHatPosition(0.1, false, highJunctionArmPos, highJunctionElbowPos, 1416);
                }
            }
            else if (step == 5 && isInRange(highJunctionElbowPos, elbow.getCurrentPosition())
                    && isInRange(highJunctionArmPos, arm.getCurrentPosition())) {
                moveTopHatPosition(0.1, false, highJunctionArmPos, highJunctionElbowPos, desiredTurnTablePosition);
                step = 6;
            }
            else if (step == 6 && isInRange(highJunctionElbowPos, elbow.getCurrentPosition())
                    && isInRange(highJunctionArmPos, arm.getCurrentPosition())
                    && isInRange(desiredTurnTablePosition, turntable.getCurrentPosition())) {
                step = 0;
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
            moveTopHatPosition(.6788, false, 2150, -270, 1358);

            //This is for high drop with preloaded
            //moveTopHatPosition(.53, false, 4300, -2152, 1900);
            noOfCones=0;
        }
        if (this.tophatAction == ATRobotEnumeration.DROP_RED_LEFT_PRELOADED_CONE) {
            openClaw(true);
            noOfCones=0;
        }
        if (this.tophatAction == ATRobotEnumeration.AUTO_RED_LEFT_HIGH_PICK_CONE) {
            if (step==1 || step==4){
                setTopHatSpeed(ATRobotEnumeration.TOPHAT_MEDIUM_AUTON_SPEED);
            }
            else{
                setTopHatSpeed(ATRobotEnumeration.TOPHAT_MEDIUM_SPEED);
            }
            if (step == 0) {
                if (noOfCones == 0) {
                    desiredWristPosition=.38;
                    desiredElbowPosition = -1673;
                    desiredArmPosition = 1155;
                    desiredTurnTablePosition=1668;
                } else if (noOfCones == 1) {
                    desiredWristPosition=.38;
                    desiredElbowPosition = -1641;
                    desiredArmPosition = 1040;
                    desiredTurnTablePosition=1669;
                } else if (noOfCones == 2) {
                    desiredWristPosition=.38;
                    desiredElbowPosition = -1597;
                    desiredArmPosition = 945;
                    desiredTurnTablePosition=1663;
                } /*else if (noOfCones == 3) {
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
                autonSleep(500);
                setWristPosition(.10);
                autonSleep(500);
                moveTopHatPosition(0.1, false,  highJunctionArmPos, highJunctionElbowPos, desiredTurnTablePosition);
                step = 2;
            }
            else if (step == 2 && isInRange(highJunctionElbowPos, elbow.getCurrentPosition())
                    && isInRange(highJunctionArmPos, arm.getCurrentPosition())) {
                moveTopHatPosition(.81, false, highJunctionArmPos, highJunctionElbowPos, 421);
                step = 3;
            }

            else if (step == 3 && isInRange(421, turntable.getCurrentPosition())) {
                moveTopHatPosition(.81, false, 1976, -1128, 421);
                step = 4;
            }
            else if (step == 4 && isInRange(-1128, elbow.getCurrentPosition())
                    && isInRange(1976, arm.getCurrentPosition())
                    && isInRange(421, turntable.getCurrentPosition())) {
                autonSleep(500);
                openClaw(true);
                autonSleep(500);
                noOfCones = noOfCones + 1;
                if (!(noOfCones<desiredNoOfConesToPick)){
                    tophatAction = ATRobotEnumeration.AUTO_RED_LEFT_HIGH_PARK;
                    step=0;
                }
                else{
                    step = 5;
                    moveTopHatPosition(0.1, false, highJunctionArmPos, highJunctionElbowPos, 421);                }
            }
            else if (step == 5 && isInRange(highJunctionElbowPos, elbow.getCurrentPosition())
                    && isInRange(highJunctionArmPos, arm.getCurrentPosition())) {
                moveTopHatPosition(0.1, false, highJunctionArmPos, highJunctionElbowPos, desiredTurnTablePosition);
                step = 6;
            }
            else if (step == 6 && isInRange(highJunctionElbowPos, elbow.getCurrentPosition())
                    && isInRange(highJunctionArmPos, arm.getCurrentPosition())
                    && isInRange(desiredTurnTablePosition, turntable.getCurrentPosition())) {
                step = 0;
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

    private void tophatDriverActionsNovi(){
        fullManualControl_New();
        partialManualControl();
    }

    private void tophatDriverActionsStateChamp(){

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

        if (gamepad2.left_stick_y != 0 && gamepad2.left_bumper && gamepad2.right_bumper && !gamepad2.dpad_up
                && !gamepad2.dpad_down) {
            tophatAction = ATRobotEnumeration.MANUAL;
            ArmPosition += -gamepad2.left_stick_y*ArmSpeed;
            telemetry.addData("left stick y Arm Up or Down", "Pressed");
        }
        if (gamepad2.left_stick_y != 0 && gamepad2.left_bumper && gamepad2.right_bumper && gamepad2.dpad_up) {
            telemetry.addData("Arm speed increasing", "Pressed");
            ArmVelocity = ArmVelocity+ (int) (ArmVelocity * 1.25);
        }
        if (gamepad2.left_stick_y != 0 && gamepad2.left_bumper && gamepad2.right_bumper && gamepad2.dpad_down) {
            telemetry.addData("Arm speed decreasing", "Pressed");
            ArmVelocity = ArmVelocity- (int) (ArmVelocity * 1.25);
        }
        if (gamepad2.right_stick_y != 0 && gamepad2.left_bumper && gamepad2.right_bumper&& !gamepad2.dpad_up
                && !gamepad2.dpad_down) {
            tophatAction = ATRobotEnumeration.MANUAL;
            ElbowPosition += gamepad2.right_stick_y*ElbowSpeed;
            telemetry.addData("right stick y Elbow Up or Down", "Pressed");
        }
        if (gamepad2.right_stick_y != 0 && gamepad2.left_bumper && gamepad2.right_bumper && gamepad2.dpad_up) {
            telemetry.addData("Elbow speed increasing", "Pressed");
            ElbowVelocity = ElbowVelocity+ (int) (ElbowVelocity * 1.25);
        }
        if (gamepad2.right_stick_y != 0 && gamepad2.left_bumper && gamepad2.right_bumper && gamepad2.dpad_down) {
            telemetry.addData("Elbow speed decreasing", "Pressed");
            ElbowVelocity = ElbowVelocity- (int) (ElbowVelocity * 1.25);
        }
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
        if (gamepad2.dpad_left && gamepad2.left_bumper && gamepad2.right_bumper)
        {
            tophatAction = ATRobotEnumeration.MANUAL;
            openClaw(true);
            HoldCone = false;
            telemetry.addData("DpadLeft Claw Open", "Pressed");
        }
        if (gamepad2.dpad_right && gamepad2.left_bumper && gamepad2.right_bumper)
        {
            tophatAction = ATRobotEnumeration.MANUAL;
            openClaw(false);
            HoldCone = true;
            telemetry.addData("DpadRight Claw Close", "Pressed");
        }

        /**
         * This is to preset TopHat to pickup cone from Alliance side specific substation for Medium Junction Drop
         * This needs to be added to the instruction sheet and not to analyze this as left or right instead focus on High vs
         * Medium Junction
         */
       /*SV SEVA

        if (gamepad2.left_trigger == 0 && gamepad2.right_trigger > 0 && gamepad2.x){
            tophatAction = ATRobotEnumeration.AUTO_TOPHAT_GROUND_MOVE_BEGIN;
            teleOpStep=0;
            setTopHatPosition(subWristMedPickupPos, true, subArmMedPickupPos, subElbowMedPickupPos, subTTMedPickupPos);
        }

        /**
         * This is to preset TopHat to pickup cone from Alliance side specific substation for High Junction Drop
         * This needs to be added to the instruction sheet and not to analyze this as left or right instead focus on High vs
         * Medium Junction
         */
        /*SV SEVA
        if (gamepad2.left_trigger > 0 && gamepad2.right_trigger == 0 && gamepad2.x){
            tophatAction = ATRobotEnumeration.AUTO_TOPHAT_GROUND_MOVE_BEGIN;
            teleOpStep=0;
            setTopHatPosition(subWristHighPickupPos, true, subArmHighPickupPos, subElbowHighPickupPos, subTTHighPickupPos);
        }

        /**
         * This is to preset TopHat to pickup cone from Alliance side specific substation and
         * have TopHat stay at medium junction specific height
         * this can be used to pickup cone and navigate anywhere within the filed to either own a junction
         * or complete the circuit
         */
        /*SV SEVA
        if (gamepad2.right_trigger>0 && gamepad2.left_trigger>0 && gamepad2.x){
            tophatAction = ATRobotEnumeration.PICK_CONE_READY_TO_NAVIGATE;
            teleOpStep=0;
        }

        /**
         * This is to pickup cone from Alliance side specific substation and have it ready to drop
         * in high junction
         */
        /*SV SEVA
        if (gamepad2.left_bumper && gamepad2.right_bumper && gamepad2.y){
            tophatAction = ATRobotEnumeration.PICK_CONE_SET_TO_DRPOP_HIGH;
            teleOpStep=0;
            substationPickConfig =ATRobotEnumeration.PICK_CONE_SET_TO_DRPOP_HIGH;
        }

        /**
         * This is to preset TopHat to pickup cone from Alliance side specific substation and
         * have TopHat continue dropping cones till action being interrupted by pressing any additional key
         */
        /*SV SEVA
        if (gamepad2.right_trigger>0 && gamepad2.left_trigger>0 && gamepad2.y){
            tophatAction = ATRobotEnumeration.PICK_CONE_DROP_HIGH_IN_LOOP;
            teleOpStep=0;
        }
        /**
         * This is to pickup cone from Alliance side specific substation and have it ready to drop
         * in medium junction
         */
        /*SV SEVA
        if (gamepad2.left_bumper && gamepad2.right_bumper && gamepad2.b){
            tophatAction = ATRobotEnumeration.PICK_CONE_SET_TO_DRPOP_MEDIUM;
            teleOpStep=0;
            substationPickConfig =ATRobotEnumeration.PICK_CONE_SET_TO_DRPOP_MEDIUM;
        }
        /**
         * This is to preset TopHat to pickup cone from Alliance side specific substation and
         * have TopHat continue dropping cones in Medium Junction till action being interrupted by pressing any additional key
         */
        /*SV SEVA
        if (gamepad2.right_trigger>0 && gamepad2.left_trigger>0 && gamepad2.b){
            tophatAction = ATRobotEnumeration.PICK_CONE_DROP_MEDIUM_IN_LOOP;
            teleOpStep=0;
        }
        /**
         * This is to increase the tophat motors speed
         */

        if (gamepad2.left_trigger > 0 && gamepad2.right_trigger > 0 && gamepad2.dpad_up){
            tophatAction = ATRobotEnumeration.SPEED_UP;
            tophatMode = ATRobotEnumeration.MANUAL;
            setTopHatSpeed(ATRobotEnumeration.TOPHAT_HIGH_SPEED);
            selectedTopHatSpeed=ATRobotEnumeration.TOPHAT_HIGH_SPEED;
        }

        /**
         * This is to decrease the tophat motors speed
         */

        if (gamepad2.left_trigger > 0 && gamepad2.right_trigger > 0 && gamepad2.dpad_down){
            tophatAction = ATRobotEnumeration.SPEED_DOWN;
            tophatMode = ATRobotEnumeration.MANUAL;
            setTopHatSpeed(ATRobotEnumeration.TOPHAT_MEDIUM_SPEED);
            selectedTopHatSpeed=ATRobotEnumeration.TOPHAT_MEDIUM_SPEED;
        }

    }

    public void setTopHatSpeed(ATRobotEnumeration speedLevel) {
        switch (speedLevel) {
            case TOPHAT_HIGH_SPEED:{
                setTopHatMotorsVelocity(4000, 4000,4000);
                topHatSpeed=ATRobotEnumeration.TOPHAT_HIGH_SPEED;
            }
            break;
            case TOPHAT_MEDIUM_SPEED:{
                setTopHatMotorsVelocity(2000, 2000,2000);
                topHatSpeed=ATRobotEnumeration.TOPHAT_MEDIUM_SPEED;
            }
            break;
            case TOPHAT_MEDIUM_AUTON_SPEED:{
                setTopHatMotorsVelocity(2000, 1500,1500);
                topHatSpeed=ATRobotEnumeration.TOPHAT_MEDIUM_AUTON_SPEED;
            }
            break;
            case TOPHAT_LOW_SPEED:{
                setTopHatMotorsVelocity(1000, 1000,1000);
                topHatSpeed=ATRobotEnumeration.TOPHAT_LOW_SPEED;
            }
            break;
            case TOPHAT_LOW_AUTON_SPEED:{
                setTopHatMotorsVelocity(750, 1000,750);
                topHatSpeed=ATRobotEnumeration.TOPHAT_LOW_AUTON_SPEED;
            }
            break;
            case TOPHAT_LOW_AUTON_SPEED_MED_DROP:{
                setTopHatMotorsVelocity(1000, 1500,1500);
                topHatSpeed=ATRobotEnumeration.TOPHAT_LOW_AUTON_SPEED_MED_DROP;
            }
            break;
        }
    }
    private void resetSelectedTopHatSpeed(){
        setTopHatSpeed(selectedTopHatSpeed);
    }
}