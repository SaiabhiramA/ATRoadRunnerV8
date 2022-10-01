package freightfrenzy.teamcode.drive;

//import String;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.TfodCurrentGame;

import java.util.List;

public class TopHatControllerV03 {

    private DcMotor intake;
    private DcMotor carosuelmoter;
    private TfodCurrentGame tfodFreightFrenzy;
    private DcMotor turntable;
    private DcMotor arm;
    private TouchSensor turntabletouch;
    private TouchSensor armdowntouch;
    private TouchSensor armuptouch;
    private Servo intakebox;
    private VuforiaCurrentGame vuforiaFreightFrenzy;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor rearLeft;
    private DcMotor rearRight;
    private HardwareMap hwMap;
    private Telemetry teleMetry;
    double xPosition_Left;
    double xPosition_Right;
    boolean IsDetected=false;
    List<Recognition> recognitions;
    Recognition recognition;

    String ducklocation;
    int ArmVelocity;
    int TurnTableVelocity;

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    private static final String VUFORIA_KEY =
            "AUvp1av/////AAABmYuKAzJt/UOavRbICexbEiCJ9QjMC+uPlQoNmvDN6FaDGSx/mtAb2fMnxiNTXJnAd9VCjw0E+ltSXjEkSPxWkK7qJmgmuW7mDbK6K7+7ngbGlMiONyLewAxIKteUvQqXmdJ5ryxNfZTrURzv4ZO+eojQc1gRnB1a+rH0yL+TETEmG5qCtlmvs/GAYTTADTKVj+4M7d8NyVDlrBw3uJQWTKmJxi/ufaP4LpZebYKHTI+3joqDQ5SCx/NiFyFlEI1ShHzMA2d5WJINSVw9qGlB4pHNSOvkm3a0hg66QQCgWlRcwsQ2KVOF470KU+/V5xPgbT7fx0lC90tONd2CQsmh1ND2qcAiToHH0vSQZeE8Yxm7";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;



    /**
     * Describe this function...
     */
    public void initializeRobot(HardwareMap hardwareMapAT, Telemetry telemetry, String Alliance) {
        boolean AutoArm;
        boolean AutoTurnTable;
        double WheelPower;
        boolean AutoIntake;
        String RobotPosition;
        boolean IsArmPositionReady;
        hwMap=hardwareMapAT;
        teleMetry=telemetry;
        intake = hwMap.get(DcMotor.class, "intake");
        carosuelmoter = hwMap.get(DcMotor.class, "carosuelmoter");
        tfodFreightFrenzy = new TfodCurrentGame();
        turntable = hwMap.get(DcMotor.class, "turntable");
        arm = hwMap.get(DcMotor.class, "arm");
        turntabletouch = hwMap.get(TouchSensor.class, "turntabletouch");
        armdowntouch = hwMap.get(TouchSensor.class, "armdowntouch");
        armuptouch = hwMap.get(TouchSensor.class, "armuptouch");
        intakebox = hwMap.get(Servo.class, "intakebox");
        vuforiaFreightFrenzy = new VuforiaCurrentGame();
        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        frontRight = hwMap.get(DcMotor.class, "frontRight");
        rearLeft = hwMap.get(DcMotor.class, "rearLeft");
        rearRight = hwMap.get(DcMotor.class, "rearRight");

        // Put initialization blocks here.
        ActivateDuckDetection();
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carosuelmoter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        carosuelmoter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ArmVelocity = 5000;
        TurnTableVelocity = 5000;
        WheelPower = 0.5;
        AutoArm = false;
        AutoIntake = false;
        AutoTurnTable = false;
        RobotPosition = "R";
        IsArmPositionReady = false;
        //remove intake power
        IntakeAction("R", -0.2);//12172021-Fixed Block holding
        ResetArmNTurnTableAuto();
        IntakeAction("R", 0);
        setIntakeBoxPosition("R");
        //sleep(10000);
        //recognitions = tfod.getRecognitions();
        detectDuckByAllianceMarkers(Alliance,"INIT");
         }


    public void initializeRobotArmOnly(HardwareMap hardwareMapAT, Telemetry telemetry) {
        boolean AutoArm;
        boolean AutoTurnTable;
        double WheelPower;
        boolean AutoIntake;
        String RobotPosition;
        boolean IsArmPositionReady;
        hwMap=hardwareMapAT;
        teleMetry=telemetry;
        intake = hwMap.get(DcMotor.class, "intake");
        carosuelmoter = hwMap.get(DcMotor.class, "carosuelmoter");
        tfodFreightFrenzy = new TfodCurrentGame();
        turntable = hwMap.get(DcMotor.class, "turntable");
        arm = hwMap.get(DcMotor.class, "arm");
        turntabletouch = hwMap.get(TouchSensor.class, "turntabletouch");
        armdowntouch = hwMap.get(TouchSensor.class, "armdowntouch");
        armuptouch = hwMap.get(TouchSensor.class, "armuptouch");
        intakebox = hwMap.get(Servo.class, "intakebox");
        vuforiaFreightFrenzy = new VuforiaCurrentGame();
        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        frontRight = hwMap.get(DcMotor.class, "frontRight");
        rearLeft = hwMap.get(DcMotor.class, "rearLeft");
        rearRight = hwMap.get(DcMotor.class, "rearRight");

        // Put initialization blocks here.
        //ActivateDuckDetection();
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carosuelmoter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        carosuelmoter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ArmVelocity = 5000;
        TurnTableVelocity = 5000;
        WheelPower = 0.5;
        AutoArm = false;
        AutoIntake = false;
        AutoTurnTable = false;
        RobotPosition = "R";
        IsArmPositionReady = false;
        IntakeAction("R", -0.2);
        ResetArmNTurnTableAuto();
        IntakeAction("R", 0);

    }



    private void blueAllianceDetectDuckLocation(String State) {
        ducklocation="LEFT";
        // Get a list of recognitions from TFOD.
        if (State.equals("INIT")) {
            recognitions = tfod.getRecognitions();
        }
        else{
            recognitions = tfod.getUpdatedRecognitions();
        }
        for (Recognition recognition_item : recognitions) {
            recognition = recognition_item;
            if (recognition.getLabel().equals("Duck")) {
                xPosition_Left = Double.parseDouble(JavaUtil.formatNumber(recognition.getLeft(), 0));
                xPosition_Right = Double.parseDouble(JavaUtil.formatNumber(recognition.getRight(), 0));
                if (xPosition_Left >= 1 && xPosition_Left <= 180) {
                    ducklocation = "CENTER";
                } else {
                    ducklocation = "RIGHT";
                }
            }
        }
        }
    /**
     * Describe this function...
     */
    private void redAllianceDetectDuckLocation(String State) {
        ducklocation = "RIGHT";
        // Get a list of recognitions from TFOD.
        if (State.equals("INIT")) {
            recognitions = tfod.getRecognitions();
        }
        else{
            recognitions = tfod.getUpdatedRecognitions();
        }
        for (Recognition recognition_item : recognitions) {
            recognition = recognition_item;
            if (recognition.getLabel().equals("Duck")) {
                xPosition_Left = Double.parseDouble(JavaUtil.formatNumber(recognition.getLeft(), 0));
                xPosition_Right = Double.parseDouble(JavaUtil.formatNumber(recognition.getRight(), 0));
                if (xPosition_Left >= 1 && xPosition_Left <= 180) {
                    ducklocation = "LEFT";
                } else{
                    ducklocation = "CENTER";}
            }
        }
      }

    public void detectDuckByAllianceMarkers(String Alliance,String State){
        if (Alliance == "RED") {
            redAllianceDetectDuckLocation(State);
        }
        if (Alliance == "BLUE") {
            blueAllianceDetectDuckLocation(State);
        }
        teleMetry.update();
    }

    /**
     * Describe this function...
     */
    public void ResetArmNTurnTableAuto() {

        setIntakeBoxPosition("R");

        //if (turntable.getCurrentPosition() > 300 || arm.getCurrentPosition() < -100) {
            ResetArmUp();
            ResetTurnTable();
            ResetArmDown();

        //}
    }

    /**
     * Describe this function...
     */
    public void IntakeAction(String action, double speed) {
        if (action.equals("I")) {
            intake.setPower(speed);
        } else if (action.equals("O")) {
            intake.setPower(speed);
        } else if (action.equals("R")) {
            intake.setPower(speed);
        }
        sleep(2000);
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }


    /**
     * Describe this function...
     */
    public void Red_alliance_carousel() {
        // Red alliance Carousel
        carosuelmoter.setPower(-0.15);
        sleep(4000);
        carosuelmoter.setPower(0);
    }

    /**
     * Describe this function...
     */
    public void setArmDropPosition(String armpos) {
        if (armpos.equals("L")) {
            // Lower Shelf of Shipping Hub
            SetArmPosition(-996);
        } else if (armpos.equals("M")) {
            // Middle Shelf of Shipping Hub
            SetArmPosition(-1987);
        } else if (armpos.equals("T")) {
            // Top Shelf of Shipping Hub
            SetArmPosition(-3167);
            teleMetry.addData("Top Shelf", arm.getCurrentPosition());
        } else if (armpos.equals("W")) {
            // Warehouse Clearance & Shared Shipping Hub
            SetArmPosition(-500);
        }
        else if(armpos.equals("P")){
            SetArmPosition(0);
        }
    }

    /**
     * Describe this function...
     */
    public void MoveTurnTablePosition() {
        SetArmPosition(-2000);//changed from 1500
        SetTurnTablePosition(4386);
    }


    /**
     * Describe this function...
     */
    private void ResetTurnTable() {
        if (!turntabletouch.isPressed()) {
            turntable.setTargetPosition(-5000);
            turntable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx) turntable).setVelocity(TurnTableVelocity);
            while (!turntabletouch.isPressed()) {
                teleMetry.addData("Reset Turn Table Position", turntable.getCurrentPosition());
            }
            turntable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            SetTurnTablePosition(300);
        }
    }

    /**
     * Describe this function...
     */
    public void Blue_alliance_carousel() {
        // Blue alliance Carousel
        carosuelmoter.setPower(0.20);
        sleep(4000);
        carosuelmoter.setPower(0);
    }

    /**
     * Describe this function...
     */
    private void SetArmPosition(int Position2) {
        arm.setTargetPosition(Position2);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) arm).setVelocity(ArmVelocity);
     // while (!(arm.isBusy() || armdowntouch.isPressed() || armuptouch.isPressed())) {
      // }
    }

    /**
     * Describe this function...
     */
    private void ResetArmUp() {
        if (!armuptouch.isPressed()) {
            arm.setTargetPosition(-5000);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx) arm).setVelocity(ArmVelocity);
            while (!armuptouch.isPressed()) {
                teleMetry.addData("Reset Arm Position", arm.getCurrentPosition());
            }
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    /**
     * Describe this function...
     */
    public void setIntakeBoxPosition(String boxpos) {
        if (boxpos.equals("T")) {
            intakebox.setPosition(0.38);
            // Top shelf drop
        } else if (boxpos.equals("M")) {
            intakebox.setPosition(0.649);
            // Middle shelf drop
        } else if (boxpos.equals("L")) {
            intakebox.setPosition(0.80);
            // Lower Shelf Drop
        } else if (boxpos.equals("B")) {
            // Drop at Shared Hub
            intakebox.setPosition(0.38);
        } else if (boxpos.equals("I")) {
            intakebox.setPosition(0.649);
            // Intake position
        } else if (boxpos.equals("R")) {
            intakebox.setPosition(0.27);
            // ResetPosition
        }
    }

    /**
     * Describe this function...
     */
    private void AutoSpinCarousel(
            // TODO: Enter the type for argument named Alliance
            String Alliance) {
        if (Alliance == "RED") {
            Red_alliance_carousel();
        }
        if (Alliance == "BLUE") {
            Blue_alliance_carousel();
        }
      }

    /**
     * Describe this function...
     */
    private void DeActivateDuckDetection() {
        // Deactivate TFOD.
        tfodFreightFrenzy.deactivate();
    }

    /**
     * Describe this function...
     */
    private void SetTurnTablePosition(int Position2) {
        turntable.setTargetPosition(Position2);
        turntable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) turntable).setVelocity(TurnTableVelocity);
       // while (!(turntable.isBusy() || turntabletouch.isPressed())) {
        //}
    }

    /**
     * Describe this function...
     */
    private void ActivateDuckDetection() {

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.1, 16.0/9.0);
        }

    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hwMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    /**
     * Describe this function...
     */
    public void DropBlock() {
        IntakeAction("O", .3);
        intake.setPower(0);
        teleMetry.update();
        setIntakeBoxPosition("R");
    }
public String duckLocationSymbol(){
    if (ducklocation.equals("CENTER")) {
        // Middle Shelf of Shipping Hub
        teleMetry.addData("Center", "Center");
        return "M";
        // Lower Shelf of Shipping Hub
    } else if (ducklocation.equals("LEFT")) {
        teleMetry.addData("Lower", "Lower");
        return "L";
        // Lower Shelf of Shipping Hub
    } else {
        teleMetry.addData("Top", "Top");
        return "T";
        // Top Shelf of Shipping Hub
    }
}

    /**
     * Describe this function...
     */
    public void Display_Key_Measures(String State) {
        /*teleMetry.addData("Front Left Position", frontLeft.getCurrentPosition());
        teleMetry.addData("Front Right Position", frontRight.getCurrentPosition());
        teleMetry.addData("Rear Left Position", rearLeft.getCurrentPosition());
        teleMetry.addData("Rear Right Position", rearRight.getCurrentPosition());
        teleMetry.addData("Turn Table", turntable.getCurrentPosition());
        teleMetry.addData("Arm", arm.getCurrentPosition());
        teleMetry.addData("Intake Box Position", intakebox.getPosition());*/
        teleMetry.addData("DUCK Position", ducklocation);
        teleMetry.addData("Duck Right Angle", xPosition_Right);
        teleMetry.addData("Duck Left Angle", xPosition_Left);
        if (State.equals("INIT")){
            teleMetry.addData("GOOD LUCK ATOMIC TOADS, You may press play now", "HIT PLAY BUTTON");
        }
        else if (State.equals("PLAY")) {
            teleMetry.addData(", GO ATOMIC TODS, Watch Play Now", "HAVE FUN");
        }
        }

    /**
     * Describe this function...
     */
    private void ResetArmDown() {
        if (!armdowntouch.isPressed()) {
            arm.setTargetPosition(5000);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx) arm).setVelocity(ArmVelocity);
            while (!armdowntouch.isPressed()) {
                teleMetry.addData("Reset Arm Position", arm.getCurrentPosition());
            }
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            SetArmPosition(-100);
        }
    }
}