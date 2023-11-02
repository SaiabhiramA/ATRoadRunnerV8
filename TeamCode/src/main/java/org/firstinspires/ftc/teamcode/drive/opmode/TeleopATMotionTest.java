package org.firstinspires.ftc.teamcode.drive.opmode;

import static freightfrenzy.teamcode.drive.AtomicToadsDriveConstants.MAX_ACCEL;
import static freightfrenzy.teamcode.drive.AtomicToadsDriveConstants.MAX_ANG_VEL;
import static freightfrenzy.teamcode.drive.AtomicToadsDriveConstants.TRACK_WIDTH;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.ATGlobalStorage;
import org.firstinspires.ftc.teamcode.drive.ATRobotEnumeration;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveAT;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveATCancelableDW;
import org.firstinspires.ftc.teamcode.drive.TopHatMotionProfiling;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/**

 */
//@Config
@TeleOp(name = "TeleOp AT Motion Test")


public class TeleopATMotionTest extends LinearOpMode {
    //private DcMotorEx leftFront, leftRear, rightRear, rightFront,arm,elbow,turntable;
    private DcMotorEx arm,elbow;//,turntable;
    private Servo wrist, rightClaw,leftClaw;
    public static PIDFCoefficients armPIDFCoefficients;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    double ArmPosition = 0;
//    private PIDFController leftFrontController,leftRearController,rightRearController,rightFrontController,armController,elbowController,turntableController;

    private PIDFController armController;//,elbowController,turntableController;


    //private PIDCoefficients coeffs  = new PIDCoefficients(0,0,0);
    //private FtcDashboard atDashboards;

    @Override
    public void runOpMode() throws InterruptedException {
        initPIDController();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        this.updateMotionTelemetry();
        //dashboard.updateConfig();
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            //dashboard.updateConfig();
            this.runPlatform();
            armPIDFCoefficients=arm.getPIDFCoefficients(arm.getMode());
            this.updateMotionTelemetry();
        }

    }

    private void updateMotionTelemetry(){
        //telemetry.addData("wrist position", wrist.getPosition());
        telemetry.addData("Current Arm position", arm.getCurrentPosition());
        //telemetry.addData("elbow position", elbow.getCurrentPosition());
        //telemetry.addData("turntable position", turntable.getCurrentPosition());
        //telemetry.addData("rightClaw position", rightClaw.getPosition());
        //telemetry.addData("leftClaw position", leftClaw.getPosition());
        telemetry.addData("Target Arm Position:", ArmPosition);
        telemetry.addData("P", armPIDFCoefficients.p);
        telemetry.addData("I", armPIDFCoefficients.i);
        telemetry.addData("D", armPIDFCoefficients.d);
        telemetry.addData("F", armPIDFCoefficients.f);
        telemetry.update();
    }

    private void setArmPIDFCoefficients(){
        armPIDFCoefficients.p=MecanumDriveAT.proportional;
        armPIDFCoefficients.i=MecanumDriveAT.integral;
        armPIDFCoefficients.d=MecanumDriveAT.derivative;
        armPIDFCoefficients.f=MecanumDriveAT.feedforward;
        arm.setPIDFCoefficients(arm.getMode(),armPIDFCoefficients);
    }

    public void initPIDController(){
/*        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        leftRear = hardwareMap.get(DcMotorEx.class, "rearLeft");
        rightRear = hardwareMap.get(DcMotorEx.class, "rearRight");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");*/

        arm = hardwareMap.get(DcMotorEx.class, "arm");
        elbow = hardwareMap.get(DcMotorEx.class, "elbow");
        //turntable = hardwareMap.get(DcMotorEx.class, "turntable");
        wrist = hardwareMap.get(Servo.class, "wrist");
        rightClaw = hardwareMap.get(Servo.class, "rightclaw");
        leftClaw = hardwareMap.get(Servo.class, "leftclaw");

        ArmPosition=Math.min(Math.max(ArmPosition, 100), 2250);

/*        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //turntable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


/*        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/

        //arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //turntable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        /*leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);*/

        /*leftFrontController = new PIDFController(coeffs);
        leftRearController = new PIDFController(coeffs);
        rightRearController = new PIDFController(coeffs);
        rightFrontController = new PIDFController(coeffs);*/

        //armController = new PIDFController(coeffs);
        //turntableController = new PIDFController(coeffs);
        //elbowController = new PIDFController(coeffs);

        /*leftFrontController.setTargetPosition(target);
        leftRearController.setTargetPosition(target);
        rightRearController.setTargetPosition(target);
        rightFrontController.setTargetPosition(target);*/

        //armController.setTargetPosition(ArmPosition);
        //turntableController.setTargetPosition(target);
        //elbowController.setTargetPosition(target);

        arm.setTargetPosition((int)ArmPosition);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setVelocity(5000);

        armPIDFCoefficients= arm.getPIDFCoefficients(arm.getMode());
        MecanumDriveAT.proportional=armPIDFCoefficients.p;
        MecanumDriveAT.integral=armPIDFCoefficients.i;
        MecanumDriveAT.derivative=armPIDFCoefficients.d;
        MecanumDriveAT.feedforward=armPIDFCoefficients.f;
    }

    public void runPlatform(){

        ArmPosition=Math.min(Math.max(ArmPosition, 100), 2000);
        this.platformDriverActionsStatesChamp();
        this.motionProfileWheels();
    }

    private void motionProfileWheels(){
        /*leftFront.setPower(leftFrontController.update(leftFront.getCurrentPosition()));
        leftRear.setPower(leftRearController.update(leftRear.getCurrentPosition()));
        rightRear.setPower(rightRearController.update(rightRear.getCurrentPosition()));
        rightFront.setPower(rightFrontController.update(rightFront.getCurrentPosition()));*/

        //arm.setPower(armController.update(arm.getCurrentPosition()));


        //elbow.setPower(elbowController.update(elbow.getCurrentPosition()));
        //turntable.setPower(turntableController.update(turntable.getCurrentPosition()));

        arm.setTargetPosition((int)ArmPosition);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setVelocity(5000);


    }

    private void platformDriverActionsStatesChamp(){
        /**
         * This method would run all the time to ensure platform drive will not be drifting due to platform action condictions
         */
        if (gamepad1.left_stick_y!=0 || gamepad1.left_stick_x!=0 || gamepad1.right_stick_x!=0) {
        }

        /** perform auto navigation from auton position to teleop pickup position only for first time
         * enhance further later from any position in zone to move to substation pickup position
         */
        if (gamepad1.left_trigger>0 && gamepad1.right_trigger>0 && gamepad1.a){
            setArmPIDFCoefficients();
            ArmPosition=2500;
/*            leftFrontController.setTargetPosition(target);
            leftRearController.setTargetPosition(target);
            rightRearController.setTargetPosition(target);
            rightFrontController.setTargetPosition(target);*/

            //turntableController.setTargetPosition(target);
            //elbowController.setTargetPosition(target);

        }

        /**
         * Execute emergency stop during auto navigation
         */
        if (gamepad1.left_trigger>0 && gamepad1.right_trigger>0 && gamepad1.x){
        }

        /** Reset the robot manually when needed
         */
        if (gamepad1.right_bumper && gamepad1.left_bumper && gamepad1.x && gamepad1.left_trigger>0 && gamepad1.right_trigger>0) {
        }

        /**
         * This is to set the tophat in navigation position in the field
         */
        if (gamepad1.left_trigger>0 && gamepad1.right_trigger>0 && gamepad1.y){
        }

        /**
         * This is to turn robot in 180 degrees in counter clockwise  - RE ENABLE THIS LATER
         * */
        if (gamepad1.left_trigger>0 && gamepad1.right_trigger>0 && gamepad1.b){
            setArmPIDFCoefficients();
            ArmPosition=100;
            //arm.setVelocityPIDFCoefficients();
            //arm.setPIDFCoefficients();
            //arm.setPositionPIDFCoefficients();

/*            leftFrontController.setTargetPosition(target);
            leftRearController.setTargetPosition(target);
            rightRearController.setTargetPosition(target);
            rightFrontController.setTargetPosition(target);*/
            //turntableController.setTargetPosition(target);
            //elbowController.setTargetPosition(target);

        }

        /**
         * This is to preset TopHat for left ground junction drop         *
         */
        if (!gamepad1.right_bumper && gamepad1.left_bumper && gamepad1.x){
        }
        /**
         * This is to preset TopHat for right ground junction drop
         */
        if (gamepad1.right_bumper && !gamepad1.left_bumper && gamepad1.x){
        }

        /**
         * This is to preset TopHat for left low junction drop
         */
        if (!gamepad1.right_bumper && gamepad1.left_bumper && gamepad1.a){
        }
        /**
         * This is to preset TopHat for right low junction drop
         */
        if (gamepad1.right_bumper && !gamepad1.left_bumper && gamepad1.a){
        }

        /**
         * This is to preset TopHat for left medium junction drop
         */
        if (gamepad1.left_bumper && !gamepad1.right_bumper && gamepad1.b){
        }

        /**
         * This is to preset TopHat for right medium junction drop
         */
        if (gamepad1.right_bumper && !gamepad1.left_bumper && gamepad1.b){
        }


        /**
         * This is to preset TopHat for left high junction drop
         */
        if (!gamepad1.right_bumper && gamepad1.left_bumper && gamepad1.y){
        }
        /**
         * This is to preset TopHat for right high junction drop
         */
        if (gamepad1.right_bumper && !gamepad1.left_bumper && gamepad1.y){
        }

        /**
         * This is to preset TopHat for ground junction drop and also front pickup         *
         */
        if (gamepad1.right_bumper && gamepad1.left_bumper && gamepad1.x){
        }

        /**
         * This is to preset TopHat for front low junction drop
         */
        if (gamepad1.right_bumper && gamepad1.left_bumper && gamepad1.a){
        }
        /**
         * This is to preset TopHat for front medium junction drop
         */
        if (gamepad1.left_bumper && gamepad1.right_bumper && gamepad1.b){
        }

        /**
         * This is to preset TopHat for front high junction drop
         */
        if (gamepad1.right_bumper && gamepad1.left_bumper && gamepad1.y){
        }
    }
}
