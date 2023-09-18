package org.firstinspires.ftc.teamcode.drive.opmode;

import static freightfrenzy.teamcode.drive.AtomicToadsDriveConstants.MAX_ACCEL;
import static freightfrenzy.teamcode.drive.AtomicToadsDriveConstants.MAX_ANG_VEL;
import static freightfrenzy.teamcode.drive.AtomicToadsDriveConstants.TRACK_WIDTH;

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

import org.firstinspires.ftc.teamcode.drive.ATGlobalStorage;
import org.firstinspires.ftc.teamcode.drive.ATRobotEnumeration;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveATCancelableDW;
import org.firstinspires.ftc.teamcode.drive.TopHatMotionProfiling;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/**

 */
@TeleOp(name = "TeleOp AT Motion Test")

public class TeleopATMotionTest extends LinearOpMode {
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;

    double target = 0;
    private PIDFController leftFrontController,leftRearController,rightRearController,rightFrontController;

    private PIDCoefficients coeffs  = new PIDCoefficients(10,0,0);

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.update();
        initPIDController();
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            this.runPlatform();
            telemetry.update();
        }

    }

    public void initPIDController(){
        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        leftRear = hardwareMap.get(DcMotorEx.class, "rearLeft");
        rightRear = hardwareMap.get(DcMotorEx.class, "rearRight");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontController = new PIDFController(coeffs);
        leftRearController = new PIDFController(coeffs);
        rightRearController = new PIDFController(coeffs);
        rightFrontController = new PIDFController(coeffs);

        leftFrontController.setTargetPosition(target);
        leftRearController.setTargetPosition(target);
        rightRearController.setTargetPosition(target);
        rightFrontController.setTargetPosition(target);


    }

    public void runPlatform(){

        this.platformDriverActionsStatesChamp();
        this.motionProfileWheels();
    }

    private void motionProfileWheels(){
        leftFront.setPower(leftFrontController.update(leftFront.getCurrentPosition()));
        leftRear.setPower(leftRearController.update(leftRear.getCurrentPosition()));
        rightRear.setPower(rightRearController.update(rightRear.getCurrentPosition()));
        rightFront.setPower(rightFrontController.update(rightFront.getCurrentPosition()));
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
            target=3000;
            leftFrontController.setTargetPosition(target);
            leftRearController.setTargetPosition(target);
            rightRearController.setTargetPosition(target);
            rightFrontController.setTargetPosition(target);
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
            target=0;
            leftFrontController.setTargetPosition(target);
            leftRearController.setTargetPosition(target);
            rightRearController.setTargetPosition(target);
            rightFrontController.setTargetPosition(target);
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
