package org.firstinspires.ftc.teamcode.drive.opmode;

import static freightfrenzy.teamcode.drive.AtomicToadsDriveConstants.MAX_ACCEL;
import static freightfrenzy.teamcode.drive.AtomicToadsDriveConstants.MAX_ANG_VEL;
import static freightfrenzy.teamcode.drive.AtomicToadsDriveConstants.TRACK_WIDTH;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.ATGlobalStorage;
import org.firstinspires.ftc.teamcode.drive.ATRobotEnumeration;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveATCancelable;
import org.firstinspires.ftc.teamcode.drive.TopHatAutoController;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
//@TeleOp(group = "drive")
@TeleOp(name = "ATTeleOpMode-GameDay")
public class TeleopATGameDay extends LinearOpMode {
    TopHatAutoController tophatController;
    MecanumDriveATCancelable drive;
    ATRobotEnumeration robotMode= ATRobotEnumeration.TELE_OP_AUTO;
    Pose2d poseEstimate;
    TrajectorySequence trajSeqPark1;
    TrajectorySequence trajSeqPark2;
    TrajectorySequence trajSeqPark3;
    boolean teleOpConePickupPositioned=false;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDriveATCancelable(hardwareMap);
        tophatController = new TopHatAutoController();
        tophatController.basicInitializeRobot(hardwareMap,telemetry,gamepad1,gamepad2,robotMode);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.update();
        drive.setPoseEstimate(ATGlobalStorage.currentPose);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.addData("Platform robot mode", robotMode);
            telemetry.addData("AutoMode Name", ATGlobalStorage.autonModeName);
            telemetry.addData("Parking Zone", ATGlobalStorage.parkingPos);
            telemetry.update();
            this.runPlatform();
            tophatController.runTopHat();
        }

    }
    public void runPlatform(){
        /** Reset the robot manually when needed
         */
        if (gamepad1.right_bumper && gamepad1.left_bumper && gamepad1.x && gamepad1.left_trigger>0 && gamepad1.right_trigger>0) {
            robotMode= ATRobotEnumeration.RESET;
            tophatController.ResetTopHat();
        }
        /** In manual mode only set the drive platform coordinates based on left/right stick x/y positions
         */
        if (gamepad1.left_stick_y!= 0 || gamepad1.left_stick_x != 0 || gamepad1.right_stick_x !=0){
            robotMode= ATRobotEnumeration.MANUAL;
        }
        /** In manual mode only set the drive platform coordinates based on left/right stick x/y positions
         */
        if (robotMode== ATRobotEnumeration.MANUAL) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -0.5*gamepad1.left_stick_y,
                            -0.5*gamepad1.left_stick_x,
                            -0.5*gamepad1.right_stick_x
                    )
            );
        }
        /** perform auto navigation from auton position to teleop pickup position only for first time
         * enhance further later from any position in zone to move to substation pickup position
        */
        if (gamepad1.left_bumper && gamepad1.right_bumper && gamepad1.a){
            robotMode= ATRobotEnumeration.TELE_OP_AUTO;
            navigateToPickupInSubstation();
        }
        /**
         * Execute emergency stop during auto navigation
         */
        if (gamepad1.left_bumper && gamepad1.right_bumper && gamepad1.x){
            robotMode= ATRobotEnumeration.MANUAL;
            drive.breakFollowing();
            tophatController.stopTopHatMovement();
        }
        /**
         * This is to resume high junction drop off remaining cones from Auton and completed during first 15 sec of
         * Tele Op Mode
         */
        if (gamepad1.left_bumper && gamepad1.right_bumper && gamepad1.y){
            robotMode= ATRobotEnumeration.TELE_OP_AUTO;
        }
        /**
         * This is to resume medium junction drop off remaining cones from Auton and completed during first 15 sec of
         * Tele Op Mode
         */
        if (gamepad1.left_bumper && gamepad1.right_bumper && gamepad1.b){
            robotMode= ATRobotEnumeration.TELE_OP_AUTO;
        }

        /**
         * This is to set the tophat in navigation position in the field
         */
        if (gamepad1.left_trigger>0 && gamepad1.right_trigger>0 && gamepad1.a){
            tophatController.setRobotMode(ATRobotEnumeration.AUTO_TOPHAT_ONEWAY_MOVE_BEGIN);
            tophatController.setTopHatPosition(.1,false,4300,-1800,945);
        }
    }
    /**
     * This method needs to be enhanced to make it more dynamic based on either RED or BLUE alliance
     */
    private void navigateToPickupInSubstation(){
        if (!teleOpConePickupPositioned) {
            teleOpConePickupPositioned = true;
            if (ATGlobalStorage.parkingPos == ATRobotEnumeration.PARK1) {
                setTrajectorySequenceForPark1(ATGlobalStorage.autonModeName);
                drive.followTrajectorySequenceAsync(trajSeqPark1);
            } else if (ATGlobalStorage.parkingPos == ATRobotEnumeration.PARK2) {
                setTrajectorySequenceForPark2(ATGlobalStorage.autonModeName);
                drive.followTrajectorySequenceAsync(trajSeqPark2);
            } else if (ATGlobalStorage.parkingPos == ATRobotEnumeration.PARK3) {
                setTrajectorySequenceForPark3(ATGlobalStorage.autonModeName);
                drive.followTrajectorySequenceAsync(trajSeqPark3);
            }
        }
     }
     private void setTrajectorySequenceForPark1(ATRobotEnumeration alliance){
        switch (alliance){
            case RED_RIGHT_HIGH_DROP:{
                trajSeqPark1=drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(13, -5, Math.toRadians(90)))
                        .lineToSplineHeading(new Pose2d(12, -32, Math.toRadians(180)),drive.getVelocityConstraint(35, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                        //.splineToSplineHeading(new Pose2d(12, -32, Math.toRadians(180)),Math.toRadians(110))
                        .build();
            }
            break;
        }
     }

    private void setTrajectorySequenceForPark2(ATRobotEnumeration alliance){
        switch (alliance){
            case RED_RIGHT_HIGH_DROP:{
                trajSeqPark2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(36, -5, Math.toRadians(90)))
                        .lineToSplineHeading(new Pose2d(36, -28, Math.toRadians(180)),drive.getVelocityConstraint(35, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                        .lineToLinearHeading(new Pose2d(12, -32, Math.toRadians(180)))
                        /*.splineToLinearHeading(new Pose2d(36, -28, Math.toRadians(90)),Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(12, -32, Math.toRadians(180)),Math.toRadians(110))*/
                        .build();
            }
            break;
        }
    }
    private void setTrajectorySequenceForPark3(ATRobotEnumeration alliance){
        switch (alliance){
            case RED_RIGHT_HIGH_DROP:{
                trajSeqPark3 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(56, -5, Math.toRadians(90)))
                        .lineToSplineHeading(new Pose2d(60, -28, Math.toRadians(180)),drive.getVelocityConstraint(35, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                        .lineToLinearHeading(new Pose2d(12, -32, Math.toRadians(180)))
/*                        .splineToLinearHeading(new Pose2d(56, -5, Math.toRadians(90)),Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(36, -28, Math.toRadians(90)),Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(12, -32, Math.toRadians(180)),Math.toRadians(110))*/
                        .build();
            }
            break;
        }
    }
}
