package org.firstinspires.ftc.teamcode.drive.opmode;

import static freightfrenzy.teamcode.drive.AtomicToadsDriveConstants.MAX_ACCEL;
import static freightfrenzy.teamcode.drive.AtomicToadsDriveConstants.MAX_ANG_VEL;
import static freightfrenzy.teamcode.drive.AtomicToadsDriveConstants.TRACK_WIDTH;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.ATConstants;
import org.firstinspires.ftc.teamcode.drive.ATGlobalStorage;
import org.firstinspires.ftc.teamcode.drive.ATRobotEnumeration;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveAT;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveATWheels;
import org.firstinspires.ftc.teamcode.drive.TopHatAutoControllerStates;
import org.firstinspires.ftc.teamcode.drive.TopHatAutoControllerStatesV2;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
//@Autonomous(group = "drive")
@Autonomous(name = "RedLeftHighDropAT-DeadWheel")
public class RedAllianceLeftHighDropDW extends LinearOpMode {
    MecanumDriveATWheels drive;
    TopHatAutoControllerStatesV2 tophatController;
    ATAprilTag ATObjectDetection;
    ATRobotEnumeration parkingZone=ATRobotEnumeration.SUBSTATION;
    Pose2d poseEstimate;
    boolean isAutonConePickupReady=true;
    @Override
    public void runOpMode() throws InterruptedException {
        tophatController=new TopHatAutoControllerStatesV2();
        tophatController.robotMode=ATRobotEnumeration.FULL_AUTON;
        ATObjectDetection = new ATAprilTag();
        ATObjectDetection.initalizeTensorFlow(hardwareMap, telemetry, ATRobotEnumeration.AUTO_RED_LEFT_HIGH_SETUP);
        //parkingZone = ATObjectDetection.detectObjectLabel();
        tophatController.fullyInitializeRobot(telemetry, gamepad1, gamepad2, ATRobotEnumeration.RESET, hardwareMap);
        tophatController.setTopHatSpeed(ATRobotEnumeration.TOPHAT_LOW_AUTON_SPEED);
        drive = new MecanumDriveATWheels(hardwareMap);
        Pose2d startPose = new Pose2d(-40, -60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        //parkingZone=ATRobotEnumeration.SUBSTATION;
        setRobotStateInStorage();
        TrajectorySequence trajSeqConePickup = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(.001, ()->{
                    tophatController.setTophatAction(ATRobotEnumeration.SET_RED_LEFT_PRELOADED_CONE);
                    tophatController.redAllianceLeftAutonHigh();})
                .splineToConstantHeading(new Vector2d(-36,-50), Math.toRadians(90),drive.getVelocityConstraint(20, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                .addTemporalMarker(1, ()->{
                    if (tophatController.tophatAction==ATRobotEnumeration.SET_RED_LEFT_PRELOADED_CONE) {
                        tophatController.redAllianceLeftAutonHigh();
                    }
                })
                .splineToConstantHeading(new Vector2d(-32,-18), Math.toRadians(90),drive.getVelocityConstraint(20, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                .addTemporalMarker(2, ()->{
                    if (tophatController.tophatAction==ATRobotEnumeration.SET_RED_LEFT_PRELOADED_CONE) {
                        tophatController.redAllianceLeftAutonHigh();
                    }
                })
                .addTemporalMarker(2.5, ()->{
                    if (tophatController.tophatAction==ATRobotEnumeration.SET_RED_LEFT_PRELOADED_CONE) {
                        tophatController.redAllianceLeftAutonHigh();
                    }
                })
                .addTemporalMarker(3.5, ()->{
                    tophatController.setTophatAction(ATRobotEnumeration.DROP_RED_LEFT_PRELOADED_CONE);
                    tophatController.redAllianceLeftAutonHigh();})
                .waitSeconds(2)
                .splineToConstantHeading(new Vector2d(-39.5,-11), Math.toRadians(0),drive.getVelocityConstraint(20, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                //.lineToConstantHeading(new Vector2d(-39.5,-11),drive.getVelocityConstraint(20, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                .build();

        while (opModeInInit()) {
            parkingZone = ATObjectDetection.detectObjectLabel();
            telemetry.addData("Parking Zone", parkingZone);
            telemetry.addData("Get Runtime", this.getRuntime());
            if (this.getRuntime()>10){
                telemetry.addData("ATOMIC TOADS, You may Now PRESS PLAY - WISH YOU GOOD LUCK", this.getRuntime());
            }
            telemetry.update();
        }
        waitForStart();
        if (isStopRequested()) return;
        drive.followTrajectorySequence(trajSeqConePickup);
        tophatController.setTopHatSpeed(ATRobotEnumeration.TOPHAT_MEDIUM_SPEED);
        if (isAutonConePickupReady) {
            tophatController.setTophatAction(ATRobotEnumeration.AUTO_RED_LEFT_HIGH_PICK_CONE);
            while ((!isStopRequested()) && !tophatController.areFiveConesDone()) {
                tophatController.redAllianceLeftAutonHigh();
                poseEstimate = drive.getPoseEstimate();
                telemetry.addData("x", poseEstimate.getX());
                telemetry.addData("y", poseEstimate.getY());
                telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
                telemetry.addData("robot mode", tophatController.getTophatAction());
                telemetry.addData("Parking Zone", parkingZone);
                telemetry.addData("Get Runtime", this.getRuntime());
                telemetry.addData("In While Loop", "YES");
                telemetry.addData("Stop  Requested", isStopRequested());
                telemetry.addData("are FiveCones Done", tophatController.areFiveConesDone());
                telemetry.update();
            }
        }
        else {
            poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.addData("robot mode", tophatController.getTophatAction());
            telemetry.addData("In While Loop", "YES");
            telemetry.update();
            tophatController.setTophatAction(ATRobotEnumeration.AUTO_RED_LEFT_HIGH_PARK);
        }
        tophatController.parkingTurnTablePosition=250;
        tophatController.setTopHatSpeed(ATRobotEnumeration.TOPHAT_HIGH_SPEED);

        TrajectorySequence trajSeqParking;
        if (parkingZone==ATRobotEnumeration.PARK1){
            trajSeqParking=drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToConstantHeading(new Vector2d(-60,-11))//,drive.getVelocityConstraint(45, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                    .addTemporalMarker(.0001, ()->{
                        if (!tophatController.isTopHatInParkingPosition()){
                            telemetry.addData("Top Hat is in Parking Mode", "YES");
                        }
                    })
                    .addTemporalMarker(1, ()->{
                        if (!tophatController.isTopHatInParkingPosition()){
                            telemetry.addData("Top Hat is in Parking Mode", "YES");
                        }
                    })

                    .build();
            drive.followTrajectorySequence(trajSeqParking);
        }
        else if (parkingZone==ATRobotEnumeration.PARK2){
            trajSeqParking=drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToConstantHeading(new Vector2d(-36,-11))//,drive.getVelocityConstraint(45, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                    .addTemporalMarker(.0001, ()->{
                        if (!tophatController.isTopHatInParkingPosition()){
                            telemetry.addData("Top Hat is in Parking Mode", "YES");
                        }
                    })
                    .addTemporalMarker(1, ()->{
                        if (!tophatController.isTopHatInParkingPosition()){
                            telemetry.addData("Top Hat is in Parking Mode", "YES");
                        }
                    })

                    .build();
            drive.followTrajectorySequence(trajSeqParking);
        }
        else if (parkingZone==ATRobotEnumeration.PARK3){
            trajSeqParking=drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToConstantHeading(new Vector2d(-12, -11))//,drive.getVelocityConstraint(45, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                    .addTemporalMarker(.0001, ()->{
                        if (!tophatController.isTopHatInParkingPosition()){
                            telemetry.addData("Top Hat is in Parking Mode", "YES");
                        }
                    })
                    .addTemporalMarker(1, ()->{
                        if (!tophatController.isTopHatInParkingPosition()){
                            telemetry.addData("Top Hat is in Parking Mode", "YES");
                        }
                    })

                    .build();
            drive.followTrajectorySequence(trajSeqParking);
        }
        else if (parkingZone==ATRobotEnumeration.SUBSTATION){
            trajSeqParking=drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToConstantHeading(new Vector2d(-12, -11))//,drive.getVelocityConstraint(45, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                    .addTemporalMarker(.0001, ()->{
                        if (!tophatController.isTopHatInParkingPosition()){
                            telemetry.addData("Top Hat is in Parking Mode", "YES");
                        }
                    })
                    .addTemporalMarker(1, ()->{
                        if (!tophatController.isTopHatInParkingPosition()){
                            telemetry.addData("Top Hat is in Parking Mode", "YES");
                        }
                    })

                    .lineToSplineHeading(new Pose2d(-12, -60, Math.toRadians(0)))//,drive.getVelocityConstraint(45, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                    .build();
            drive.followTrajectorySequence(trajSeqParking);
        }
        if (tophatController.tophatAction==ATRobotEnumeration.AUTO_RED_LEFT_HIGH_PARK){
            tophatController.parkingTurnTablePosition=250;
            tophatController.setTopHatSpeed(ATRobotEnumeration.TOPHAT_HIGH_SPEED);
            while ((!isStopRequested()) && !tophatController.isTopHatInParkingPosition()){
                telemetry.addData("Top Hat is in Parking Mode", "YES");
            }
        }

        setRobotStateInStorage();

        poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
        telemetry.addData("robot mode", tophatController.getTophatAction());
        telemetry.addData("Parking Zone", parkingZone);
        telemetry.addData("Get Runtime", this.getRuntime());
        telemetry.update();
    }
    private void setRobotStateInStorage(){
        ATGlobalStorage.autonModeName=ATRobotEnumeration.RED_LEFT_HIGH_DROP;
        ATGlobalStorage.currentPose=drive.getPoseEstimate();
        ATGlobalStorage.parkingPos=parkingZone;
        ATGlobalStorage.allianceName=ATRobotEnumeration.RED_ALLIANCE;
    }
}
