package org.firstinspires.ftc.teamcode.drive.opmode;

import static freightfrenzy.teamcode.drive.AtomicToadsDriveConstants.MAX_ACCEL;
import static freightfrenzy.teamcode.drive.AtomicToadsDriveConstants.MAX_ANG_VEL;
import static freightfrenzy.teamcode.drive.AtomicToadsDriveConstants.TRACK_WIDTH;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.ATGlobalStorage;
import org.firstinspires.ftc.teamcode.drive.ATRobotEnumeration;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveATCancelableDW;
import org.firstinspires.ftc.teamcode.drive.TopHatAutoControllerStates;
import org.firstinspires.ftc.teamcode.drive.TopHatAutoControllerStatesV2;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/**

 */
//@TeleOp(group = "drive")
@TeleOp(name = "ATTeleOp-GameDayStates-V2**")
public class TeleopATGameDayStatesDWV2 extends LinearOpMode {
    TopHatAutoControllerStatesV2 tophatController;
    MecanumDriveATCancelableDW drive;
    ATRobotEnumeration platformAction;
    ATRobotEnumeration platformMode;
    Pose2d poseEstimate;
    TrajectorySequence trajSeqPark1;
    TrajectorySequence trajSeqPark2;
    TrajectorySequence trajSeqPark3;
    TrajectorySequence trajSeqSubstation;
    boolean teleOpConePickupPositioned=false;
    double robotXpos;
    double robotYpos;
    double robotHeading;
    double elapsedTime;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDriveATCancelableDW(hardwareMap);
        tophatController = new TopHatAutoControllerStatesV2();
        platformMode = ATRobotEnumeration.AUTO;
        platformAction = ATRobotEnumeration.TELE_OP_AUTO;
        tophatController.basicInitializeRobot(hardwareMap,telemetry,gamepad1,gamepad2, platformAction);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.update();
        drive.setPoseEstimate(ATGlobalStorage.currentPose);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            this.runPlatform();
            drive.update();
            if (tophatController.sleepMode==ATRobotEnumeration.SLEEP_MODE_REQUEST){
                elapsedTime=getRuntime()+tophatController.sleepMSecReq;
                tophatController.sleepMode=ATRobotEnumeration.SLEEP_MODE_ON;
            }
            if (elapsedTime<getRuntime()){
                tophatController.sleepMode=ATRobotEnumeration.SLEEP_MODE_OFF;
            }
            tophatController.runTopHat();
            poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.addData("Platform robot mode", platformAction);
            telemetry.addData("AutoMode Name", ATGlobalStorage.autonModeName);
            telemetry.addData("Parking Zone", ATGlobalStorage.parkingPos);
            telemetry.update();
        }

    }

    public void runPlatform(){

        //this.platformDriverActionsNovi();

        this.platformDriverActionsStatesChamp();


    }
    /**
     * This method is to move the robot to substation pickup position
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
            else if (ATGlobalStorage.parkingPos == ATRobotEnumeration.SUBSTATION) {
                setTrajectorySequenceForSubstation(ATGlobalStorage.autonModeName);
                drive.followTrajectorySequenceAsync(trajSeqSubstation);
            }
        }
        else{
                robotXpos=drive.getPoseEstimate().getX();
                robotYpos=drive.getPoseEstimate().getY();
                robotHeading=drive.getPoseEstimate().getHeading();
                telemetry.addData("x", poseEstimate.getX());
                telemetry.addData("y", poseEstimate.getY());
                telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
                telemetry.addData("robot mode", tophatController.getTophatAction());
                telemetry.addData("Alliance Name", ATGlobalStorage.allianceName);

                if (ATGlobalStorage.autonModeName==ATRobotEnumeration.BLUE_LEFT_HIGH_DROP) {
                    if (Math.abs(robotYpos)>27 &&Math.abs(robotYpos) <39) {
                        trajSeqSubstation = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(12, 39, Math.toRadians(180)), drive.getVelocityConstraint(35, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                                .build();
                        drive.followTrajectorySequenceAsync(trajSeqSubstation);
                    }
                }
                else if (ATGlobalStorage.autonModeName==ATRobotEnumeration.BLUE_RIGHT_HIGH_DROP){
                    if (Math.abs(robotYpos)>40 &&Math.abs(robotYpos) <50) {
                        trajSeqSubstation = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(-12, 38, Math.toRadians(0)), drive.getVelocityConstraint(35, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                                .build();
                        drive.followTrajectorySequenceAsync(trajSeqSubstation);
                    }
                }
                else if (ATGlobalStorage.autonModeName==ATRobotEnumeration.RED_LEFT_HIGH_DROP) {
                    if (Math.abs(robotYpos)>36 &&Math.abs(robotYpos) <46) {
                        trajSeqSubstation = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(-12, -38, Math.toRadians(0)), drive.getVelocityConstraint(35, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                                .build();
                        drive.followTrajectorySequenceAsync(trajSeqSubstation);
                    }
                }
                else if (ATGlobalStorage.autonModeName==ATRobotEnumeration.RED_RIGHT_HIGH_DROP){
                    if (Math.abs(robotYpos)>39 &&Math.abs(robotYpos) <49) {
                        trajSeqSubstation = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(14, -40, Math.toRadians(180)), drive.getVelocityConstraint(35, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                                .build();
                        drive.followTrajectorySequenceAsync(trajSeqSubstation);
                    }
                }
            }
     }
     private void setTrajectorySequenceForPark1(ATRobotEnumeration alliance){
        switch (alliance){
            case RED_RIGHT_HIGH_DROP:{
                trajSeqPark1=drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(14, -12, Math.toRadians(90)))
                        .lineToSplineHeading(new Pose2d(14, -40, Math.toRadians(180)),drive.getVelocityConstraint(35, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                        .build();
            }
            break;
            case RED_LEFT_HIGH_DROP:{
                trajSeqPark1=drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToConstantHeading(new Vector2d(-12,-11))
                        .lineToLinearHeading(new Pose2d(-12,-40,Math.toRadians(0)),drive.getVelocityConstraint(35, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                        .build();
            }
            break;
            case BLUE_LEFT_HIGH_DROP:{
                trajSeqPark1=drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToConstantHeading(new Vector2d(12, 11))
                        .lineToSplineHeading(new Pose2d(12, 39, Math.toRadians(180)),drive.getVelocityConstraint(35, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                        .build();
            }
            break;
            case BLUE_RIGHT_HIGH_DROP:{
                trajSeqPark1=drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToConstantHeading(new Vector2d(-12, 14))
                        .lineToLinearHeading(new Pose2d(-12,42,Math.toRadians(0)),drive.getVelocityConstraint(35, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                        .lineToConstantHeading(new Vector2d(-15, 42))
                        .build();
            }
            break;
        }
     }

    private void setTrajectorySequenceForPark2(ATRobotEnumeration alliance){
        switch (alliance){
            case RED_RIGHT_HIGH_DROP:{
                trajSeqPark2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(14, -12, Math.toRadians(90)))
                        .lineToSplineHeading(new Pose2d(14, -40, Math.toRadians(180)),drive.getVelocityConstraint(35, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                        .build();
            }
            break;
            case RED_LEFT_HIGH_DROP:{
                trajSeqPark2=drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToConstantHeading(new Vector2d(-12,-11))
                        .lineToLinearHeading(new Pose2d(-12,-40,Math.toRadians(0)),drive.getVelocityConstraint(35, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                        .build();
            }
            break;
            case BLUE_LEFT_HIGH_DROP:{
                trajSeqPark2=drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToConstantHeading(new Vector2d(12, 11))
                        .lineToSplineHeading(new Pose2d(12, 39, Math.toRadians(180)),drive.getVelocityConstraint(35, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                        .build();
            }
            break;
            case BLUE_RIGHT_HIGH_DROP:{
                trajSeqPark2=drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToConstantHeading(new Vector2d(-12, 14))
                        .lineToLinearHeading(new Pose2d(-12,42,Math.toRadians(0)),drive.getVelocityConstraint(35, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                        .lineToConstantHeading(new Vector2d(-15, 42))
                        .build();
            }
            break;
        }
    }
    private void setTrajectorySequenceForPark3(ATRobotEnumeration alliance){
        switch (alliance){
            case RED_RIGHT_HIGH_DROP:{
                trajSeqPark3 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(14, -12, Math.toRadians(90)))
                        .lineToSplineHeading(new Pose2d(14, -40, Math.toRadians(180)),drive.getVelocityConstraint(35, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                        .build();
            }
            break;
            case RED_LEFT_HIGH_DROP:{
                trajSeqPark3=drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToConstantHeading(new Vector2d(-12,-11))
                        .lineToLinearHeading(new Pose2d(-12,-40,Math.toRadians(0)),drive.getVelocityConstraint(35, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                        .build();
            }
            break;
            case BLUE_LEFT_HIGH_DROP:{
                trajSeqPark3=drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToConstantHeading(new Vector2d(12, 11))
                        .lineToSplineHeading(new Pose2d(12, 39, Math.toRadians(180)),drive.getVelocityConstraint(35, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))

                        .build();
            }
            break;
            case BLUE_RIGHT_HIGH_DROP:{
                trajSeqPark3=drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToConstantHeading(new Vector2d(-12, 14))
                        .lineToLinearHeading(new Pose2d(-12,42,Math.toRadians(0)),drive.getVelocityConstraint(35, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                        .lineToConstantHeading(new Vector2d(-15, 42))
                        .build();
            }
            break;
        }
    }
    private void setTrajectorySequenceForSubstation(ATRobotEnumeration alliance){
        switch (alliance){
            case RED_RIGHT_HIGH_DROP:{
                trajSeqSubstation = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(14, -40, Math.toRadians(180)))
                        .build();
            }
            break;
            case RED_LEFT_HIGH_DROP:{
                trajSeqSubstation = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-12,-40,Math.toRadians(0)),drive.getVelocityConstraint(35, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                        .build();
            }
            break;
            case BLUE_LEFT_HIGH_DROP:{
                trajSeqSubstation=drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                          .lineToSplineHeading(new Pose2d(12, 39, Math.toRadians(180)),drive.getVelocityConstraint(35, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                      .build();
            }
            break;
            case BLUE_RIGHT_HIGH_DROP:{
                trajSeqSubstation=drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-15,42,Math.toRadians(0)),drive.getVelocityConstraint(35, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                        .build();
            }
            break;
        }
    }

    /**
     * This method is only used when Gameday states to be debugged without Auton Program
     */
    public void mockupAutonExecution(){
        ATGlobalStorage.autonModeName=ATRobotEnumeration.BLUE_LEFT_HIGH_DROP;
        ATGlobalStorage.currentPose=drive.getPoseEstimate();
        ATGlobalStorage.parkingPos=ATRobotEnumeration.PARK2;
        ATGlobalStorage.allianceName=ATRobotEnumeration.BLUE_ALLIANCE;
        tophatController.fullyInitializeRobot(telemetry, gamepad1, gamepad2, platformAction, hardwareMap);
    }

    private void platformDriverActionsNovi(){
        /**
         * This method would run all the time to ensure platform drive will not be drifting due to platform action condictions
         */
        if (gamepad1.left_stick_y!=0 || gamepad1.left_stick_x!=0 || gamepad1.right_stick_x!=0) {
            platformAction = ATRobotEnumeration.PLATFORM_DRIVE;
            platformMode = ATRobotEnumeration.MANUAL;
        }
        if (platformAction==ATRobotEnumeration.PLATFORM_DRIVE){
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * 0.8,
                            -gamepad1.left_stick_x * 0.5,
                            -gamepad1.right_stick_x * 0.5
                    )
            );
        }
        /** Reset the robot manually when needed
         */
        if (gamepad1.right_bumper && gamepad1.left_bumper && gamepad1.x && gamepad1.left_trigger>0 && gamepad1.right_trigger>0) {
            platformAction = ATRobotEnumeration.RESET;
            platformMode = ATRobotEnumeration.AUTO;
            tophatController.ResetTopHat();
        }

        /** perform auto navigation from auton position to teleop pickup position only for first time
         * enhance further later from any position in zone to move to substation pickup position
         */
        if (gamepad1.left_bumper && gamepad1.right_bumper && gamepad1.a){
            platformAction = ATRobotEnumeration.SUBSTATION_PICKUP_POS;
            platformMode = ATRobotEnumeration.AUTO;
            navigateToPickupInSubstation();
        }
        /**
         * Execute emergency stop during auto navigation
         */
        if (gamepad1.left_bumper && gamepad1.right_bumper && gamepad1.x){
            platformAction = ATRobotEnumeration.KILL_ALL_ACTIONS;
            platformMode = ATRobotEnumeration.AUTO;
            drive.breakFollowing();
            tophatController.stopTopHatMovement();
        }

        /**
         * This is to turn robot in 180 degrees in counter clockwise
         * */
        if (gamepad1.left_bumper && gamepad1.right_bumper && gamepad1.b){
            platformAction = ATRobotEnumeration.TURN_PLATFORM_180;
            platformMode = ATRobotEnumeration.AUTO;
            TrajectorySequence trajSeqMedSubstation = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .turn(Math.toRadians(180))
                    .build();
            drive.followTrajectorySequenceAsync(trajSeqMedSubstation);
        }

        /**
         * This is to increase the tophat motors speed
         */
        if (gamepad1.left_trigger > 0 && gamepad1.right_trigger > 0 && gamepad1.dpad_up){
            platformAction = ATRobotEnumeration.SPEED_UP;
            platformMode = ATRobotEnumeration.MANUAL;
            tophatController.setTopHatMotorsVelocity(2000,3000,3000);
        }

        /**
         * This is to decrease the tophat motors speed
         */

        if (gamepad1.left_trigger > 0 && gamepad1.right_trigger > 0 && gamepad1.dpad_down){
            platformAction = ATRobotEnumeration.SPEED_DOWN;
            platformMode = ATRobotEnumeration.MANUAL;
            tophatController.setTopHatMotorsVelocity(1000,1000,1000);
        }

        /**
         * This is to set the tophat in navigation position in the field
         */
        if (gamepad1.left_trigger>0 && gamepad1.right_trigger>0 && gamepad1.a){
            tophatController.setTophatAction(ATRobotEnumeration.AUTO_TOPHAT_ONEWAY_MOVE_BEGIN);
            tophatController.setTophatMode(ATRobotEnumeration.AUTO);
            tophatController.setTopHatPosition(.1,false,4300*tophatController.armMultiplier,-1800*tophatController.elbowMultiplier,945);
        }
    }

    private void platformDriverActionsStatesChamp(){
        /**
         * This method would run all the time to ensure platform drive will not be drifting due to platform action condictions
         */
        if (gamepad1.left_stick_y!=0 || gamepad1.left_stick_x!=0 || gamepad1.right_stick_x!=0) {
            platformAction = ATRobotEnumeration.PLATFORM_DRIVE;
            platformMode = ATRobotEnumeration.MANUAL;
        }
        if (platformAction==ATRobotEnumeration.PLATFORM_DRIVE){
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * 0.8,
                            -gamepad1.left_stick_x * 0.5,
                            -gamepad1.right_stick_x * 0.5
                    )
            );
        }

        /** perform auto navigation from auton position to teleop pickup position only for first time
         * enhance further later from any position in zone to move to substation pickup position
         */
        if (gamepad1.left_trigger>0 && gamepad1.right_trigger>0 && gamepad1.a){
            //platformAction = ATRobotEnumeration.SUBSTATION_PICKUP_POS;
            //platformMode = ATRobotEnumeration.AUTO;
            //navigateToPickupInSubstation();
        }

        /**
         * Execute emergency stop during auto navigation
         */
        if (gamepad1.left_trigger>0 && gamepad1.right_trigger>0 && gamepad1.x){
            platformAction = ATRobotEnumeration.KILL_ALL_ACTIONS;
            platformMode = ATRobotEnumeration.AUTO;
            drive.breakFollowing();
            tophatController.stopTopHatMovement();
        }

        /** Reset the robot manually when needed
         */
        if (gamepad1.right_bumper && gamepad1.left_bumper && gamepad1.x && gamepad1.left_trigger>0 && gamepad1.right_trigger>0) {
            platformAction = ATRobotEnumeration.RESET;
            platformMode = ATRobotEnumeration.AUTO;
            tophatController.ResetTopHat();
        }

        /**
         * This is to set the tophat in navigation position in the field
         */
        if (gamepad1.left_trigger>0 && gamepad1.right_trigger>0 && gamepad1.y){
            tophatController.setTophatAction(ATRobotEnumeration.AUTO_TOPHAT_ONEWAY_MOVE_BEGIN);
            tophatController.setTophatMode(ATRobotEnumeration.AUTO);
            tophatController.setTopHatPosition(.1,false,4300*tophatController.armMultiplier,-1800*tophatController.elbowMultiplier,tophatController.frontPickTTPos);
        }

        /**
         * This is to turn robot in 180 degrees in counter clockwise  - RE ENABLE THIS LATER
         * */
        if (gamepad1.left_trigger>0 && gamepad1.right_trigger>0 && gamepad1.b){
            /*platformAction = ATRobotEnumeration.TURN_PLATFORM_180;
            platformMode = ATRobotEnumeration.AUTO;
            TrajectorySequence trajSeqMedSubstation = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .turn(Math.toRadians(180))
                    .build();
            drive.followTrajectorySequenceAsync(trajSeqMedSubstation);*/
        }

        /**
         * This is to preset TopHat for left ground junction drop         *
         */
        if (!gamepad1.right_bumper && gamepad1.left_bumper && gamepad1.x){
            tophatController.tophatAction = ATRobotEnumeration.AUTO_TOPHAT_GROUND_MOVE_BEGIN;
            tophatController.teleOpStep=0;
            tophatController.setTopHatPosition(tophatController.groundJunctionWristPos,false,tophatController.groundJunctionArmPos,tophatController.groundJunctionElbowPos,tophatController.leftDiagPickTTPos);
        }
        /**
         * This is to preset TopHat for right ground junction drop
         */
        if (gamepad1.right_bumper && !gamepad1.left_bumper && gamepad1.x){
            tophatController.tophatAction = ATRobotEnumeration.AUTO_TOPHAT_GROUND_MOVE_BEGIN;
            tophatController.teleOpStep=0;
            tophatController.setTopHatPosition(tophatController.groundJunctionWristPos,false,tophatController.groundJunctionArmPos,tophatController.groundJunctionElbowPos,tophatController.rightDiagPickTTPos);
        }

        /**
         * This is to preset TopHat for left low junction drop
         */
        if (!gamepad1.right_bumper && gamepad1.left_bumper && gamepad1.a){
            tophatController.tophatAction = ATRobotEnumeration.AUTO_TOPHAT_ONEWAY_MOVE_BEGIN;
            tophatController.teleOpStep=0;
            tophatController.setTopHatPosition(tophatController.lowJunctionWristPos,false,tophatController.lowJunctionArmPos,tophatController.lowJunctionElbowPos,tophatController.leftDiagPickTTPos);
        }
        /**
         * This is to preset TopHat for right low junction drop
         */
        if (gamepad1.right_bumper && !gamepad1.left_bumper && gamepad1.a){
            tophatController.tophatAction = ATRobotEnumeration.AUTO_TOPHAT_ONEWAY_MOVE_BEGIN;
            tophatController.teleOpStep=0;
            tophatController.setTopHatPosition(tophatController.lowJunctionWristPos,false,tophatController.lowJunctionArmPos,tophatController.lowJunctionElbowPos,tophatController.rightDiagPickTTPos);
        }

        /**
         * This is to preset TopHat for left medium junction drop
         */
        if (gamepad1.left_bumper && !gamepad1.right_bumper && gamepad1.b){
            tophatController.tophatAction = ATRobotEnumeration.AUTO_TOPHAT_ONEWAY_MOVE_BEGIN;
            tophatController.teleOpStep=0;
            tophatController.setTopHatPosition(tophatController.mediumJunctionWristPos,false,tophatController.mediumJunctionArmPos,tophatController.mediumJunctionElbowPos,tophatController.leftDiagPickTTPos);
        }

        /**
         * This is to preset TopHat for right medium junction drop
         */
        if (gamepad1.right_bumper && !gamepad1.left_bumper && gamepad1.b){
            tophatController.tophatAction = ATRobotEnumeration.AUTO_TOPHAT_ONEWAY_MOVE_BEGIN;
            tophatController.teleOpStep=0;
            tophatController.setTopHatPosition(tophatController.mediumJunctionWristPos,false,tophatController.mediumJunctionArmPos,tophatController.mediumJunctionElbowPos,tophatController.rightDiagPickTTPos);
        }


        /**
         * This is to preset TopHat for left high junction drop
         */
        if (!gamepad1.right_bumper && gamepad1.left_bumper && gamepad1.y){
            tophatController.tophatAction = ATRobotEnumeration.AUTO_TOPHAT_ONEWAY_MOVE_BEGIN;
            tophatController.teleOpStep=0;
            tophatController.setTopHatPosition(tophatController.highJunctionWristPos,false, tophatController.highJunctionArmPos, tophatController.highJunctionElbowPos,tophatController.leftDiagPickTTPos);
        }
        /**
         * This is to preset TopHat for right high junction drop
         */
        if (gamepad1.right_bumper && !gamepad1.left_bumper && gamepad1.y){
            tophatController.tophatAction = ATRobotEnumeration.AUTO_TOPHAT_ONEWAY_MOVE_BEGIN;
            tophatController.teleOpStep=0;
            tophatController.setTopHatPosition(tophatController.highJunctionWristPos,false, tophatController.highJunctionArmPos, tophatController.highJunctionElbowPos,tophatController.rightDiagPickTTPos);
        }

        /**
         * This is to preset TopHat for ground junction drop and also front pickup         *
         */
        if (gamepad1.right_bumper && gamepad1.left_bumper && gamepad1.x){
            tophatController.tophatAction = ATRobotEnumeration.AUTO_TOPHAT_GROUND_MOVE_BEGIN;
            tophatController.teleOpStep=0;
            tophatController.setTopHatPosition(tophatController.groundJunctionWristPos,false,tophatController.groundJunctionArmPos,tophatController.groundJunctionElbowPos,tophatController.frontPickTTPos);
        }

        /**
         * This is to preset TopHat for front low junction drop
         */
        if (gamepad1.right_bumper && gamepad1.left_bumper && gamepad1.a){
            tophatController.tophatAction = ATRobotEnumeration.AUTO_TOPHAT_ONEWAY_MOVE_BEGIN;
            tophatController.teleOpStep=0;
            tophatController.setTopHatPosition(tophatController.lowJunctionWristPos,false,tophatController.lowJunctionArmPos,tophatController.lowJunctionElbowPos,tophatController.frontPickTTPos);
        }
        /**
         * This is to preset TopHat for front medium junction drop
         */
        if (gamepad1.left_bumper && gamepad1.right_bumper && gamepad1.b){
            tophatController.tophatAction = ATRobotEnumeration.AUTO_TOPHAT_ONEWAY_MOVE_BEGIN;
            tophatController.teleOpStep=0;
            tophatController.setTopHatPosition(tophatController.mediumJunctionWristPos,false,tophatController.mediumJunctionArmPos,tophatController.mediumJunctionElbowPos,tophatController.frontPickTTPos);
        }

        /**
         * This is to preset TopHat for front high junction drop
         */
        if (gamepad1.right_bumper && gamepad1.left_bumper && gamepad1.y){
            tophatController.tophatAction = ATRobotEnumeration.AUTO_TOPHAT_ONEWAY_MOVE_BEGIN;
            tophatController.teleOpStep=0;
            tophatController.setTopHatPosition(tophatController.highJunctionWristPos,false, tophatController.highJunctionArmPos, tophatController.highJunctionElbowPos,tophatController.frontPickTTPos);
        }
    }
}
