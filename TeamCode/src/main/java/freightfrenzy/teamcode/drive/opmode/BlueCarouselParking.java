package freightfrenzy.teamcode.drive.opmode;

import static freightfrenzy.teamcode.drive.AtomicToadsDriveConstants.MAX_ACCEL;
import static freightfrenzy.teamcode.drive.AtomicToadsDriveConstants.MAX_ANG_VEL;
import static freightfrenzy.teamcode.drive.AtomicToadsDriveConstants.TRACK_WIDTH;
import static freightfrenzy.teamcode.drive.AtomicToadsDriveConstants.MAX_VEL;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import freightfrenzy.teamcode.drive.AtomicToadsMecanumDrive;
import freightfrenzy.teamcode.drive.TopHatController;
import freightfrenzy.teamcode.drive.TopHatControllerV02;
import freightfrenzy.teamcode.drive.TopHatControllerV03;

@Autonomous (name = "Blue Alliance Carousel-Blue Parking")
@Disabled
public class BlueCarouselParking extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AtomicToadsMecanumDrive drive = new AtomicToadsMecanumDrive(hardwareMap);
        TopHatControllerV03 tophat = new TopHatControllerV03();
        tophat.initializeRobot(hardwareMap, telemetry, "BLUE");
        tophat.Display_Key_Measures("INIT");
        telemetry.update();
        waitForStart();
        tophat.detectDuckByAllianceMarkers("BLUE", "Play");
        tophat.Display_Key_Measures("PLAY");
        telemetry.update();
        tophat.IntakeAction("R", -0.1);
        tophat.MoveTurnTablePosition();
        Pose2d startPose = new Pose2d(-43.62, 62.21, Math.toRadians(271.33));
        drive.setPoseEstimate(startPose);
        Trajectory lineUpToShippingHubP1 = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-59.46, 40.37), Math.toRadians(-80))
                .build();
        Trajectory lineUpToShippingHubP2 = drive.trajectoryBuilder(lineUpToShippingHubP1.end())
                .splineToLinearHeading(new Pose2d(-45, 16, Math.toRadians(30)), Math.toRadians(-200))
                .build();
        Trajectory lineUpToShippingHubP3;
        if (tophat.duckLocationSymbol().equals("M")) {
            lineUpToShippingHubP3 = drive.trajectoryBuilder(lineUpToShippingHubP2.end())
            .lineTo(new Vector2d(-37.5, 16), drive.getVelocityConstraint(10, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                    .build();

        } else if (tophat.duckLocationSymbol().equals("L")) {
            lineUpToShippingHubP3 = drive.trajectoryBuilder(lineUpToShippingHubP2.end())
                    .lineTo(new Vector2d(-39, 16), drive.getVelocityConstraint(10, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                    .build();

        } else {
            lineUpToShippingHubP3 = drive.trajectoryBuilder(lineUpToShippingHubP2.end())
                    .lineTo(new Vector2d(-36, 16), drive.getVelocityConstraint(10, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                    .build();
        }

        Trajectory lineuptoCarouselP1 = drive.trajectoryBuilder(lineUpToShippingHubP3.end())
                .splineToLinearHeading(new Pose2d(-56, 16, Math.toRadians(0)), Math.toRadians(0))
                .build();

       Trajectory lineuptoCarouselP2 = drive.trajectoryBuilder(lineuptoCarouselP1.end())
               .lineTo(new Vector2d(-60, 52))
                .build();

        Trajectory lineUpToCarouselP3 = drive.trajectoryBuilder(lineuptoCarouselP2.end())
                .lineTo(new Vector2d(-60, 60.5), drive.getVelocityConstraint(10, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                .build();

        Trajectory lineUpToBlueParkingP1 = drive.trajectoryBuilder(lineUpToCarouselP3.end())
                .lineTo(new Vector2d(-64, 40))
                .build();

        drive.followTrajectory(lineUpToShippingHubP1);
        sleep(100);
        tophat.setArmDropPosition(tophat.duckLocationSymbol());
        tophat.setIntakeBoxPosition(tophat.duckLocationSymbol());
        drive.followTrajectory(lineUpToShippingHubP2);
        drive.followTrajectory(lineUpToShippingHubP3);
        tophat.DropBlock();
        drive.followTrajectory(lineuptoCarouselP1);
        sleep(100);
        //tophat.setArmDropPosition("W");
        drive.followTrajectory(lineuptoCarouselP2);
        drive.followTrajectory(lineUpToCarouselP3);
        tophat.setIntakeBoxPosition("R");
        tophat.setArmDropPosition("P");
        tophat.Blue_alliance_carousel();
        drive.followTrajectory(lineUpToBlueParkingP1);
        sleep(100);
        //following lines are changed


    }

   }
