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

@Autonomous (name = "Red Alliance Carousel-Red Parking")
@Disabled
public class RedCarouselParking extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AtomicToadsMecanumDrive drive = new AtomicToadsMecanumDrive(hardwareMap);
        TopHatControllerV03 tophat = new TopHatControllerV03();
        tophat.initializeRobot(hardwareMap, telemetry, "RED");
        tophat.Display_Key_Measures("INIT");
        telemetry.update();
        waitForStart();
        tophat.detectDuckByAllianceMarkers("RED", "Play");
        tophat.Display_Key_Measures("PLAY");
        telemetry.update();
        tophat.IntakeAction("R", -0.1);
        tophat.MoveTurnTablePosition();


        Pose2d startPose = new Pose2d (-41, -62, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        Trajectory lineUpToShippingHubP1 = drive.trajectoryBuilder(startPose)
            .splineToConstantHeading(new Vector2d(-60, -36), Math.toRadians(80))
            .build();
        Trajectory lineUpToShippingHubP2;
        if (tophat.duckLocationSymbol().equals("M")){
            lineUpToShippingHubP2= drive.trajectoryBuilder(lineUpToShippingHubP1.end())
                    .splineToLinearHeading(new Pose2d(-35, -18, Math.toRadians(0)), Math.toRadians(0))
                    .build();
        } else if(tophat.duckLocationSymbol().equals("L")){
            lineUpToShippingHubP2 = drive.trajectoryBuilder(lineUpToShippingHubP1.end())
                    .splineToLinearHeading(new Pose2d(-36, -18, Math.toRadians(0)), Math.toRadians(0))
                    .build();
    } else {
        lineUpToShippingHubP2 = drive.trajectoryBuilder(lineUpToShippingHubP1.end())
        .splineToLinearHeading(new Pose2d(-34, -18, Math.toRadians(0)), Math.toRadians(0))
                .build();}

        Trajectory lineUpToShippingHubP3 = drive.trajectoryBuilder(lineUpToShippingHubP2.end())
                .lineToLinearHeading(new Pose2d(-65, -24, Math.toRadians(90)))
                .build();
        Trajectory lineUpToCarouselP2 = drive.trajectoryBuilder(lineUpToShippingHubP3.end())
                .lineToLinearHeading(new Pose2d(-65, -48, Math.toRadians(90)))
                .build();

        Trajectory lineUpToCarouselP3 = drive.trajectoryBuilder(lineUpToCarouselP2.end())
                .lineTo(new Vector2d(-65, -55),drive.getVelocityConstraint(10, MAX_ANG_VEL, TRACK_WIDTH),drive.getAccelerationConstraint(MAX_ACCEL))
                .build();
        Trajectory lineUpToRedParkingP1 = drive.trajectoryBuilder(lineUpToCarouselP3.end())
                .splineToLinearHeading(new Pose2d(-58, -36, Math.toRadians(0)), Math.toRadians(0))
                .build();
        Trajectory lineUpToRedParkingP2 = drive.trajectoryBuilder(lineUpToRedParkingP1.end())
                .lineTo(new Vector2d(-64, -36),drive.getVelocityConstraint(10, MAX_ANG_VEL, TRACK_WIDTH),drive.getAccelerationConstraint(MAX_ACCEL))
                .build();
        drive.followTrajectory(lineUpToShippingHubP1);
        sleep(100);
        tophat.setArmDropPosition(tophat.duckLocationSymbol());
        tophat.setIntakeBoxPosition(tophat.duckLocationSymbol());
        drive.followTrajectory(lineUpToShippingHubP2);
        tophat.DropBlock();
        drive.followTrajectory(lineUpToShippingHubP3);
        sleep(100);
        tophat.setArmDropPosition("W");
        drive.followTrajectory(lineUpToCarouselP2);
        drive.followTrajectory(lineUpToCarouselP3);
        tophat.Red_alliance_carousel();
        drive.followTrajectory(lineUpToRedParkingP1);
        sleep(100); // replace this with spinning of carousel
        drive.followTrajectory(lineUpToRedParkingP2);
        sleep(100); // replace this with spinning of carousel
        //following lines are changed
        tophat.setIntakeBoxPosition("R");
        tophat.setArmDropPosition("P");
    }
}
