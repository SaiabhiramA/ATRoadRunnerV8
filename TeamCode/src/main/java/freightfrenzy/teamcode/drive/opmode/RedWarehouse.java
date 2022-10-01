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

@Autonomous (name = "Red Alliance & Warehouse")
@Disabled
public class RedWarehouse extends LinearOpMode {
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

        Pose2d startPose = new Pose2d(15.44, -62.52, Math.toRadians(93.26));
        drive.setPoseEstimate(startPose);
        Trajectory lineUpToShippingHubP1 = drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-7.18, -55, Math.toRadians(90.07)), Math.toRadians(90.00))
                .build();
        Trajectory lineUpToShippingHubP2;
        if (tophat.duckLocationSymbol().equals("M")) {
            lineUpToShippingHubP2 = drive.trajectoryBuilder(lineUpToShippingHubP1.end())
                    .lineTo(new Vector2d(-7.18, -48), drive.getVelocityConstraint(10, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                    .build();

        } else if (tophat.duckLocationSymbol().equals("L")) {
            lineUpToShippingHubP2 = drive.trajectoryBuilder(lineUpToShippingHubP1.end())
                    .lineTo(new Vector2d(-7.18, -49), drive.getVelocityConstraint(10, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                    .build();

        } else {
            lineUpToShippingHubP2 = drive.trajectoryBuilder(lineUpToShippingHubP1.end())
                    .lineTo(new Vector2d(-7.18, -47), drive.getVelocityConstraint(10, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                    .build();
        }


        Trajectory lineUpToWarehouseParkingP1 = drive.trajectoryBuilder(lineUpToShippingHubP2.end())
                .splineToLinearHeading(new Pose2d(0, -65, Math.toRadians(0)), Math.toRadians(0))
                .build();

       Trajectory lineUpToWarehouseParkingP2 = drive.trajectoryBuilder(lineUpToWarehouseParkingP1.end())
                .lineTo(new Vector2d(45.00, -65))
                .build();

        /*Trajectory lineUpToWarehouseParkingP3 = drive.trajectoryBuilder(lineUpToWarehouseParkingP2.end())
                .strafeRight(1)
                .build();*/

       Trajectory lineUpToWarehouseParkingP3 = drive.trajectoryBuilder(lineUpToWarehouseParkingP2.end())
                .lineTo(new Vector2d(45.00, -45.00))
                .build();


        //sleep (2000);
        drive.followTrajectory(lineUpToShippingHubP1);
        sleep(500);
        tophat.setArmDropPosition(tophat.duckLocationSymbol());

        tophat.setIntakeBoxPosition(tophat.duckLocationSymbol());

        drive.followTrajectory(lineUpToShippingHubP2);
        sleep(500);
        tophat.DropBlock();
        sleep(500);
        drive.followTrajectory(lineUpToWarehouseParkingP1);
        sleep(500);
        tophat.setArmDropPosition("W");
        sleep(500); // replace this with spinning of carousel
        drive.followTrajectory(lineUpToWarehouseParkingP2);
        sleep(500); // replace this with spinning of carousel
        drive.followTrajectory(lineUpToWarehouseParkingP3);
        sleep(500); // replace this with spinning of carousel*/

    }

    private void setTrajectory() {
    }
}
