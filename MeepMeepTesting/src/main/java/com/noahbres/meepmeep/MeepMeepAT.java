package com.noahbres.meepmeep;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepAT {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                //.addEntity(blueAllianceRightStraightTuns(meepMeep))
                .addEntity(blueAllianceRightSplaineTuns(meepMeep))
                .start();
    }
    private static RoadRunnerBotEntity blueAllianceRightSplaineTuns(MeepMeep atRobot){
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(atRobot)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(270), Math.toRadians(270), 14)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, 62, -90))
                                //.back(30)
                                //.turn(Math.toRadians(-90))
                                //.forward(30)
                                //.turn(Math.toRadians(45))
                                //.forward(20)
                                .splineToConstantHeading(new Vector2d(-60, 57.5), Math.toRadians(-100))
                                //.splineToLinearHeading(new Pose2d(-62, 30, Math.toRadians(-90)), Math.toRadians(-90))
                                .splineToLinearHeading(new Pose2d(-50, 12, Math.toRadians(-90)), Math.toRadians(0))
                                //.splineToLinearHeading(new Pose2d(52, 10, Math.toRadians(90)), Math.toRadians(20))
                                //.strafeLeft(10)
                                //.turn(Math.toRadians(90))
                                //.forward(30)

                                //.turn(Math.toRadians(90))
                                //.forward(30)
                                //.turn(Math.toRadians(90))
                                .build()
                );
        return myBot;
    }

    private static RoadRunnerBotEntity blueAllianceRightStraightTuns(MeepMeep atRobot){
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(atRobot)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, 60, 0))
                                .back(30)
                                .turn(Math.toRadians(-90))
                                .forward(30)
                                .turn(Math.toRadians(45))
                                .forward(20)
                                //.strafeLeft(10)
                                //.turn(Math.toRadians(90))
                                //.forward(30)
                                //.turn(Math.toRadians(90))
                                //.forward(30)
                                //.turn(Math.toRadians(90))
                                .build()
                );
        return myBot;
    }
}