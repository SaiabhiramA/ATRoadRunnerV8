package com.noahbres.meepmeep;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(0), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(30, -55, Math.toRadians(90)))

                                //.splineTo(50,50)
                                //.lineToLinearHeading(new Pose2d(35, -55, Math.toRadians(90)))
                                //.lineToLinearHeading(new Pose2d(35, -4, Math.toRadians(90)))
                                //.lineToLinearHeading(new Pose2d(35, -15, Math.toRadians(270)))
                                //.turn(Math.toRadians(180))
                                //splineToLinearHeading(new Pose2d(35, -4, Math.toRadians(90)),Math.toRadians(0))
                                //.back(5)
                                //.turn(Math.toRadians(180))
                                //.forward(30)
                                //.turn(Math.toRadians(90))
                                //.forward(30)
                                //.turn(Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(33, -40, Math.toRadians(90)),Math.toRadians(90))
                                //.waitSeconds(3)
                                .splineToLinearHeading(new Pose2d(33, -3, Math.toRadians(90)),Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(33, -12, Math.toRadians(90)),Math.toRadians(90))
                                .lineToLinearHeading(new Pose2d(48, -12, Math.toRadians(270)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}