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
                                .splineToLinearHeading(new Pose2d(35, -40, Math.toRadians(90)), Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(41, -5, Math.toRadians(90)), Math.toRadians(90))
                                /*.lineToLinearHeading(new Pose2d(13, -6, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(13, -40, Math.toRadians(180))) //Park1*/

                                /*.lineToLinearHeading(new Pose2d(36, -6, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(36, -40, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(13, -40, Math.toRadians(180))) //Park2*/
                                .lineToLinearHeading(new Pose2d(55, -6, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(55, -40, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(13, -40, Math.toRadians(180)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}