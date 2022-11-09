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
                        drive.trajectorySequenceBuilder(new Pose2d(40, 60, Math.toRadians(270))) // blue left high drop
                        //drive.trajectorySequenceBuilder(new Pose2d(-34, 60, Math.toRadians(270)))// blue right high drop
                                //Red Right High Drop & Drop
                                /*.splineToLinearHeading(new Pose2d(35, -40, Math.toRadians(90)), Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(41, -5, Math.toRadians(90)), Math.toRadians(90))
                                .lineToLinearHeading(new Pose2d(56, -6, Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(60, -28, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(12, -32, Math.toRadians(180)))*/

                                //blue left high drop & park
                                .waitSeconds(2.5)
                                .splineToConstantHeading(new Vector2d(36, 58), Math.toRadians(-90))

                                .lineTo(new Vector2d(36, 4))
                                .waitSeconds(1)
                                .lineTo(new Vector2d(33, 4))
                                .lineTo(new Vector2d(33, 12))
                                .lineTo(new Vector2d(46, 12))
                                .addTemporalMarker(.001, ()->{})
                                .addTemporalMarker(6, ()->{})
                                /*.splineToConstantHeading(new Vector2d(36, 30), Math.toRadians(-90))
                                .splineToConstantHeading(new Vector2d(44, 10), Math.toRadians(0))
                                .lineToConstantHeading(new Vector2d(36, 10))
                                .lineToSplineHeading(new Pose2d(36, 32, Math.toRadians(180)))
                                .lineToConstantHeading(new Vector2d(12, 32))*/

                                //Blue right high drop & park in three different zones followed by substation pickup
                                /*.splineToConstantHeading(new Vector2d(-36, 55), Math.toRadians(-90))
                                .lineToConstantHeading(new Vector2d(-36, 2))
                                .addTemporalMarker(1.5, ()->{

                                })
                                .waitSeconds(1)
                                //.splineToConstantHeading(new Vector2d(-36, 20), Math.toRadians(-90))
                                //.lineToConstantHeading(new Vector2d(-36, 12))
                                .splineToConstantHeading(new Vector2d(-44, 12), Math.toRadians(0))*/

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}