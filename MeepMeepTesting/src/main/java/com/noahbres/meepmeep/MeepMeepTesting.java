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
                        //drive.trajectorySequenceBuilder(new Pose2d(40, 60, Math.toRadians(270))) // blue left high drop
                        drive.trajectorySequenceBuilder(new Pose2d(-33, 60, Math.toRadians(270)))// blue right high drop
                                //Red Right High Drop & Drop
                                /*.splineToLinearHeading(new Pose2d(35, -40, Math.toRadians(90)), Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(41, -5, Math.toRadians(90)), Math.toRadians(90))
                                .lineToLinearHeading(new Pose2d(56, -6, Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(60, -28, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(12, -32, Math.toRadians(180)))*/


                                //blue left high drop & park
                                /*.waitSeconds(2.5)
                                .splineToConstantHeading(new Vector2d(36,58), Math.toRadians(-90))
                                .splineToConstantHeading(new Vector2d(36,3), Math.toRadians(-90))
                                .splineToConstantHeading(new Vector2d(33,3), Math.toRadians(0))
                                .waitSeconds(.5)
                                .addDisplacementMarker(()->{})
                                .waitSeconds(1.5)
                                .splineToConstantHeading(new Vector2d(36,15), Math.toRadians(-270))
                                .waitSeconds(.25)
                                .splineToConstantHeading(new Vector2d(45,10), Math.toRadians(-90))
                                .waitSeconds(12)*/

                                //Blue right high drop & park in three different zones followed by substation pickup
                                .addTemporalMarker(.001, ()->{})
                                .waitSeconds(2.5)
                                .splineToConstantHeading(new Vector2d(-36,58), Math.toRadians(-90))
                                .splineToConstantHeading(new Vector2d(-36,40), Math.toRadians(-90))
                                .splineToConstantHeading(new Vector2d(-36,1), Math.toRadians(-90))
                                .splineToConstantHeading(new Vector2d(-29,1), Math.toRadians(-180))
                                .waitSeconds(.5)
                                .splineToConstantHeading(new Vector2d(-30,1), Math.toRadians(-180))
                                .addDisplacementMarker(()->{})
                                .waitSeconds(1.5)
                                .splineToConstantHeading(new Vector2d(-40,10), Math.toRadians(0))
                                //.waitSeconds(12)
                                //.lineToConstantHeading(new Vector2d(-12,10)) // Park 1 & substation
                                //.lineToLinearHeading(new Pose2d(-12,60,Math.toRadians(90))) // substation parking
                                //.lineToLinearHeading(new Pose2d(-12,36,Math.toRadians(0))) // substation cone pickup
                                //.lineToConstantHeading(new Vector2d(-60,12)) // Park 3*/

                                //Red left high drop & park in three different zones followed by substation pickup
                               /*.waitSeconds(2.5)
                                .addTemporalMarker(.001, ()->{})
                                .splineToConstantHeading(new Vector2d(-36,-50), Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(-36,-8), Math.toRadians(90))
                                .waitSeconds(.5)
                                .splineToConstantHeading(new Vector2d(-33,-3.5), Math.toRadians(90))
                                .waitSeconds(.5)
                                .addDisplacementMarker(()->{})
                                .waitSeconds(1)
                                .splineToConstantHeading(new Vector2d(-36,-10), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-45,-10), Math.toRadians(180)) ////cone pickup & drop
                                .addDisplacementMarker(()->{})
                                .waitSeconds(1)
                                //.lineToConstantHeading(new Vector2d(-60,-10)) // Park 1
                                .addDisplacementMarker(()->{})
                                .waitSeconds(1)
                                .lineToConstantHeading(new Vector2d(-36,-10)) // Park 2

                                .addDisplacementMarker(()->{})
                                .waitSeconds(1)
                                .lineToConstantHeading(new Vector2d(-12,-10)) // Park 3
                                //.lineToLinearHeading(new Pose2d(-12,-60,Math.toRadians(0))) // substation parking
                                .lineToLinearHeading(new Pose2d(-12,-36,Math.toRadians(0))) // substation cone pickup*/


                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}