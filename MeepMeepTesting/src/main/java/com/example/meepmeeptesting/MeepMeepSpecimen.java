package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepSpecimen {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(16.25, 17)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(8, -61, Math.toRadians(90)))
                    .splineToConstantHeading(new Vector2d(0, -33), Math.toRadians(90)) //drop off pre loaded

                    .setTangent(Math.toRadians(300))
                    .splineToConstantHeading(new Vector2d(34, -37), Math.toRadians(90)) //path around submersible
                    .setTangent(Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(35, -6), Math.toRadians(300)) //to first
                    .splineToConstantHeading(new Vector2d(44, -56), Math.toRadians(90)) //push observation zone
                    .splineToConstantHeading(new Vector2d(52, -6), Math.toRadians(300)) //to second
                    .splineToConstantHeading(new Vector2d(50, -50), Math.toRadians(180)) //push observation zone
                    .splineToSplineHeading(new Pose2d(36,-50,Math.toRadians(265)),Math.toRadians(270)) //turn
                    .splineToConstantHeading(new Vector2d(36,-58),Math.toRadians(270)) //intaking

                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(0,-31,Math.toRadians(270)),Math.toRadians(90)) //drop off first
                    .setTangent(Math.toRadians(315))
                    .splineToLinearHeading(new Pose2d(36,-59,Math.toRadians(270)),Math.toRadians(270)) //intaking
                    .setTangent(Math.toRadians(120))
                    .splineToLinearHeading(new Pose2d(0,-33,Math.toRadians(270)),Math.toRadians(90)) //drop off second
                    .setTangent(Math.toRadians(315))
                    .splineToLinearHeading(new Pose2d(36,-59,Math.toRadians(270)),Math.toRadians(270)) //intaking
                    .setTangent(Math.toRadians(120))
                    .splineToLinearHeading(new Pose2d(-3,-33,Math.toRadians(270)),Math.toRadians(90)) //drop off third
                    .splineToLinearHeading(new Pose2d(40,-59,Math.toRadians(270)),Math.toRadians(0)) //park



//                        .splineTo(new Vector2d(-48,-40),Math.toRadians(90)) //to first sample; rotation 0.5


//                        .setReversed(true)
//                        .splineTo(new Vector2d(-52, -53),Math.toRadians(225)) //drop off sample 1
//                        .setReversed(false)
//                        .splineTo(new Vector2d(-53,-40),Math.toRadians(110)) //to second sample; rotation 0.75
//                        .setReversed(true)
//                        .splineTo(new Vector2d(-52, -53),Math.toRadians(225)) //drop off sample 2
//                        .setReversed(false)
//                        .splineTo(new Vector2d(-56,-40),Math.toRadians(130)) // to third sample; rotation 0.95/1
//                        .setReversed(true)
//                        .splineTo(new Vector2d(-52,-53),Math.toRadians(225)) //drop off sample 3
//                        .setReversed(false)
//                        .splineTo(new Vector2d(-30,-11),Math.toRadians(0))

                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}