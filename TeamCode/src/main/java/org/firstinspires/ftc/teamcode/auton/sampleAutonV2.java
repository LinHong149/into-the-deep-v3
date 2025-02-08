package org.firstinspires.ftc.teamcode.auton;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
        import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
        import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

        import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous
public class sampleAutonV2 extends LinearOpMode{



    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-38, -61, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);


        TrajectoryActionBuilder dropOffPreload = drive.actionBuilder(initialPose)
                .splineToLinearHeading(new Pose2d(-52.5, -51.5, Math.toRadians(45)), Math.toRadians(261))
                .waitSeconds(.2);
        TrajectoryActionBuilder toFirstSample = drive.actionBuilder(new Pose2d(-52.5,-51.5,Math.toRadians(45)))
                .splineTo(new Vector2d(-48.1,-40),Math.toRadians(90))
                .waitSeconds(.1);
        TrajectoryActionBuilder dropOffFirst = drive.actionBuilder(new Pose2d(-48.1,-40,Math.toRadians(90)))
                .setReversed(true)
                .splineTo(new Vector2d(-51.5,-52.5),Math.toRadians(229.5))
                .waitSeconds(.2);
        TrajectoryActionBuilder toSecondSample = drive.actionBuilder(new Pose2d(-51.5,-52.5,Math.toRadians(49.5)))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-61.5,-40,Math.toRadians(99)),Math.toRadians(107))
                .waitSeconds(.1);
        TrajectoryActionBuilder dropOffSecond = drive.actionBuilder(new Pose2d(-61.5,-40,Math.toRadians(99)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-54,-50,Math.toRadians(48)),Math.toRadians(232))
                .waitSeconds(.2);
        TrajectoryActionBuilder toThirdSample = drive.actionBuilder(new Pose2d(-54,-50,Math.toRadians(48)))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-61,-40, Math.toRadians(135)), Math.toRadians(55))
                .waitSeconds(.1);
        TrajectoryActionBuilder dropOffThird = drive.actionBuilder(new Pose2d(-61,-40,Math.toRadians(135)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-53.5,-51,Math.toRadians(48)), Math.toRadians(230))
                .waitSeconds(.2);
        TrajectoryActionBuilder park = drive.actionBuilder(new Pose2d(-53.5,-51,Math.toRadians(48)))
                .setReversed(false)
                .splineTo(new Vector2d(-30,-11),Math.toRadians(0))
                .waitSeconds(.1);




        waitForStart();

//        Code Running
        if (isStopRequested()) return;

        if (opModeIsActive()) {
            Actions.runBlocking(
                    new SequentialAction(
                        dropOffPreload.build(),
                        toFirstSample.build(),
                        dropOffFirst.build(),
                        toSecondSample.build(),
                        dropOffSecond.build(),
                        toThirdSample.build(),
                        dropOffThird.build()
                )
            );
        }



    }





}