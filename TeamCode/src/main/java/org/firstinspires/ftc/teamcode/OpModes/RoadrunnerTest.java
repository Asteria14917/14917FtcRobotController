package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RR.Localizer;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring;

@Autonomous(name="RoadRunnerTest", group="Linear OpMode")
@Config
public class RoadrunnerTest extends LinearOpMode {

    private Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {
        Scoring scoring = new Scoring(this);
        Lift lift = new Lift(this);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        scoring.init();
        lift.init(scoring);

        waitForStart();

        if (isStopRequested()) return;

        int cycle = 0;

        while(opModeIsActive() && cycle < 5) {
            // Run individual trajectories
            Actions.runBlocking(new ParallelAction(
                    buildDriveToSubmersible(drive, cycle),
                    new SequentialAction(
                            scoring.pivotAction(Scoring.PIVOT_ZERO),
                            lift.liftAction(Lift.EXT_HIGH_CHAMBER)
                    ),
                    scoring.clawSubmersible())

            );
            Actions.runBlocking(new ParallelAction(
                    buildBackToObservationZone(drive, drive.localizer.getPose()),
                    lift.liftAction(Lift.EXT_RETRACTED),
                    scoring.pivotAction(Scoring.PIVOT_OBSERVATION_ZONE),
                    scoring.clawToObservationZone())
            );
            cycle++;
        }
        // Run sequence of two new, fresh actions
        /*
        Actions.runBlocking(
                new SequentialAction(
                        buildBackToSub(drive),
                        buildCloseOut(drive)
                )
        );

         */

        /*
        // Additional scoring/lift logic can go here
        Actions.runBlocking(scoring.clawToObservationZone());
        Actions.runBlocking(scoring.clawSubmersible());
        Actions.runBlocking(scoring.pivotAction(Scoring.PIVOT_OBSERVATION_ZONE));
        Actions.runBlocking(scoring.pivotAction(Scoring.PIVOT_ZERO));
        Actions.runBlocking(lift.liftAction(Lift.EXT_LOW_BASKET));
        Actions.runBlocking(lift.liftAction(Lift.EXT_RETRACTED));

        Actions.runBlocking(new ParallelAction(
                scoring.pivotObservationZone(),
                scoring.clawToObservationZone()
        ));
        Actions.runBlocking(new ParallelAction(
                scoring.pivotSubmersible(),
                scoring.clawSubmersible()
        ));
        */
    }

    private Action buildDriveToSubmersible(MecanumDrive drive, double cycle) {
        int chamberOffset = 2;
        return drive.actionBuilder(initialPose)
                .waitSeconds(.4)
                .setTangent(Math.toRadians(90)) // Face upward initially
                .splineToLinearHeading(new Pose2d(24, 10 + cycle*chamberOffset, Math.toRadians(0)), Math.toRadians(0)) // Arrive facing forward
                .build();
    }

    private Action buildBackToObservationZone(MecanumDrive drive, Pose2d startingPose) {
        return drive.actionBuilder(startingPose)
                .strafeTo(new Vector2d(8, -24)) // Re-approach same point (demo)
                .build();
    }

    private Action buildCloseOut(MecanumDrive drive) {
        return drive.actionBuilder(new Pose2d(24, 10, Math.toRadians(0)))
                .strafeTo(new Vector2d(0, 0)) // Return to origin
                .build();
    }
}
