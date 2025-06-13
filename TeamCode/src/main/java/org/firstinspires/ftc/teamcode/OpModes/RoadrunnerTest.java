package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring;


@Autonomous(name="RoadRunnerTest", group="Linear OpMode")
@Config
public class RoadrunnerTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Scoring scoring = new Scoring(this);
        Lift lift = new Lift(this);

        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        scoring.init();
        lift.init(scoring);



        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .setTangent(0).splineToLinearHeading(new Pose2d(24, 36, 0), 0);


        Action driveToObservationZone = tab1.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(0, 0))
                .build();

        Action driveToSubmersible = tab1.build();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(driveToSubmersible);
        //Actions.runBlocking(scoring.clawToObservationZone());
        //Actions.runBlocking(scoring.clawSubmersible());
        //Actions.runBlocking(scoring.pivotAction(Scoring.PIVOT_OBSERVATION_ZONE));
        //Actions.runBlocking(scoring.pivotAction(Scoring.PIVOT_ZERO));
        //Actions.runBlocking(lift.liftAction(Lift.EXT_LOW_BASKET));
        //Actions.runBlocking(lift.liftAction(Lift.EXT_RETRACTED));

        /*Actions.runBlocking(new ParallelAction(
                scoring.pivotObservationZone(),
                scoring.clawToObservationZone()
        ));
        Actions.runBlocking(new ParallelAction(
                scoring.pivotSubmersible(),
                scoring.clawSubmersible()
        ));

         */
    }
}