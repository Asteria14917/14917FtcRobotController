package org.firstinspires.ftc.teamcode.OpModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RR.Drawing;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name="PedroXRoadRunnerSpecimenAuto", group="Linear OpMode")
@Config
public class PedroXRoadrunnerSpecimenAuto extends LinearOpMode {

    private Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
    private Follower follower;
    public ElapsedTime timer = new ElapsedTime();
    Scoring scoring;
    Lift lift;
    private Timer pathTimer, actionTimer, opmodeTimer;
    int cycle = 0;

    PathChain retrieveGroundSamples, retrieveFirstSpecimen, scorePreLoad;

    @Override
    public void runOpMode() throws InterruptedException {
        scoring = new Scoring(this);
        lift = new Lift(this);
        pathTimer = new Timer();

        buildPaths();

        //Post is 9 and 57 on Pedro Coordinate System
        Pose startPose = new Pose(0, 0, 0);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        scoring.init();
        lift.init(scoring);

        waitForStart();

        cycle = 0;

        if (isStopRequested()) return;

        //*******SCORE PRELOAD********
        //Score preload on submersible
        Actions.runBlocking(scoreOnSubmersible);
        //Retract lift and run through pathchain to retrieve ground samples
        Actions.runBlocking(retrieveSamples);

        //Pick up first specimen from observation zone
        Actions.runBlocking(new SequentialAction(
                scoring.clawToObservationZone(),
                scoring.pivotAction(Scoring.PIVOT_OBSERVATION_ZONE),
                pedroDriveOnPathChain(retrieveFirstSpecimen))
        );

        while(opModeIsActive() && cycle < 5) {
            // Run individual trajectories
            Actions.runBlocking(new ParallelAction(
                    //buildDriveToSubmersible(drive, cycle),
                    pedroDriveToPose(new Pose(24, 10+cycle*2, 0)),
                    new SequentialAction(
                            scoring.pivotAction(Scoring.PIVOT_ZERO),
                            lift.liftAction(Lift.EXT_HIGH_CHAMBER)
                    ),
                    scoring.clawSubmersible())
            );
            Actions.runBlocking(new ParallelAction(
                    pedroDriveToPose(new Pose(0, -24, 0)),
                    lift.liftAction(Lift.EXT_RETRACTED),
                    scoring.pivotAction(Scoring.PIVOT_OBSERVATION_ZONE),
                    scoring.clawToObservationZone())
            );
            cycle++;
        }
    }

    ParallelAction retrieveSamples = new ParallelAction(
            pedroDriveOnPathChain(retrieveGroundSamples),
            lift.liftAction((Lift.EXT_RETRACTED)),
            scoring.clawToObservationZone()
            );

    ParallelAction scoreOnSubmersible = new ParallelAction(
            pedroBezierDriveToPose(new Pose(35, 70-cycle*2, 0), new Pose(16,68, 0)),
            new SequentialAction(
            scoring.pivotAction(Scoring.PIVOT_ZERO),
                            lift.liftAction(Lift.EXT_HIGH_CHAMBER)
            ),
            scoring.clawSubmersible());

    ParallelAction pickUpSpecimen = new ParallelAction(
            pedroTwoControlPointsBezierDriveToPose(new Pose(8, 23.5, 0), new Pose(21.5,65, 0), new Pose(37, 22, 0)),
            new SequentialAction(
                    scoring.pivotAction(Scoring.PIVOT_ZERO),
                    lift.liftAction(Lift.EXT_HIGH_CHAMBER)
            ),
            scoring.clawSubmersible());

    private Action pedroDriveToPose(Pose targetPose){
        return new Action() {
            private boolean initialized = false;
            Path newPath = null;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                    timer.reset();
                    Pose initialPose = follower.getPose();
                    newPath = new Path(new BezierLine(new Point(initialPose), new Point(targetPose)));
                    newPath.setLinearHeadingInterpolation(initialPose.getHeading(),targetPose.getHeading());
                    follower.followPath(newPath);
                }

                follower.update();

                packet.fieldOverlay().setStroke("#3F51B5");
                Pose currentPose = follower.getPose();
                Drawing.drawRobot(packet.fieldOverlay(), new Pose2d(currentPose.getX(), currentPose.getY(), currentPose.getHeading()));
                FtcDashboard.getInstance().sendTelemetryPacket(packet);

                telemetry.addData("timer", timer.seconds());
                telemetry.addData("x", follower.getPose().getX());
                telemetry.addData("y", follower.getPose().getY());
                telemetry.addData("heading", follower.getPose().getHeading());
                telemetry.update();

                return follower.isBusy();
            }
        };
    }

    private Action pedroBezierDriveToPose(Pose targetPose, Pose controlPoint){
        return new Action() {
            private boolean initialized = false;
            Path newPath = null;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                    timer.reset();
                    Pose initialPose = follower.getPose();
                    newPath = new Path(new BezierCurve(new Point(initialPose), new Point(controlPoint), new Point(targetPose)));
                    newPath.setLinearHeadingInterpolation(initialPose.getHeading(),targetPose.getHeading());
                    follower.followPath(newPath);
                }

                follower.update();

                packet.fieldOverlay().setStroke("#3F51B5");
                Pose currentPose = follower.getPose();
                Drawing.drawRobot(packet.fieldOverlay(), new Pose2d(currentPose.getX(), currentPose.getY(), currentPose.getHeading()));
                FtcDashboard.getInstance().sendTelemetryPacket(packet);

                telemetry.addData("timer", timer.seconds());
                telemetry.addData("x", follower.getPose().getX());
                telemetry.addData("y", follower.getPose().getY());
                telemetry.addData("heading", follower.getPose().getHeading());
                telemetry.update();

                return follower.isBusy();
            }
        };
    }

    private Action pedroTwoControlPointsBezierDriveToPose(Pose targetPose, Pose controlPointOne, Pose controlPointTwo){
        return new Action() {
            private boolean initialized = false;
            Path newPath = null;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                    timer.reset();
                    Pose initialPose = follower.getPose();
                    newPath = new Path(new BezierCurve(new Point(initialPose), new Point(controlPointOne), new Point(controlPointTwo), new Point(targetPose)));
                    newPath.setLinearHeadingInterpolation(initialPose.getHeading(),targetPose.getHeading());
                    follower.followPath(newPath);
                }

                follower.update();

                packet.fieldOverlay().setStroke("#3F51B5");
                Pose currentPose = follower.getPose();
                Drawing.drawRobot(packet.fieldOverlay(), new Pose2d(currentPose.getX(), currentPose.getY(), currentPose.getHeading()));
                FtcDashboard.getInstance().sendTelemetryPacket(packet);

                telemetry.addData("timer", timer.seconds());
                telemetry.addData("x", follower.getPose().getX());
                telemetry.addData("y", follower.getPose().getY());
                telemetry.addData("heading", follower.getPose().getHeading());
                telemetry.update();

                return follower.isBusy();
            }
        };
    }
    private Action pedroDriveOnPath(Path targetPath){
        return new Action() {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                    timer.reset();
                    follower.followPath(targetPath);
                }

                follower.update();

                packet.fieldOverlay().setStroke("#3F51B5");
                Pose currentPose = follower.getPose();
                Drawing.drawRobot(packet.fieldOverlay(), new Pose2d(currentPose.getX(), currentPose.getY(), currentPose.getHeading()));
                FtcDashboard.getInstance().sendTelemetryPacket(packet);

                telemetry.addData("timer", timer.seconds());
                telemetry.addData("x", follower.getPose().getX());
                telemetry.addData("y", follower.getPose().getY());
                telemetry.addData("heading", follower.getPose().getHeading());
                telemetry.update();

                return follower.isBusy();
            }
        };
    }

    private Action pedroDriveOnPathChain(PathChain targetPathChain){
        return new Action() {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                    timer.reset();
                    follower.followPath(targetPathChain);
                }

                follower.update();

                packet.fieldOverlay().setStroke("#3F51B5");
                Pose currentPose = follower.getPose();
                Drawing.drawRobot(packet.fieldOverlay(), new Pose2d(currentPose.getX(), currentPose.getY(), currentPose.getHeading()));
                FtcDashboard.getInstance().sendTelemetryPacket(packet);

                telemetry.addData("timer", timer.seconds());
                telemetry.addData("x", follower.getPose().getX());
                telemetry.addData("y", follower.getPose().getY());
                telemetry.addData("heading", follower.getPose().getHeading());
                telemetry.update();

                return follower.isBusy();
            }
        };
    }



    public void buildPaths() {
        // Path chain to score preload
        PathChain scorePreLoad = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(9.000, 57.000, Point.CARTESIAN),
                                new Point(23.063, 73.260, Point.CARTESIAN),
                                new Point(38.956, 74.229, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        /* Path chain to push floor samples into observation zone*/
        PathChain retrieveGroundSamples = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(38.956, 74.229, Point.CARTESIAN),
                                new Point(0.000, 4.070, Point.CARTESIAN),
                                new Point(105.626, 55.817, Point.CARTESIAN),
                                new Point(65.895, 20.738, Point.CARTESIAN),
                                new Point(17.055, 27.133, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 3
                        new BezierCurve(
                                new Point(17.055, 27.133, Point.CARTESIAN),
                                new Point(109.890, 19.187, Point.CARTESIAN),
                                new Point(17.249, 16.280, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(17.249, 16.280, Point.CARTESIAN),
                                new Point(111.634, 10.078, Point.CARTESIAN),
                                new Point(17.249, 5.427, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        //path from 3rd push to retrieve first specimen from Observation Zone
        PathChain retrieveFirstSpecimen = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(17.249, 5.427, Point.CARTESIAN),
                                new Point(18.799, 24.614, Point.CARTESIAN),
                                new Point(8.334, 23.838, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }
}
