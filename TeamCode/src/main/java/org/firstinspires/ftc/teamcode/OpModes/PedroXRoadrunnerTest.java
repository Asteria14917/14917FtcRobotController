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
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
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

@Autonomous(name="PedroXRoadRunnerTest", group="Linear OpMode")
@Config
public class PedroXRoadrunnerTest extends LinearOpMode {

    private Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
    private Follower follower;
    public ElapsedTime timer = new ElapsedTime();
    private Timer pathTimer, actionTimer, opmodeTimer;

    @Override
    public void runOpMode() throws InterruptedException {
        Scoring scoring = new Scoring(this);
        Lift lift = new Lift(this);
        pathTimer = new Timer();

        Pose startPose = new Pose(0, 0, 0);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        scoring.init();
        lift.init(scoring);

        waitForStart();

        int cycle = 0;

        if (isStopRequested()) return;

        //Actions.runBlocking(pedroDriveToPose(new Pose(24, 0, 0)));

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
