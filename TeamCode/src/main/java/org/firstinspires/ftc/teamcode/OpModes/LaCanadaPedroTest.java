package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

@Autonomous(name = "AutoTest", group = "Examples")
//@Disabled
public class LaCanadaPedroTest extends LinearOpMode {

    Pose startPose;
    private Telemetry telemetryA;

    private Follower follower;

    //**TODO firstPath through eighthPath should be renamed to be more descriptive.
    private PathChain firstPath;

    private PathChain secondPath;

    private PathChain thirdPath;
    private PathChain fourthPath;
    private PathChain fifthPath;
    private PathChain sixthPath;
    private PathChain seventhPath;
    private PathChain eighthPath;

    private PathChain forwardToSpecimen;
    private PathChain toSubmersible1;
    private PathChain toSubmersible2;
    private PathChain toSubmersible3;
    private PathChain toSubmersible4;

    private PathChain toObZone1;
    private PathChain toObZone2;
    private PathChain toObZone3;
    private PathChain park;

    @Override
    public void runOpMode() {

        startPose = new Pose(8.44, -62.5, Math.toRadians(90));
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        //**TODO Each Pose should be defined as a variable the way startPose is.
        // Pose endFirstApproachSubmersiblePose = new Pose(11, -39, Math.toRadians(90));
        // And then refer to each Pose and create Points, see --
        // https://pedropathing.com/pedro/examples/auto.html

        // First approach to submersible.
        Pose endFirstApproachSubmersiblePose = new Pose(11, -39, Math.toRadians(90));
        firstPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(endFirstApproachSubmersiblePose))) //new Point(11, -39, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(startPose.getHeading(), endFirstApproachSubmersiblePose.getHeading())
                .build();

        secondPath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(follower.getPose()), new Point(20, -55, Point.CARTESIAN), new Point(30, -40, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                .build();

        thirdPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(30, -40, Point.CARTESIAN), new Point(39, -50, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(-45))
                .build();

        fourthPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(39, -50, Point.CARTESIAN), new Point(49, -34, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(45))
                .build();

        fifthPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(49, -34, Point.CARTESIAN), new Point(49, -50, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(-45))
                .build();

        sixthPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(49, -50), new Point(57, -26, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .build();

        seventhPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(57, -26, Point.CARTESIAN), new Point(53, -50, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .build();

        eighthPath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(53, -50, Point.CARTESIAN), new Point(50, -36, Point.CARTESIAN), new Point(30, -58, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .build();

        forwardToSpecimen = follower.pathBuilder()
                .addPath(new BezierLine(new Point(30, -58, Point.CARTESIAN), new Point(38, -58, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        toSubmersible1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(38, -58, Point.CARTESIAN), new Point(10, -60, Point.CARTESIAN), new Point(8, -34, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        toObZone1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(8, -34, Point.CARTESIAN), new Point(38, -58, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();

        toSubmersible2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(38, -58, Point.CARTESIAN), new Point(8, -60, Point.CARTESIAN), new Point(5, -34, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        toObZone2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(5, -34, Point.CARTESIAN), new Point(38, -58, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();

        toSubmersible3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(38, -58, Point.CARTESIAN), new Point(6, -60, Point.CARTESIAN), new Point(3, -34, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        toObZone3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(3, -34, Point.CARTESIAN), new Point(38, -58, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();

        toSubmersible4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(38, -58, Point.CARTESIAN), new Point(3, -60, Point.CARTESIAN), new Point(0, -34, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(0, -34, Point.CARTESIAN), new Point(55, -55, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        waitForStart();

        // Start the first approach to the submersible.
        follower.followPath(firstPath, true);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();

        // Wait for the first approach to the submersible to complete.
        waitForPathCompletion();
        sleep(2000);

        // Back off from the submersible.
        //**TODO You can start moving your peripherals such as an elevator
        // now, in advance of moving the robot, or after the next line,
        // i.e. after the robot has started moving.
        follower.followPath(secondPath, true);
        waitForPathCompletion();
        //**TODO You'll also want to wait for your peripheral movement
        // to complete.

        //**TODO uncomment after you have completed the renaming
        // and follow the same pattern as above.
        //follower.followPath(thirdPath, true);
        //waitForPathCompletion();

        //follower.followPath(fourthPath, true);
        //waitForPathCompletion();

        //follower.followPath(fifthPath, true);
        //waitForPathCompletion();

        //follower.followPath(sixthPath, true);
        //waitForPathCompletion();

        //follower.followPath(seventhPath, true);
        //waitForPathCompletion();

        //follower.followPath(eighthPath, true);
        //waitForPathCompletion();

        //follower.followPath(forwardToSpecimen, true);
        //waitForPathCompletion();

        //follower.followPath(toSubmersible1, true);
        //waitForPathCompletion();

        //follower.followPath(toObZone1, true);
        //waitForPathCompletion();

        //follower.followPath(toSubmersible2, true);
        //waitForPathCompletion();

        //follower.followPath(toObZone2, true);
        //waitForPathCompletion();

        //follower.followPath(toSubmersible3, true);
        //waitForPathCompletion();

        //follower.followPath(toObZone3, true);
        //waitForPathCompletion();

        //follower.followPath(toSubmersible4, true);
        //waitForPathCompletion();

        //follower.followPath(park, true);
        //waitForPathCompletion();
    }

    // Returns when the follower is not busy, i.e. the path is complete.
    private void waitForPathCompletion() {
        while (follower.isBusy()) {
            follower.update();
            follower.telemetryDebug(telemetryA);
        }
    }
}
