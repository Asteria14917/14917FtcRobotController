package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "Pedro Specimen Auto", group = "Examples")
public class PedroSpecimenAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    enum State {
        START,
        SCORE_PRELOAD,
        RETRIEVE_SAMPLES,
        PARK
    }

    State currentState = State.SCORE_PRELOAD;
    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot, right side aligned with tile edge to left of observation zone */
    private final Pose startPose = new Pose(10, 57, Math.toRadians(0));

    /** Control point so robot heads directly towards chamber*/
    private final Pose scorePoseControlPoint = new Pose(22, 74, Math.toRadians(0));

    /** Scoring Pose of our robot. It is facing the chamber with lift extended*/
    private final Pose scorePose = new Pose(35, 74, Math.toRadians(0));

    /** transition point between chamber and observation zone */
    private final Pose transferControlPoint = new Pose(25, 43, Math.toRadians(0));

    /** transition point to avoid first ground sample */
    private final Pose sample1ControlPoint = new Pose(60, 33, Math.toRadians(0));

    /** point to push first sample */
    private final Pose sample1Pose = new Pose(60, 23, Math.toRadians(0));

    /** placing the first sample */
    private final Pose sample1ObservationZone = new Pose(18, 23, Math.toRadians(0));

    /** transition point to avoid second ground sample */
    private final Pose sample2ControlPoint = new Pose(52, 23, Math.toRadians(0));

    /** point to push second sample */
    private final Pose sample2Pose = new Pose(60, 17, Math.toRadians(0));

    /** placing the second sample */
    private final Pose sample2ObservationZone = new Pose(18, 17, Math.toRadians(0));

    /** transition point to avoid third ground sample */
    private final Pose sample3ControlPoint = new Pose(52, 14, Math.toRadians(0));

    /** point to push third sample */
    private final Pose sample3Pose = new Pose(60, 10, Math.toRadians(0));

    /** placing the third sample */
    private final Pose sample3ObservationZone = new Pose(18, 10, Math.toRadians(0));

    /** Parking Pose of our robot. It is in observation Zone*/
    private final Pose parkPose = new Pose(10, 37, Math.toRadians(0));


    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    //private final Pose parkControlPose = new Pose(60, 98, Math.toRadians(90));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park;
    private PathChain retrieveSamples, retrieveSample2, retrieveSample3;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        retrieveSamples = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose),
                        new Point(transferControlPoint),
                        new Point(sample1ControlPoint),
                        new Point(sample1Pose),
                        new Point(sample1ObservationZone)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .addPath(new BezierCurve(new Point(sample1ObservationZone),
                        new Point(sample2ControlPoint),
                        new Point(sample2Pose),
                        new Point(sample2ObservationZone)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .addPath(new BezierCurve(new Point(sample2ObservationZone),
                        new Point(sample3ControlPoint),
                        new Point(sample3Pose),
                        new Point(sample3ObservationZone)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        /* This is our park path.  */
        park = new Path(new BezierLine(new Point(scorePose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (currentState) {
            case START:
                follower.followPath(scorePreload);
                setPathState(State.SCORE_PRELOAD);
                break;
            case SCORE_PRELOAD:
                /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */
                    follower.followPath(retrieveSamples,true);
                    setPathState(State.RETRIEVE_SAMPLES);
                }
                break;
            case RETRIEVE_SAMPLES:
                /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    follower.followPath(park,true);
                    setPathState(State.PARK);
                }
                break;
            case PARK:

                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(State newState) {
        currentState = newState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("current state", currentState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(State.SCORE_PRELOAD);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}

