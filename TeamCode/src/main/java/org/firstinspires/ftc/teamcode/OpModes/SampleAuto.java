package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring;

    @Autonomous(name="SampleAuto", group="Linear OpMode")
    @Config
    public class SampleAuto extends LinearOpMode {

        RobotHardware robot;

        ElapsedTime timer = new ElapsedTime();

        // This enum defines our "state"
        // This is essentially just defines the possible steps our program will take
        //TODO Update states to reflect flow of robot actions
        enum State {
            DRIVE_FORWARD,
            GET_SAMPLE,
            GO_TO_BUCKET,
            DUMP,
            DRIVE_TO_START,
            LIFT,
            DRIVE_FORWARD_TWO,
            IDLE
        }

        // We define the current state we're on
        // Default to IDLE
        org.firstinspires.ftc.teamcode.OpModes.SampleAuto.State currentState = org.firstinspires.ftc.teamcode.OpModes.SampleAuto.State.DRIVE_FORWARD;

        // Define our start pose
        Pose2D startPose = new Pose2D(DistanceUnit.INCH, 0,0, AngleUnit.DEGREES,0);

        // Define our target
        public static double targetX = 0;
        public static double targetY = 0;
        public static double targetHeading = 0;
        public static int score = 0;
        Pose2D targetPose = new Pose2D(DistanceUnit.INCH, targetX,targetY, AngleUnit.DEGREES, targetHeading);

        @Override
        public void runOpMode() {
            //calling constructor
            robot = new RobotHardware(this);


            //calling init function
            robot.init();

            //TODO Pass starting pose to localizer
            //for Gobilda it looks like this
            robot.drivetrain.localizer.odo.setPosition(startPose);
            //for sparkfun it looks like this
            //robot.drivetrain.localizer.myOtos.setPosition(startPose);

            //Set the drivetrain's first target
            robot.drivetrain.setTargetPose(targetPose);

            // Wait for the game to start (driver presses START)
            telemetry.addData("Status", "Waiting for Start");
            telemetry.update();
            waitForStart();
            timer.reset();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive() && !isStopRequested()) {
                switch (currentState){
                    case DRIVE_FORWARD:
                        robot.scoring.claw.setPosition(Scoring.CLAW_OPEN);
                        if(timer.seconds() > 2) {
                            currentState = org.firstinspires.ftc.teamcode.OpModes.SampleAuto.State.GET_SAMPLE;
                            robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 24, 20, AngleUnit.DEGREES, 0));
                            timer.reset();
                        }
                        break;
                    case GET_SAMPLE:
                        robot.scoring.pivotMode = Scoring.PivotMode.PIVOT_SUBMERSIBLE;
                        robot.lift.liftMode = Lift.LiftMode.RETRACTED;
                        robot.scoring.clawPivot.setPosition(Scoring.WRIST_IN);
                        //put condition for switch at the beginning, condition can be based on time or completion of a task
                        if(timer.seconds() > 2){
                            robot.scoring.claw.setPosition(Scoring.CLAW_CLOSED);
                            currentState = org.firstinspires.ftc.teamcode.OpModes.SampleAuto.State.GO_TO_BUCKET;
                            robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, -20,12, AngleUnit.DEGREES, -45));
                            timer.reset();
                        }
                        break;
                    case GO_TO_BUCKET:
                        robot.scoring.pivotMode = Scoring.PivotMode.PIVOT_HIGH_BASKET;
                        robot.lift.liftMode = Lift.LiftMode.HIGH_BASKET;
                        robot.scoring.clawPivot.setPosition(Scoring.WRIST_OUT);
                        robot.scoring.claw.setPosition(Scoring.CLAW_OPEN);
                        if(robot.drivetrain.targetReached || timer.seconds() > 3) {
                            currentState = org.firstinspires.ftc.teamcode.OpModes.SampleAuto.State.IDLE;
                            timer.reset();
                        }
                        break;
                    case DUMP:
                        robot.scoring.claw.setPosition(Scoring.CLAW_CLOSED);
                        //306
                        if(timer.seconds() > 2.0){
                            currentState = org.firstinspires.ftc.teamcode.OpModes.SampleAuto.State.DRIVE_TO_START;
                            robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 0,3, AngleUnit.DEGREES, 0));
                            timer.reset();
                        }
                        break;
                    case DRIVE_TO_START:
                        robot.scoring.clawPivot.setPosition(Scoring.WRIST_MID);
                        robot.scoring.claw.setPosition(Scoring.CLAW_CLOSED);
                        if(timer.seconds() > 3){
                            robot.lift.liftMode = Lift.LiftMode.HIGH_CHAMBER;
                        }
                        if(robot.drivetrain.targetReached || timer.seconds() > 4.0){
                            currentState = org.firstinspires.ftc.teamcode.OpModes.SampleAuto.State.DRIVE_FORWARD;
                            robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 24, 8+score*2, AngleUnit.DEGREES, 0));
                            timer.reset();
                            if(score < 2){
                                score++;
                                currentState = org.firstinspires.ftc.teamcode.OpModes.SampleAuto.State.DRIVE_FORWARD;
                            }else{
                                currentState = org.firstinspires.ftc.teamcode.OpModes.SampleAuto.State.DRIVE_FORWARD_TWO;
                            }
                        }
                        break;
                    /*
                case LIFT:
                    if(robot.drivetrain.targetReached || timer.seconds() > 1){
                        currentState = State.DRIVE_FORWARD_TWO;
                        timer.reset();
                    }
                    */
                    case DRIVE_FORWARD_TWO:
                        if(timer.seconds() > 1){
                            currentState = org.firstinspires.ftc.teamcode.OpModes.SampleAuto.State.IDLE;
                            robot.drivetrain.setTargetPose(new Pose2D (DistanceUnit.INCH, targetX, -30, AngleUnit.DEGREES, targetHeading));
                            timer.reset();
                        }
                        break;
                    case IDLE:
                        robot.lift.liftMode = Lift.LiftMode.RETRACTED;
                        robot.scoring.claw.setPosition(Scoring.CLAW_OPEN);
                        break;
                }

                // Anything outside of the switch statement will run independent of the currentState
                // We update robot continuously in the background, regardless of state
                robot.update();

                telemetry.addData("state", currentState);
                telemetry.update();

            }
        }

    }

