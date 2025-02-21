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

    @Autonomous(name="FiveSpecimenAuto", group="Linear OpMode")
    @Config
    public class FiveSpecimenAuto extends LinearOpMode {

        RobotHardware robot;

        ElapsedTime timer = new ElapsedTime();

        // This enum defines our "state"
        // This is essentially just defines the possible steps our program will take
        //TODO Update states to reflect flow of robot actions
        enum State {
            LIFT_OUT,
            SWIPE_SAMPLE,
            DRIVE_TO_SPECIMEN,
            PASS_SPECIMEN,
            FORWARD,
            GET_SPECIMEN,
            DRIVE_TO_START,
            LIFT,
            DRIVE_FORWARD_TWO,
            GET_SAMPLE, GO_TO_BUCKET, DUMP, IDLE
        }

        // We define the current state we're on
        // Default to IDLE
        org.firstinspires.ftc.teamcode.OpModes.FiveSpecimenAuto.State currentState = org.firstinspires.ftc.teamcode.OpModes.FiveSpecimenAuto.State.LIFT_OUT;

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
            //robot.scoring.claw.setPosition(Scoring.CLAW_CLOSED);
            robot.scoring.clawPivot.setPosition(Scoring.WRIST_IN);


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
            score = 0;
            waitForStart();
            timer.reset();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive() && !isStopRequested()) {
                switch (currentState){
                    case LIFT_OUT:
                        //robot.scoring.claw.setPosition(Scoring.CLAW_CLOSED);
                        robot.lift.liftMode = Lift.LiftMode.HIGH_CHAMBER;
                        robot.scoring.clawPivot.setPosition(Scoring.WRIST_AUTO);
                        if(timer.seconds() > 0.5) {
                            currentState = org.firstinspires.ftc.teamcode.OpModes.FiveSpecimenAuto.State.SWIPE_SAMPLE;
                            robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 28, 3, AngleUnit.DEGREES, 0));
                            timer.reset();
                        }
                        break;
                    case SWIPE_SAMPLE:
                        if(timer.seconds() > 1){
                            //2
                            //robot.scoring.claw.setPosition(Scoring.CLAW_OPEN);
                            robot.lift.liftMode = Lift.LiftMode.RETRACTED;
                        }
                        //put condition for switch at the beginning, condition can be based on time or completion of a task
                        if(timer.seconds() > 1.5 && score < 4){
                            //currentState = org.firstinspires.ftc.teamcode.OpModes.FiveSpecimenAuto.State.PASS_SPECIMEN;
                            robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 15, -24, AngleUnit.DEGREES, 0));
                            robot.scoring.pivotMode = Scoring.PivotMode.PIVOT_SUBMERSIBLE;
                            robot.lift.liftMode = Lift.LiftMode.HIGH_CHAMBER;
                            timer.reset();
                        } else if(timer.seconds() > 2 && score < 4){
                            robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 15, -24, AngleUnit.DEGREES, 100));

                        }
                        else if(timer.seconds() > 1.5 && score < 0){
                            //2.5
                            currentState = org.firstinspires.ftc.teamcode.OpModes.FiveSpecimenAuto.State.DRIVE_TO_SPECIMEN;
                            robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 3,-30, AngleUnit.DEGREES, -180));
                            timer.reset();
                        }
                        break;
                    case PASS_SPECIMEN:
                        if(timer.seconds() < 1){
                            robot.drivetrain.driveToTarget(new Pose2D(DistanceUnit.INCH, 22, -24, AngleUnit.DEGREES, 0));
                        }else if(timer.seconds() < 2){
                            robot.drivetrain.driveToTarget(new Pose2D(DistanceUnit.INCH, 50, -30, AngleUnit.DEGREES, 0));
                        }else if(timer.seconds() < 2.5){
                            robot.drivetrain.driveToTarget(new Pose2D(DistanceUnit.INCH, 50, -40, AngleUnit.DEGREES, 0));
                        }else if(timer.seconds() < 3.5){
                            robot.drivetrain.driveToTarget(new Pose2D(DistanceUnit.INCH, 6, -40, AngleUnit.DEGREES, 0));
                        }else if(timer.seconds() < 4.5){
                            robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 6, -30, AngleUnit.DEGREES, -180));
                            currentState = org.firstinspires.ftc.teamcode.OpModes.FiveSpecimenAuto.State.DRIVE_TO_SPECIMEN;
                            timer.reset();
                        }
                        break;
                    case DRIVE_TO_SPECIMEN:
                        robot.scoring.clawPivot.setPosition(Scoring.WRIST_OUT);
                        if(timer.seconds() < 2){
                            robot.drivetrain.driveToTarget(new Pose2D(DistanceUnit.INCH, 8, -30, AngleUnit.DEGREES, -180));
                        }else if(timer.seconds() < 3){
                            robot.drivetrain.driveToTarget(new Pose2D(DistanceUnit.INCH, 1, -30, AngleUnit.DEGREES, -180));
                        }else{
                            currentState = org.firstinspires.ftc.teamcode.OpModes.FiveSpecimenAuto.State.GET_SPECIMEN;
                            timer.reset();
                        }
                        break;
                    case GET_SPECIMEN:
                        if(timer.seconds() > .5){
                            //robot.scoring.claw.setPosition(Scoring.CLAW_CLOSED);
                        }
                        //306
                        if(timer.seconds() > 1){
                            currentState = org.firstinspires.ftc.teamcode.OpModes.FiveSpecimenAuto.State.DRIVE_TO_START;
                            robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 0,3, AngleUnit.DEGREES, 0));
                            timer.reset();
                        }
                        break;
                    case DRIVE_TO_START:
                        robot.scoring.clawPivot.setPosition(Scoring.WRIST_AUTO);
                        //robot.scoring.claw.setPosition(Scoring.CLAW_CLOSED);
                        if(timer.seconds() > 1.5){
                            robot.lift.liftMode = Lift.LiftMode.HIGH_CHAMBER;
                        }
                        if(robot.drivetrain.targetReached || timer.seconds() > 2.3){
                            currentState = org.firstinspires.ftc.teamcode.OpModes.FiveSpecimenAuto.State.SWIPE_SAMPLE;
                            robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 28, 8+score*2, AngleUnit.DEGREES, 0));
                            timer.reset();
                            if(score < 2){
                                score++;
                                currentState = org.firstinspires.ftc.teamcode.OpModes.FiveSpecimenAuto.State.SWIPE_SAMPLE;
                            }else{
                                currentState = org.firstinspires.ftc.teamcode.OpModes.FiveSpecimenAuto.State.DRIVE_FORWARD_TWO;
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
                        break;
                    case IDLE:
                        robot.lift.liftMode = Lift.LiftMode.RETRACTED;
                        //robot.scoring.claw.setPosition(Scoring.CLAW_OPEN);
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


