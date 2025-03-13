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
            DRIVE_FORWARD,
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
        public static int specimennum = 0;
        Pose2D targetPose = new Pose2D(DistanceUnit.INCH, targetX,targetY, AngleUnit.DEGREES, targetHeading);

        @Override
        public void runOpMode() {
            //calling constructor
            robot = new RobotHardware(this);


            //calling init function
            robot.init();
            robot.scoring.clawRotate.setPosition(Scoring.CLAW_ROTATE_SPECIMEN);
            //robot.scoring.claw.setPosition(Scoring.CLAW_CLOSED);
           // robot.scoring.clawPivot.setPosition(Scoring.WRIST_IN);


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
            specimennum = 0;
            waitForStart();
            timer.reset();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive() && !isStopRequested()) {
                telemetry.addData("specimennum", specimennum);
                switch (currentState){
                    case LIFT_OUT:
                        robot.scoring.claw.setPosition(Scoring.CLAW_CLOSED);
                        //robot.lift.liftMode = Lift.LiftMode.HIGH_CHAMBER;
                        robot.scoring.clawPivot.setPosition(Scoring.WRIST_SPECIMEN);
                        //was wrist auto
                        robot.scoring.clawRotate.setPosition(Scoring.CLAW_ROTATE_SPECIMEN);
                        robot.lift.liftMode = Lift.LiftMode.HIGH_CHAMBER;
                        //robot.scoring.pivotMode = Scoring.PivotMode.PIVOT_UP_AUTO;
                        if(timer.seconds() > 0.7) {
                            currentState = org.firstinspires.ftc.teamcode.OpModes.FiveSpecimenAuto.State.DRIVE_FORWARD;
                            robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 26.5, 2, AngleUnit.DEGREES, 0));//27 //2
                            timer.reset();
                        }
                        break;
                    case DRIVE_FORWARD:
                        if(timer.seconds()>0.8){
                            //0.8
                            robot.scoring.clawPivot.setPosition(Scoring.WRIST_IN);
                            robot.scoring.claw.setPosition(Scoring.CLAW_OPEN);
                        }
                        if(timer.seconds() > 1){
                            //1
                        //2
                       // robot.scoring.claw.setPosition(Scoring.CLAW_OPEN);

                            // robot.scoring.pivotMode = Scoring.PivotMode.PIVOT_DOWN_AUTO;
                        robot.lift.liftMode = Lift.LiftMode.RETRACTED;

                        }
                        //put condition for switch at the beginning, condition can be based on time or completion of a task
                        if(timer.seconds() > 1 && score == 0){//1
                            currentState = FiveSpecimenAuto.State.PASS_SPECIMEN;
                            robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 10, -24, AngleUnit.DEGREES, 0));
                            timer.reset();
                        } else if(timer.seconds() > 1 && score > 0 && score < 4){//1.5 //1
                            //2.5
                            currentState = FiveSpecimenAuto.State.DRIVE_TO_SPECIMEN;
                            robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 3,-30, AngleUnit.DEGREES, 0));//-180
                            timer.reset();
                        } else if (timer.seconds() > 1 && score >= 4){
                            currentState = State.DRIVE_FORWARD_TWO;
                            robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 3,-30, AngleUnit.DEGREES, 0));//-180
                            timer.reset();
                        }
                        break;
                    case PASS_SPECIMEN:
                        if(timer.seconds() < 0.7 && specimennum == 0){
                            robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 10, -24, AngleUnit.DEGREES, 0));
                        }else if(timer.seconds() < 1.3 && specimennum > 0){ //1.3
                            robot.drivetrain.driveToTarget(new Pose2D(DistanceUnit.INCH, 53, -24-(specimennum*8.5), AngleUnit.DEGREES, 0));
                            //53
                        }else if(timer.seconds() < 1.6){ //1.6
                            robot.drivetrain.driveToTarget(new Pose2D(DistanceUnit.INCH, 53, -29-(specimennum*8.7), AngleUnit.DEGREES, 0));
                        }else if(timer.seconds() < 1.9){ //1.9
                            robot.drivetrain.driveToTarget(new Pose2D(DistanceUnit.INCH, 53, -39-(specimennum*8.7), AngleUnit.DEGREES, 0));
                        }else if(timer.seconds() < 3.1){ //3.1
                            robot.drivetrain.driveToTarget(new Pose2D(DistanceUnit.INCH, 7.95, -39-(specimennum*8.7), AngleUnit.DEGREES, 0));
                        }else if(specimennum < 2){
                            timer.reset();
                            specimennum++;
                        }else{
                            currentState = State.DRIVE_TO_SPECIMEN;
                            timer.reset();
                        }
                        /*else if(timer.seconds() < 4.5) {
                            robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 50, -24-(specimennum+1), AngleUnit.DEGREES, 0));
                            currentState = State.SWIPE_SAMPLE;
                            timer.reset();
                        }

                         */

                        break;
                    case DRIVE_TO_SPECIMEN:
                        //robot.scoring.clawPivot.setPosition(Scoring.WRIST_OUT);
                        robot.scoring.pivotMode = Scoring.PivotMode.OBSERVATION_ZONE;
                        robot.scoring.claw.setPosition(Scoring.CLAW_OPEN);//7.6
                        robot.lift.liftMode = Lift.LiftMode.RETRACTED;
                        robot.drivetrain.driveToTarget(new Pose2D(DistanceUnit.INCH, 8.1, -40.5, AngleUnit.DEGREES, 0));//15//6.9 //8.1

                        if(timer.seconds() < 1){ //1.5
                            //1.5
                            robot.scoring.clawPivot.setPosition(Scoring.WRIST_IN);
                           // robot.drivetrain.driveToTarget(new Pose2D(DistanceUnit.INCH, 8, -39, AngleUnit.DEGREES, 0));//15
                            robot.scoring.clawRotate.setPosition(Scoring.CLAW_ROTATE_SPECIMEN);
                        }else if(timer.seconds() < 1.2 && score == 4){ //1.7 //1.5
                            robot.scoring.pivotMode = Scoring.PivotMode.PIVOT_ZERO;
                        }
                        /*
                        else if(timer.seconds() < 1.5 && score > 0){
                            robot.drivetrain.driveToTarget(new Pose2D(DistanceUnit.INCH, 7,-39, AngleUnit.DEGREES, 0));
                        }

                         */
                       else{
                            currentState = org.firstinspires.ftc.teamcode.OpModes.FiveSpecimenAuto.State.GET_SPECIMEN;
                            timer.reset();
                        }
                        break;
                    case GET_SPECIMEN:
                        if(timer.seconds() > 0.58){
                            //1
                            robot.scoring.claw.setPosition(Scoring.CLAW_CLOSED);
                        }
                        //306
                        if(timer.seconds() > 1){ //1
                            //1.3
                            currentState = org.firstinspires.ftc.teamcode.OpModes.FiveSpecimenAuto.State.DRIVE_TO_START;
                            robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 0,3, AngleUnit.DEGREES, 0));
                            timer.reset();
                        }
                        break;
                    case DRIVE_TO_START:
                        robot.scoring.clawPivot.setPosition(Scoring.WRIST_SPECIMEN);
                        robot.scoring.pivotMode = Scoring.PivotMode.PIVOT_ZERO;
                        //robot.scoring.claw.setPosition(Scoring.CLAW_CLOSED);
                        if(timer.seconds() > 0.5){
                            robot.lift.liftMode = Lift.LiftMode.HIGH_CHAMBER;
                        }
                        if(robot.drivetrain.targetReached || timer.seconds() > 1.4){//1.5
                            currentState = org.firstinspires.ftc.teamcode.OpModes.FiveSpecimenAuto.State.DRIVE_FORWARD;
                            robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 26.5, score*6+5, AngleUnit.DEGREES, 0)); //3.8
                            timer.reset();
                            if(score < 4){
                                score++;
                                currentState = org.firstinspires.ftc.teamcode.OpModes.FiveSpecimenAuto.State.DRIVE_FORWARD;
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
                        robot.scoring.clawPivot.setPosition(Scoring.WRIST_IN);
                        robot.lift.liftMode = Lift.LiftMode.RETRACTED;
                        robot.scoring.pivotMode = Scoring.PivotMode.PIVOT_ZERO;
                        break;
                    case IDLE:

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


