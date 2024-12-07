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

@Autonomous(name="Meet2Auto", group="Linear OpMode")
@Config
public class Meet2Auto extends LinearOpMode {

    RobotHardware robot;

    ElapsedTime timer = new ElapsedTime();

    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    //TODO Update states to reflect flow of robot actions
    enum State {
        LIFT_OUT,
        DRIVE_FORWARD,
        DRIVE_TO_SPECIMEN,
        GET_SPECIMEN,
        DRIVE_TO_START,
        DRIVE_FORWARD_TWO,
        IDLE
    }

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.LIFT_OUT;

    // Define our start pose
     Pose2D startPose = new Pose2D(DistanceUnit.INCH, 0,0, AngleUnit.DEGREES,0);

    // Define our target
    public static double targetX = 0;
    public static double targetY = 0;
    public static double targetHeading = 0;
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
                case LIFT_OUT:
                    robot.lift.liftMode = Lift.LiftMode.HIGH_CHAMBER;
                    robot.scoring.clawPivot.setPosition(Scoring.WRIST_MID);
                    robot.scoring.claw.setPosition(Scoring.CLAW_CLOSED);
                    if(timer.seconds() > 2) {
                     currentState = State.DRIVE_FORWARD;
                     robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 26, 3, AngleUnit.DEGREES, 0));
                    }
                    break;
                case DRIVE_FORWARD:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    if(timer.seconds() > 3){
                        currentState = State.DRIVE_TO_SPECIMEN;
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 5,-30, AngleUnit.DEGREES, -180));
                        timer.reset();
                    }
                    break;
                case DRIVE_TO_SPECIMEN:
                    robot.scoring.claw.setPosition(Scoring.CLAW_OPEN);
                    robot.lift.liftMode = Lift.LiftMode.RETRACTED;
                    robot.scoring.clawPivot.setPosition(Scoring.WRIST_AUTO);
                    if(robot.drivetrain.targetReached || timer.seconds() > 3) {
                        currentState = State.GET_SPECIMEN;
                        timer.reset();
                    }
                    break;
                case GET_SPECIMEN:
                    robot.scoring.claw.setPosition(Scoring.CLAW_CLOSED);
                    //306
                    if(timer.seconds() > 2.0){
                        currentState = State.DRIVE_TO_START;
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 6,3, AngleUnit.DEGREES, 0));
                        timer.reset();
                    }
                    break;
                case DRIVE_TO_START:
                    robot.lift.liftMode = Lift.LiftMode.HIGH_CHAMBER;
                    robot.scoring.clawPivot.setPosition(Scoring.WRIST_MID);
                    robot.scoring.claw.setPosition(Scoring.CLAW_CLOSED);
                    if(robot.drivetrain.targetReached || timer.seconds() > 3.0){
                        currentState = State.DRIVE_FORWARD_TWO;
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 26, 3, AngleUnit.DEGREES, 0));
                        timer.reset();
                    }
                    break;
                case DRIVE_FORWARD_TWO:
                    if(timer.seconds() > 4){
                        currentState = State.IDLE;
                        robot.drivetrain.setTargetPose(new Pose2D (DistanceUnit.INCH, targetX, targetY, AngleUnit.DEGREES,targetHeading));
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
