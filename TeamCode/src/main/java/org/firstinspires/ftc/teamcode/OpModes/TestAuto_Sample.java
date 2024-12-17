package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Autonomous(name="TestAuto", group="Linear OpMode")
@Config
public class TestAuto_Sample extends LinearOpMode {

    RobotHardware robot;

    ElapsedTime timer = new ElapsedTime();

    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    //TODO Update states to reflect flow of robot actions
    enum State {
        DRIVE_TO_BASKET,
        SCORE_ONE,
        DRIVE_TO_SAMPLE,
        IDLE,
    }

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.DRIVE_TO_BASKET;

    // Define our start pose
     Pose2D startPose = new Pose2D(DistanceUnit.INCH, 0,0, AngleUnit.DEGREES,0);

    // Define our target
    public static double basketX = 3;
    public static double basketY = 24;
    public static double basketHeading = -45;
    Pose2D targetPose = new Pose2D(DistanceUnit.INCH, basketX,basketY, AngleUnit.DEGREES, basketHeading);

    public static double sampleX = 24;
    public static double sampleY = 16;
    public static double sampleHeading = 0;

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
                case DRIVE_TO_BASKET:
                    robot.drivetrain.driveToTarget(new Pose2D (DistanceUnit.INCH, basketX, basketY, AngleUnit.DEGREES,basketHeading));
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    if(robot.drivetrain.targetReached||timer.seconds()>2){
                        currentState = State.SCORE_ONE;
                        timer.reset();
                    }
                    break;
                case SCORE_ONE:
                    if(timer.seconds() > 2.0){
                        currentState = State.DRIVE_TO_SAMPLE;
                    }
                    break;
                case DRIVE_TO_SAMPLE:
                    robot.drivetrain.driveToTarget(new Pose2D (DistanceUnit.INCH, sampleX, sampleY, AngleUnit.DEGREES,sampleHeading));
                    if(robot.drivetrain.targetReached){
                        currentState = State.IDLE;
                        timer.reset();
                    }
                    break;
                case IDLE:

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
