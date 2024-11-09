package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.List;


@Autonomous(name = "LeftRegionals", group = "Linear Opmode")

public class AutoMeet1 extends LinearOpMode {

    private static final String[] LABELS = {
            "Marlbots",
            "M"
    };
    Robot robot = new Robot(this);
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor pivot = null;
    private Servo clawPivot = null;
    private Servo claw = null;

    private String position = "RIGHT";

    public void runOpMode() {
        robot.init();
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        pivot = hardwareMap.get(DcMotor.class, "pivot");
        clawPivot = hardwareMap.get(DcMotor.class, "clawPivot");
        claw = hardwareMap.get(DcMotor.class, "claw");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        waitForStart();

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
        }

        runtime.reset();
        while (!isStarted() && !isStopRequested()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());

                        if (updatedRecognitions.size() >= 2 && recognition.getLabel() == "M")
                            position = "LEFT";
                        else if (updatedRecognitions.size() == 1 && recognition.getLabel() == "M")
                            position = "MID";
                        else if (updatedRecognitions.size() < 1)
                            position = "RIGHT";
                        telemetry.addData("position", position);

                        i++;
                    }
                    telemetry.update();
                }
            }
        }

        robot.drivetrain.resetEncoders();
        robot.drivetrain.useEncoders();

        waitForStart();

        robot.Drivetrain.resetEncoders();
        robot.drivetrain.useEncoders();
        robot.drivetrain.driveSideProfiledPID(3250);
        robot.drivetrain.resetEncoders();
        robot.drivetrain.useEncoders();
        robot.drivetrain.driveStraightProfiledPID(-700);
        robot.drivetrain.resetEncoders();
        robot.drivetrain.useEncoders();

        robot.toJunctionLeft();
        robot.lift.resetLiftPID();

        robot.deliver();

        robot.toStackLeft(575);
        robot.lift.resetLiftPID();

        robot.retrieve();

        robot.toJunctionLeft();
        robot.lift.resetLiftPID();

        robot.deliver();

        robot.toStackLeft(425);
        robot.lift.resetLiftPID();

        robot.retrieve();

        robot.toJunctionLeft();
        robot.lift.resetLiftPID();

        robot.deliver();

        runtime.reset();
        robot.turret.newTarget(-30.0);
        while (runtime.seconds() < 2 && opModeIsActive()) {
            robot.turret.turretProfiledPIDNoLoop(robot.turret.targetAngle, robot.turret.startingAngle, runtime.seconds());
            robot.lift.resetLift(-1);
            robot.scoring.pivot.setPosition(robot.scoring.CLAW_UP);
            robot.scoring.extension.setPosition(robot.scoring.EXTENSION_IN);
            telemetry.update();
        }
        robot.drivetrain.resetEncoders();
        robot.drivetrain.useEncoders();

        robot.drivetrain.encoderTurn(18f, 1);

        robot.drivetrain.resetEncoders();
        robot.drivetrain.useEncoders();
        if (position == "RIGHT") {
            robot.drivetrain.driveSideProfiledPID(-2100);
        } else if (position == "MID") {
            robot.drivetrain.driveSideProfiledPID(-600);
        } else {
            robot.drivetrain.driveSideProfiledPID(800);
        }
    }
    /**
     * Initialize the TensorFlow Object Detection engine.
     */
}