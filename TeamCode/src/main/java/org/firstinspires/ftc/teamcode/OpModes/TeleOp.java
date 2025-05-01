package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Regionals TeleOp", group="Linear OpMode")
public class TeleOp extends LinearOpMode {

    Drivetrain drivetrain;
    Lift lift;
    Scoring scoring;
    ElapsedTime timer;
    boolean ascending;


    @Override
    public void runOpMode() {

        drivetrain = new Drivetrain(this);
        lift = new Lift(this);
        scoring = new Scoring(this);
        timer = new ElapsedTime();
        ascending = false;

        drivetrain.init();
        scoring.init();
        lift.init(scoring);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            drivetrain.teleOp();
            lift.teleOp();
            scoring.teleOp();
            telemetry.update();
            if(scoring.pivotMode == Scoring.PivotMode.MANUAL){
                //prevent pivot from going past vertical when lift is extended
                if(scoring.pivot.getCurrentPosition() > 1300 && lift.leftLift.getCurrentPosition() > 200 && -gamepad2.left_stick_y > .1){
                    scoring.pivot.setPower(0);
                }
            }
            if (gamepad1.y) {
                timer.reset();
                ascending = true;
                lift.liftMode = Lift.LiftMode.ASCENT;
                scoring.pivotMode = Scoring.PivotMode.ASCENT;
            }
            if(gamepad1.x){
                timer.reset();
                ascending = false;
            }
            if (ascending) {
                if (timer.seconds() < 1) {
                    //align angle of stationary hooks
                    scoring.pivotToTargetPosition(0.8, -1822);
                } else if (timer.seconds() < 3) {
                    //align height of stationary hooks
                    lift.liftToPositionPIDClass(100);
                    scoring.pivotToTargetPosition(0.8, -1622);
                } else if (timer.seconds() < 4) {
                    //attach stationary hooks
                    scoring.pivotToTargetPosition(0.8, -500);
                } else if(timer.seconds()< 6){
                    //extend lift past high bar
                    lift.liftToPositionPIDClass(3000);
                } else if (timer.seconds() < 8) {
                    //pivot down to align lift hooks with high bar
                    scoring.pivotToTargetPosition(0.9, -1013);
                } else if (timer.seconds()< 9){
                    //retract lift to hook onto high bar
                    lift.liftToPositionPIDClass(2000);
                }else if (timer.seconds() < 11) {
                    //retract and pivot
                    lift.liftToPositionPIDClass(0);
                    scoring.pivotToTargetPosition(0.9, 1100);
                } else if (timer.seconds() < 13) {
                    //pivot to get wheel over low bar
                    lift.liftToPositionPIDClass(0);
                    scoring.pivotToTargetPosition(0.8, -81);
                } else {
                    //align robot on high bar
                    lift.liftToPositionPIDClass(0);
                    scoring.pivotToTargetPosition(0.8, 400);
                }
            }
        }
    }
}
