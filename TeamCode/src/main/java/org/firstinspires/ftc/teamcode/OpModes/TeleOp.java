package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Meet 2 TeleOp", group="Linear OpMode")
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
            if(gamepad2.y){
                timer.reset();
                lift.liftMode = Lift.LiftMode.ASCENT;
                scoring.pivotMode = Scoring.PivotMode.ASCENT;
            }
            if(ascending){
                if(timer.seconds()<5){
                    scoring.pivotToTargetPosition(0.8,-1622);
                }
                else if (timer.seconds()<10){
                    lift.liftToPositionPIDClass(150);
                }
                else if(timer.seconds()<15){
                    scoring.pivotToTargetPosition(0.8,-500);
                }
                else if (timer.seconds()<20){
                    lift.liftToPositionPIDClass(2600);
                }
                else if (timer.seconds()<25){
                    scoring.pivotToTargetPosition(0.8, -600);
                }
                else if (timer.seconds()<30){
                    scoring.pivotToTargetPosition(0.8,-700);
                }
                else if (timer.seconds()<35){
                    scoring.pivotToTargetPosition(0.8, 900);
                }
                else if (timer.seconds()<40){
                    scoring.pivotToTargetPosition(0.8, -81);
                }
                else if (timer.seconds()<45){
                    lift.liftToPositionPIDClass(2370);
                }
                else if (timer.seconds()<50){
                    lift.liftToPositionPIDClass(0);
                }
                else if (timer.seconds()<55){
                    scoring.pivotToTargetPosition(0.8, 5);
                }
                else if (timer.seconds()<60){
                    scoring.pivotToTargetPosition(0.8, 8);
                }
    }
}
