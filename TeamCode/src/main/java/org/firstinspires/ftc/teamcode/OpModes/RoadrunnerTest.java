package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring;


@Autonomous(name="RoadRunnerTest", group="Linear OpMode")
@Config
public class RoadrunnerTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Scoring scoring = new Scoring(this);
        Lift lift = new Lift(this);
        scoring.init();
        lift.init(scoring);
        waitForStart();

        //Actions.runBlocking(scoring.clawToObservationZone());
        //Actions.runBlocking(scoring.clawSubmersible());
        //Actions.runBlocking(scoring.pivotAction(Scoring.PIVOT_OBSERVATION_ZONE));
        //Actions.runBlocking(scoring.pivotAction(Scoring.PIVOT_ZERO));
        Actions.runBlocking(lift.liftAction(Lift.EXT_LOW_BASKET));
        Actions.runBlocking(lift.liftAction(Lift.EXT_RETRACTED));

        /*Actions.runBlocking(new ParallelAction(
                scoring.pivotObservationZone(),
                scoring.clawToObservationZone()
        ));
        Actions.runBlocking(new ParallelAction(
                scoring.pivotSubmersible(),
                scoring.clawSubmersible()
        ));

         */
    }
}