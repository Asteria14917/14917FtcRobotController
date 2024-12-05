package org.firstinspires.ftc.teamcode.Utility;

public class RampingController {
    private double maxSpeed;
    private double minSpeed;
    private double rampUpRate;
    private double rampDownRate;
    private double threshold;
    private double speed;

    public boolean targetReached;

    public RampingController(double maxSpeed, double minSpeed,
                             double rampUpRate, double rampDownRate, double threshold) {
        this.maxSpeed = maxSpeed;
        this.minSpeed = minSpeed;
        this.rampUpRate = rampUpRate;
        this.rampDownRate = rampDownRate;
        this.threshold = threshold;
        this.targetReached = false;
        speed = 0;
    }

    public double calculate(double targetPosition, double currentPosition) {
        double distanceToTarget = Math.abs(targetPosition - currentPosition);
        double direction = Math.signum(targetPosition - currentPosition);

        //check if target has been reached
        if (distanceToTarget <= threshold) {
            targetReached = true;
            speed = 0;
            return 0;
        }else{
            targetReached = false;
        }



        //ramp down phase
        if (Math.abs(distanceToTarget) <= maxSpeed / rampDownRate) {
            speed = distanceToTarget * rampDownRate;
        } else if (speed < maxSpeed){
            speed+=rampUpRate;
            //ramp up phase or cruising at max speed
            //speed = Math.abs(currentPosition - targetPosition) * rampUpRate;
        }

        //clamp the speed between the minSpeed and maxSpeed
        speed = Math.max(minSpeed, Math.min(Math.abs(speed), maxSpeed)) * direction;

        return speed;
    }


    public boolean isTargetReached() {
        return targetReached;
    }

}
