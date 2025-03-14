package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.util.ElapsedTime;

public class MotionProfile2D {
    private double startX, startY, targetX, targetY;
    private double maxVelocity, maxAcceleration;
    private double totalDistance, timeToMaxVelocity, distanceToMaxVelocity, cruiseDistance, cruiseTime, totalTime;

    ElapsedTime timeElapsed;

    public MotionProfile2D(double startX, double startY, double targetX, double targetY, double maxVelocity, double maxAcceleration) {
        this.startX = startX;
        this.startY = startY;
        this.targetX = targetX;
        this.targetY = targetY;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;

        // Compute total distance to target
        double deltaX = targetX - startX;
        double deltaY = targetY - startY;
        this.totalDistance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

        // Compute motion profile times
        this.timeToMaxVelocity = maxVelocity / maxAcceleration;
        this.distanceToMaxVelocity = 0.5 * maxAcceleration * Math.pow(timeToMaxVelocity, 2);
        this.cruiseDistance = Math.max(0, totalDistance - 2 * distanceToMaxVelocity);
        this.cruiseTime = cruiseDistance / maxVelocity;
        this.totalTime = 2 * timeToMaxVelocity + cruiseTime;

        timeElapsed = new ElapsedTime();
        timeElapsed.reset();
    }

    public double[] getTargetPosition() {
        if (totalDistance == 0) {
            return new double[]{startX, startY}; // Already at target
        }

        double progress;
        if (timeElapsed.seconds() < timeToMaxVelocity) {
            progress = 0.5 * maxAcceleration * Math.pow(timeElapsed.seconds(), 2); // Acceleration phase
        } else if (timeElapsed.seconds() < timeToMaxVelocity + cruiseTime) {
            progress = distanceToMaxVelocity + maxVelocity * (timeElapsed.seconds() - timeToMaxVelocity); // Cruise phase
        } else if (timeElapsed.seconds() < totalTime) {
            double decelTime = timeElapsed.seconds() - (timeToMaxVelocity + cruiseTime);
            progress = distanceToMaxVelocity + cruiseDistance + (maxVelocity * decelTime) - (0.5 * maxAcceleration * Math.pow(decelTime, 2)); // Deceleration phase
        } else {
            progress = totalDistance; // Reached target
        }

        // Scale progress to (x, y)
        double ratio = progress / totalDistance;
        double x = startX + (targetX - startX) * ratio;
        double y = startY + (targetY - startY) * ratio;
        return new double[]{x, y};
    }

    boolean profileComplete(){
        if(timeElapsed.seconds() >= totalTime){
            return true;
        }else{
            return false;
        }
    }
}
