package org.firstinspires.ftc.teamcode.Utility;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MotionProfile2D {
    private double startX, startY, targetX, targetY;
    private double maxVelocity, maxAcceleration;
    public double totalDistance, timeToMaxVelocity, distanceToMaxVelocity, cruiseDistance, cruiseTime, totalTime;

    public String phase;

    private boolean isTriangular;
    public ElapsedTime timeElapsed;

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

        // Compute motion profile
        this.distanceToMaxVelocity = (maxVelocity * maxVelocity) / (2 * maxAcceleration);

        if (2 * distanceToMaxVelocity > totalDistance) {
            phase = "TRI_CREATED";
            // Use a triangular profile instead (lower peak velocity)
            isTriangular = true;
            maxVelocity = Math.sqrt(2 * maxAcceleration * (totalDistance / 2));
            timeToMaxVelocity = maxVelocity / maxAcceleration;
            distanceToMaxVelocity = (maxVelocity * maxVelocity) / (2 * maxAcceleration); // Recalculate correctly
            cruiseDistance = 0;
            cruiseTime = 0;
        } else {
            phase = "TRAP_CREATED";
            // Normal trapezoidal profile
            isTriangular = false;
            timeToMaxVelocity = maxVelocity / maxAcceleration;
            cruiseDistance = totalDistance - (2 * distanceToMaxVelocity);
            cruiseTime = cruiseDistance / maxVelocity;
        }

        totalTime = 2 * timeToMaxVelocity + cruiseTime;
        timeElapsed = new ElapsedTime();
        timeElapsed.reset();
    }

    public double[] getTargetPosition() {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        if (totalDistance == 0) {
            return new double[]{startX, startY}; // Already at target
        }

        double elapsedTime = timeElapsed.seconds(); // Store time once
        double progress = 0;

        /*if (isTriangular) {
            // Triangular motion profile (only acceleration and deceleration)
            if (elapsedTime < timeToMaxVelocity) {
                // Acceleration phase
                phase = "TRI_ACCEL";
                progress = 0.5 * maxAcceleration * Math.pow(elapsedTime, 2);
            } else if (elapsedTime < 2 * timeToMaxVelocity) {
                // Deceleration phase
                phase = "TRI_DECEL";
                double decelTime = elapsedTime - timeToMaxVelocity;
                dashboardTelemetry.addData("ProfileDecelTime", decelTime);
                progress = (totalDistance / 2) + ((maxVelocity * decelTime) - (0.5 * maxAcceleration * Math.pow(decelTime, 2)));
            } else {
                phase = "TRI_REACHED";
                // Target reached
                progress = totalDistance;
            }
        } else {

         */
            // Standard trapezoidal profile
            if (elapsedTime < timeToMaxVelocity) {
                // Acceleration phase
                phase = "TRAP_ACCEL";
                progress = 0.5 * maxAcceleration * Math.pow(elapsedTime, 2);
            } else if (elapsedTime < timeToMaxVelocity + cruiseTime) {
                // Cruise phase
                phase = "TRAP_CRUISE";
                progress = distanceToMaxVelocity + maxVelocity * (elapsedTime - timeToMaxVelocity);
            } else if (elapsedTime < totalTime) {
                // Deceleration phase
                phase = "TRAP_DECEL";
                double decelTime = elapsedTime - (timeToMaxVelocity + cruiseTime);
                progress = distanceToMaxVelocity + cruiseDistance + (maxVelocity * decelTime) - (0.5 * maxAcceleration * Math.pow(decelTime, 2));
            } else {
                // Target reached
                phase = "TRAP_REACHED";
                progress = totalDistance;
            }


        // Prevent overshooting
        progress = Math.min(progress, totalDistance);

        dashboardTelemetry.addData("ProfileProgress", progress);

        // Scale progress to (x, y)
        double ratio = progress / totalDistance;
        double x = startX + (targetX - startX) * ratio;
        double y = startY + (targetY - startY) * ratio;
        return new double[]{x, y};
    }

    boolean profileComplete() {
        return timeElapsed.seconds() >= totalTime + 1.0;
    }
}
