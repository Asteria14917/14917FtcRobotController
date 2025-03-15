package org.firstinspires.ftc.teamcode.Utility;

public class MotionProfileRedux {
    // Start and target positions
    public double startX, startY;
    public double targetX, targetY;

    // Motion parameters
    public double maxVelocity;
    public double maxAcceleration;

    // Profile values
    public double totalDistance;         // Total distance from start to target
    public double timeToMaxVelocity;     // Time to accelerate to peak velocity
    public double cruiseTime;            // Duration of constant (peak) velocity phase
    public double totalTime;             // Total time for the complete profile
    public double peakVelocity;          // The maximum velocity achieved in this profile

    // Dynamic fields updated during the profile execution
    public double timeElapsed;           // Time elapsed since the profile started (in seconds)
    public int phase;                    // 0: accelerating, 1: cruising, 2: decelerating, 3: complete

    // Internal timer
    private long startTime;              // Start time in milliseconds

    /**
     * Constructs a MotionProfile2D from a start position to a target position.
     *
     * @param startX         Starting X position.
     * @param startY         Starting Y position.
     * @param targetX        Target X position.
     * @param targetY        Target Y position.
     * @param maxVelocity    Maximum allowed velocity (in same distance units per second).
     * @param maxAcceleration Maximum allowed acceleration (in same distance units per second^2).
     */
    public MotionProfileRedux(double startX, double startY, double targetX, double targetY, double maxVelocity, double maxAcceleration) {
        this.startX = startX;
        this.startY = startY;
        this.targetX = targetX;
        this.targetY = targetY;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;

        // Compute the straight-line distance between start and target.
        totalDistance = Math.hypot(targetX - startX, targetY - startY);

        // Assume we try to accelerate to maxVelocity initially.
        timeToMaxVelocity = maxVelocity / maxAcceleration;
        double distanceDuringAccel = 0.5 * maxAcceleration * timeToMaxVelocity * timeToMaxVelocity;

        // Check if the robot can reach max velocity before it needs to decelerate.
        if (2 * distanceDuringAccel > totalDistance) {
            // Triangular profile: the distance is too short.
            timeToMaxVelocity = Math.sqrt(totalDistance / maxAcceleration);
            peakVelocity = maxAcceleration * timeToMaxVelocity;
            cruiseTime = 0;
            totalTime = 2 * timeToMaxVelocity;
        } else {
            // Trapezoidal profile: there is a constant (max) velocity phase.
            peakVelocity = maxVelocity;
            cruiseTime = (totalDistance - 2 * distanceDuringAccel) / maxVelocity;
            totalTime = 2 * timeToMaxVelocity + cruiseTime;
        }

        // Initialize timer and dynamic fields.
        startTime = System.currentTimeMillis();
        timeElapsed = 0;
        phase = 0;
    }

    /**
     * Returns the current target position [x, y] based on the elapsed time of the motion profile.
     *
     * @return a double array with the instantaneous target x and y positions.
     */
    public double[] getTargetPosition() {
        // Compute elapsed time in seconds.
        timeElapsed = (System.currentTimeMillis() - startTime) / 1000.0;
        double distanceTraveled = 0;

        // Determine which phase of the profile we are in and calculate the distance traveled.
        if (timeElapsed < timeToMaxVelocity) {
            // Accelerating phase: s = 0.5 * a * t^2
            distanceTraveled = 0.5 * maxAcceleration * timeElapsed * timeElapsed;
            phase = 0;
        } else if (timeElapsed < timeToMaxVelocity + cruiseTime) {
            // Cruising phase: add the distance from acceleration then cruise at constant velocity.
            distanceTraveled = (0.5 * maxAcceleration * timeToMaxVelocity * timeToMaxVelocity)
                    + peakVelocity * (timeElapsed - timeToMaxVelocity);
            phase = 1;
        } else if (timeElapsed < totalTime) {
            // Decelerating phase: compute the distance covered during deceleration.
            double tDecel = timeElapsed - timeToMaxVelocity - cruiseTime;
            distanceTraveled = (0.5 * maxAcceleration * timeToMaxVelocity * timeToMaxVelocity)
                    + peakVelocity * cruiseTime
                    + peakVelocity * tDecel - 0.5 * maxAcceleration * tDecel * tDecel;
            phase = 2;
        } else {
            // Profile complete: ensure we do not overshoot the total distance.
            distanceTraveled = totalDistance;
            phase = 3;
        }

        // Calculate the ratio of distance traveled relative to the total distance.
        double ratio = (totalDistance == 0) ? 1 : (distanceTraveled / totalDistance);
        if (ratio > 1) ratio = 1;

        // Compute the instantaneous target position along the straight-line path.
        double currentX = startX + (targetX - startX) * ratio;
        double currentY = startY + (targetY - startY) * ratio;

        return new double[]{currentX, currentY};
    }

    /**
     * Indicates whether the motion profile has completed.
     *
     * @return true if the elapsed time is greater than or equal to the total time required for the profile.
     */
    public boolean profileComplete() {
        timeElapsed = (System.currentTimeMillis() - startTime) / 1000.0;
        return timeElapsed >= totalTime;
    }
}
