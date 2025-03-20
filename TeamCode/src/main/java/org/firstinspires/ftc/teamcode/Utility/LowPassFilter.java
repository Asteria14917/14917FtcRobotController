package org.firstinspires.ftc.teamcode.Utility;

public class LowPassFilter {
    private double alpha;  // Smoothing factor (0 < alpha ≤ 1)
    private double filteredValue = 0;
    private boolean initialized = false;

    public LowPassFilter(double alpha) {
        this.alpha = alpha;
    }

    public double filter(double newValue) {
        if (!initialized) {
            filteredValue = newValue;
            initialized = true;
        } else {
            filteredValue = alpha * newValue + (1 - alpha) * filteredValue;
        }
        return filteredValue;
    }
}