package org.firstinspires.ftc.teamcode.nePremier.utils.filters;

public class LowPassFilter {
    private double alpha;
    private double lastValue = 0;
    private boolean filterInit;
    public LowPassFilter(double alpha){
        if(alpha >= 0 && alpha <= 1){
            this.alpha = alpha;
        }
        else{
            this.alpha = 0;
        }
        filterInit = false;
    }
    public double update(double value) {
        if (!filterInit) {
            filterInit = true;
            lastValue = value;
            return value;
        } else {
            double filtered =  (alpha * value) + ((1.0 - alpha) * lastValue);
            lastValue = filtered;
            return filtered;
        }
    }
    public void setAlpha(double alpha){
        this.alpha = alpha;
    }
}
