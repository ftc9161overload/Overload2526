package org.firstinspires.ftc.teamcode.Util;

public class Lerp {
    public double startNum;

    public double currentNum;
    public double targetNum;
    public double time;
    public Timer timer = new Timer();

    /**
     * Constructor for Lerp.
     * @param startNum The starting number
     * @param targetNum The target number to reach at the end of the time param
     * @param time The length of time (in seconds) to preform
     */
    public Lerp(double startNum, double targetNum, double time){
        this.startNum = currentNum;
        this.targetNum = targetNum;
        this.time = time;
    }

    public void reset() {
        timer.reset();
    }

    public void setNewTarget(double targetNum, double time) {
        timer.reset();
        this.targetNum = targetNum;
        this.time = time;
    }

    public double getNum() {
        currentNum = startNum + (targetNum - startNum) * Math.min(Math.max(timer.getTimeSeconds() / time, 0), 1);
        return currentNum;
    }
}
