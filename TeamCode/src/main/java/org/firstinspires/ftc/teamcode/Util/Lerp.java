package org.firstinspires.ftc.teamcode.Util;

public class Lerp {
    public double currentNum;
    public double targetNum;
    public double time;
    public Timer timer = new Timer();

    /**
     * Constructor for Lerp.
     * @param currentNum The starting number
     * @param targetNum The target number to reach at the end of the time param
     * @param time The length of time (in seconds) to preform
     */
    public Lerp(double currentNum, double targetNum, double time){
        this.currentNum = currentNum;
        this.targetNum = targetNum;
        this.time = time;
    }

    public double getNum(){
        return currentNum + (targetNum - currentNum) * (timer.getTimeSeconds() / time);
    }
}
