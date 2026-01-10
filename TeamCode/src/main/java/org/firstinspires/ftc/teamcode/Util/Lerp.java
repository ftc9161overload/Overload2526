package org.firstinspires.ftc.teamcode.Util;

/**
 * This is a Lerp class intended for use in Project Overload.
 * @ author Ryan Hajj - 9161 Overlaod
 */
public class Lerp {

    public enum Ease {
        LINEAR,
        EASE_IN,
        EASE_OUT,

        EASE_IN_OUT
    }

    public Ease ease = Ease.LINEAR;

    public double startNum;
    public double currentNum;
    public double targetNum;
    public double time;
    public Timer timer = new Timer();

    /**
     * Constructor for Lerp.
     * @param startNum The starting number
     * @param targetNum The target number to reach at the end of the time param
     * @param time The length of time (in seconds) to perform
     */
    public Lerp(double startNum, double targetNum, double time){
        this.startNum = startNum;
        this.targetNum = targetNum;
        this.time = time;
        this.currentNum = startNum;
    }

    public void reset() {
        timer.reset();
    }

    public void setEase(Ease easeType) {
        this.ease = easeType;
    }

    /**
     * Sets new Target and resets Timer.
     * @param targetNum The target number to reach at the end of the time param
     * @param time The length of time (in seconds) to perform
     */
    public void setNewTarget(double targetNum, double time) {
        timer.reset();
        this.startNum = currentNum;   // start from wherever we are
        this.targetNum = targetNum;
        this.time = time;
    }

    private double applyEase(double t) {
        switch (ease) {
            case EASE_IN:
                return t * t;
            case EASE_OUT:
                return 1 - (1 - t) * (1 - t);
            case EASE_IN_OUT:
                return t * t * (3 - 2 * t);
            default:
                return t;
        }
    }

    public double getNum() {
        double t = timer.getTimeSeconds() / time;
        t = Math.min(Math.max(t, 0), 1);  // clamp 0–1=
        t = applyEase(t);
        currentNum = startNum + (targetNum - startNum) * t;
        return currentNum;
    }
}
