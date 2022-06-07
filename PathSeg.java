package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Define a "PathSegment" object, used for building a path for the robot to follow.
 */
public class PathSeg {
    static final double timeError = 50; //50 milliseconds
    final double rat45 = 1;
    //desired distance for each motor
    public double fld;
    public double frd;
    public double bld;
    public double brd;
    //desired speed
    public static double speed = 1;
    //encoder tick target for motors
    public int flTarget;
    public int frTarget;
    public int blTarget;
    public int brTarget;
    ElapsedTime runtime;
    double timeOut = 10000;
    double startTime;

    public PathSeg(double leftDistance,
                   double rightDistance,
                   double bld,
                   double brd,
                   ElapsedTime runtime) {

        this.fld = rat45 * leftDistance;
        this.frd = rat45 * rightDistance;
        this.bld = rat45 * bld;
        this.brd = rat45 * brd;

        //speed = .75; //WAS .5
        this.runtime = runtime;
    }

    public boolean isTimedOut() {
        return isTimedOut(this.runtime);
    }

    public boolean isTimedOut(ElapsedTime currTime) {
        return currTime.milliseconds() - (startTime + timeOut) > timeError;
    }

    public static void setSpeed(double newSpeed){
        speed = newSpeed;
    }
}