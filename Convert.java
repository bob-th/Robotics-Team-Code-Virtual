package org.firstinspires.ftc.teamcode;

/**
 * Created by 22tjiang on 12/4/21.
**/

public class Convert {
    public static final double inchesInAYeetGyro = 3.375; // GYRO
    public static final double inchesInAYeetGyroHorizontal = 2.85; // GYRO HORIZONTAL
    public static final double inchesInAYeetNormal = 0.825; // NORMAL
    public static final double tileSize = 23.5;

    public static int inchesToYeetGV(double input) { return (int)(input / inchesInAYeetGyro); }

    public static int inchesToYeetGH(double input) { return (int)(input / inchesInAYeetGyroHorizontal); }

    public static int inchesToYeetN(double input) { return (int)(input / inchesInAYeetNormal); }

    public static int tileToYeetGV(double input) { return (int)(input * tileSize / inchesInAYeetGyro); }

    public static int tileToYeetGH(double input) { return (int)(input * tileSize / inchesInAYeetGyroHorizontal); }

    public static int tileToYeetN(double input) { return (int)(input * tileSize / inchesInAYeetNormal); }
}