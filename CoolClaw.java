package org.firstinspires.ftc.teamcode;
/**
 * Created by 23spatel on 5/26/22.
 */

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="ArmControl", group="Drive")
public class CoolClaw extends LinearOpMode {

    /* Declare instance of robot object */
    RobotArm arm           = new RobotArm();   // Use a 2022 robot's hardware

    @Override
    public void runOpMode() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        arm.initHardwareMap(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("MSG", "Robot init");
        telemetry.update();
        
        

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        arm.initClaw();

        // run until the end of the match (driver presses STOP)
        
        boolean[] inputs = new boolean[];
        
        final int ARM;
        
        static final long startTime = System.nanoTime();
        long lastTime = startTime;
        long loopPause = 50000;
        while (opModeIsActive()) {
            
            long now = System.nanoTime()
            long dt = now-lastTime;
            
            if (dt >= loopPause) {
                lastTime = now;            
                
                boolean armUp = gamepad1.x;
                boolean armDown = gamepad1.b;
                boolean openClaw = gamepad1.y;
                boolean closeClaw = gamepad1.a;

                double elbow = gamepad1.right_stick_y;
                double arm1 = gamepad1.left_stick_y;
                double base = gamepad1.left_stick_x;

                telemetry.addData("leftX ", armDown);
                telemetry.addData("rightX ", armUp);
                telemetry.addData("openClaw ", openClaw);
                telemetry.addData("closeClaw ", closeClaw);
                telemetry.addData("elbow ", elbow);
                telemetry.update();

                if(openClaw){
                    arm.moveClaw(3);
                }
                if(closeClaw){
                    arm.moveClaw(-3);
                }

                arm.moveArm((int)(arm1*5));
                arm.moveElbow((int)(elbow*5));
                arm.moveBase((int)(base*5));
                
            }
        }
    }
}
