/*
Created by 23spatel on 5/26/22
*/


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//import java.util.HashMap;


public class RobotArm {
    //hardware map
    private HardwareMap hwMap;
    private double basePos;
    private double armPos;
    private double elbowPos;
    private double clawPos;
    
    private final double SPEED = 0.01;
    
    //private HashMap<String, Double>
    
    //arm servos
    public CRServo baseServo;
    public Servo armServo;
    public Servo elbowServo;
    public Servo clawServo;
    
    public RobotArm(){
        //constructor
        basePos = 0.5;
        armPos = 0.5;
        elbowPos = 0.5;
        clawPos = 0.5;
    }
    
    public void init(HardwareMap hwMap) {
        initHardwareMap(hwMap);
        hwMap.logDevices();
    }
    
    public void initHardwareMap(HardwareMap hwMap) {
        this.hwMap = hwMap;
        baseServo = hwMap.crservo.get("baseServo");
        armServo = hwMap.servo.get("armServo");
        elbowServo = hwMap.servo.get("elbowServo");
        clawServo = hwMap.servo.get("clawServo");

    }
    
    public void initClaw(){
        armServo.setPosition(armPos);
        clawServo.setPosition(clawPos);
        elbowServo.setPosition(elbowPos);
    }
    
    public void moveBase(int direction){
        baseServo.setPower(direction);
    }
    
//     public void moveServo(String servo, int direction) {
//         servo = hwMap.servo.get(servo);
        
//     }

    public void moveArm(int direction){
        armPos = Math.min( Math.max(0 , SPEED*direction), 1);
        armServo.setPosition(armPos);
    } 
    
    public void moveElbow(int direction){
        elbowPos = Math.min( Math.max(0 , SPEED*direction), 1);
        elbowServo.setPosition(elbowPos);
    }
    
    public void moveClaw(int direction){
        clawPos = Math.min( Math.max(0 , SPEED*direction), 1);
        clawServo.setPosition(clawPos);
    }

}
    
