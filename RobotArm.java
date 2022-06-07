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


public class RobotArm {
    //hardware map
    private HardwareMap hwMap;
    private  double basePos;
    private  double armPos;
    private  double elbowPos;
    private  double clawPos;
    
    
    //arm servos
    public  CRServo baseServo;
    public  Servo armServo;
    public  Servo elbowServo;
    public  Servo clawServo;
    
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
    
    public  void moveBase(int direction){
        baseServo.setPower(direction);
    }

    public  void moveArm(int direction){
        if (0<(armPos+(0.01*direction)) && (armPos+(0.01*direction))<1){
            armPos+=0.01*direction;
            armServo.setPosition(armPos);
        }
    } 
    
    public  void moveElbow(int direction){
        if (0<(elbowPos+(0.01*direction)) && (elbowPos+(0.01*direction))<1){
            elbowPos+=0.01*direction;
            elbowServo.setPosition(elbowPos);
        }
    }
    
    public  void moveClaw(int direction){
        if (0<(clawPos+(0.01*direction)) && (clawPos+(0.01*direction))<1){
            clawPos+=0.01*direction;
            clawServo.setPosition(clawPos); 
        }
    }

}
    
