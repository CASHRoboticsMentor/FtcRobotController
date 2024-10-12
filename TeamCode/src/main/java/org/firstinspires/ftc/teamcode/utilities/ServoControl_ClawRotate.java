package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

//Claw Servo Control
public class ServoControl_ClawRotate {

    public OpMode _opMode;
    //This is variable of the motor to control the motor

    public Servo rotateServo;
    //This is the Servo object that contorls the drone launcher


    //init of the servo
    public void init(OpMode opMode, String configName){
        _opMode = opMode;
        rotateServo = _opMode.hardwareMap.get(Servo.class, configName);
    }
    public void DownPosition(double command) {
        rotateServo.setPosition(command);}

    public void UpPosition(double command) {
        rotateServo.setPosition(command);}


}
