package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

//Claw Servo Control
public class ServoControl_Claw {

    public OpMode _opMode;
    //This is variable of the motor to control the motor

    public Servo clawServo;
    //This is the Servo object that contorls the drone launcher


    //init of the servo
    public void init(OpMode opMode, String configName){
        _opMode = opMode;
        clawServo = _opMode.hardwareMap.get(Servo.class, configName);
    }

    public void OpenClaw(double command) {
        clawServo.setPosition(command);
    }
    public void CloseClaw(double command) {
        clawServo.setPosition(command);
    }
}
