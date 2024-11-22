package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

//Claw Servo Control
public class CashServo {
    public CashServo(OpMode opMode,String configName) {
        m_config_name = configName;
        _opMode = opMode;
        CashServo = _opMode.hardwareMap.get(Servo.class, configName);
    }
    String m_config_name;

    public OpMode _opMode;
    //This is variable of the motor to control the motor

    public Servo CashServo;
    //This is the Servo object that contorls the drone launcher

    public void CWCmd(double command) {
        CashServo.setPosition(command);
    }
    public void CCWCmd(double command) {
        CashServo.setPosition(command);
    }
}
