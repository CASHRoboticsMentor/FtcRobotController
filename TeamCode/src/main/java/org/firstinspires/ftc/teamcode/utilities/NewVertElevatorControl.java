package org.firstinspires.ftc.teamcode.utilities;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.RobotLog;

//Elevator Control:
//  This class defines the controls for the elevator
//  The elevator is the item that will raise and lower the pixles to a level where the "bucket" will rotate
//  to drop the pixles into place  This class defines a servo for operating the bucket rotation as well.

@Config
public class NewVertElevatorControl {
    private PIDController controller;
    public static double p=0.005, i = .25, d = 0;
    public static double f = .075;
    public static double pFactor = 1; //for max down command
    public static int hold_position_setpoint = 0;
    private final DcMotorEx elevator_motor;


    String m_configName;
    public OpMode _opMode;
    //This is variable of the motor to control the motor
    private final CashServo VertClawControl;
    private final CashServo VerticalClawRotate;

    //  Added PID controller that will be used to control the teliop to a position the elevator to desired position
    ///This area is for config values that set limits of the elevator control
    private final int RAISE_LIMIT = 830;  // encoder value that is the highest we want the elevator to go.
    private final int LOWER_LIMIT = 0;  // encoder value for the lowest we want the elevator to go.

    //This is the area that we create the snubbing of the end of travel for the elevator.
    //Snubbing is slowing the motor close to the end of travel so that we don't slam the elevator to the ends.
    private int UPPERSNUB = 600; // Encoder value when we slow the motor down when extending to max
    private double UPPERSNUBFACTOR = .75; // The factor that we slow the command down by when extending
    private int LOWERSNUB = 300; // Encoder value when we slow the motor down when retracting
    private double LOWERSNUBFACTOR = 0.25;// The factor that we slow the command down by when retracting

    public void setPIDVals(){
        controller.setPID(p,i,d);
        controller.setF(f);
    }
    public NewVertElevatorControl(OpMode opMode, String configName){
        m_configName = configName;
        _opMode = opMode;
        elevator_motor = _opMode.hardwareMap.get(DcMotorEx.class, configName);
        this.resetElevatorMotorEncoder();
        VertClawControl = new CashServo(_opMode,"vert_claw");
        VerticalClawRotate = new CashServo(_opMode,"vert_claw_rotate");
        controller = new PIDController(p,i,d);
    }

    public void resetElevatorMotorEncoder(){
        elevator_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        elevator_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void raiseLowerElevator_Teliop(double rawCmd){
        int elevPos = elevator_motor.getCurrentPosition();
        double cmd;
//        if (elevPos >= UPPERSNUB && rawCmd >=0){
//            cmd = UPPERSNUBFACTOR*rawCmd;
//        }else{
//            cmd = rawCmd;
//        }
        cmd = rawCmd;

        double power;
        if (Math.abs(rawCmd) > 0.25){
            if (cmd < 0 ){
                power = pFactor* cmd;
            }else {
                power = cmd;
            }
//            if (elevPos > RAISE_LIMIT && cmd > 0){
//                hold_position_setpoint = RAISE_LIMIT;
//            }else if (elevPos<= LOWER_LIMIT && cmd <=0){
//                hold_position_setpoint = LOWER_LIMIT;
//            }else{
//                hold_position_setpoint = elevPos;
//            }
            hold_position_setpoint = elevPos;

        }else{
            double pid = controller.calculate(elevPos,hold_position_setpoint);
            double ff = f;
            power = pid + ff;
        }
//        double pid = controller.calculate(elevPos,target);
//        double ff = f;
//
//        double power = pid + ff;
        elevator_motor.setPower(power);
    }

    //This method is used to set the position of the elevator in auto mode.
    public void setElevatorToPostion(int desiredPos) {
        double elevPos = elevator_motor.getCurrentPosition();
        double pid = controller.calculate(elevPos,desiredPos);
        double power = pid + f;
        elevator_motor.setPower(power);
    }
    public int getCurrentPostion() {
        return elevator_motor.getCurrentPosition();
    }
    public int getTargetPosition() {
        return hold_position_setpoint;
    }

    //Opens/close commands for Veticale Claw
    public void closeClaw(double command) {
        VertClawControl.CCWCmd(command);
    }
    public void openClaw(double command) {
        VertClawControl.CWCmd(command);
    }

    //Sets the Deliver and Recieve positions of the Vertical Claw
    public void clawDeliverPosition(double command) {
        VerticalClawRotate.CCWCmd(command);
    }
    public void clawReceivePosition(double command) {
        VerticalClawRotate.CWCmd(command);
    }

}


