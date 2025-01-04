package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.RobotLog;

//Elevator Control:
//  This class defines the controls for the elevator
//  The elevator is the item that will raise and lower the pixles to a level where the "bucket" will rotate
//  to drop the pixles into place  This class defines a servo for operating the bucket rotation as well.

public class HorizontalSliderControl {
    public HorizontalSliderControl(OpMode opMode, String configName){
        m_configName = configName;
        _opMode = opMode;
        slider_motor = _opMode.hardwareMap.get(DcMotor.class, configName);
//        this.resetElevatorMotorEncoder();
        HorzClawControl = new CashServo(_opMode,"horz_claw");
        HorizontalClawRotate = new CashServo(_opMode,"horz_claw_rotate");
        pos_pid = new pid_controller(); // PID control for position controls
        pos_pid.init_pid(.0025, 0, 0);  //Position controller to control elevator to a specific position.
    }
    String m_configName;
    public OpMode _opMode;
    //This is variable of the motor to control the motor
    public DcMotor slider_motor;
    private final CashServo HorzClawControl;
    private final CashServo HorizontalClawRotate;

    //  Added PID controller that will be used to control the teliop to a position the elevator to desired position
    public pid_controller pos_pid;
    private double hold_position_setpoint;  //This is the set point to control elevator to get to position.

    ///This area is for config values that set limits of the elevator control
    private double MAX_POWER = 1; //will not let the motor faster than this
    private double EXTEND_LIMIT = 840;  // encoder value that is the highest we want the elevator to go.
    private double RETRACT_LIMIT = 0;  // encoder value for the lowest we want the elevator to go.

    //This is the area that we create the snubbing of the end of travel for the elevator.
    //Snubbing is slowing the motor close to the end of travel so that we don't slam the elevator to the ends.
    private double EXTENDSNUB = 600; // Encoder value when we slow the motor down when extending to max
    private double EXTENDNUBFACTOR = 0.25; // The factor that we slow the command down by when extending
    private double RETRACTSNUB = 200; // Encoder value when we slow the motor down when retracting
    private double RETRACTSNUBFACTOR = 0.25;// The factor that we slow the command down by when retracting




    public void resetHorizontalMotorEncoder(){
        slider_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        slider_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //Elevator Controls for Teliop mode from joystick command
    public void ExtendRetract_Slider(double cmd) {
        int SliderPosition = slider_motor.getCurrentPosition();
        double SliderCommand;
        //If command is to raise elevator and we are past the limit of the extention, set cmd to zero
        //else if command is to lower and less that zero set command to zero.
        //otherwise just set the command to what is passed in.
        if (SliderPosition > EXTEND_LIMIT && cmd > 0) {
            SliderCommand = 0;
        } else if (SliderPosition < RETRACT_LIMIT && cmd < 0) {
            SliderCommand = 0;
        } else {
            SliderCommand = cmd;
        }

        //Snubbing functionality.
        //If cmd is extend and we are above the Upper snub encoder position, mult the cmd by the snub factor
        if (SliderPosition > EXTENDSNUB && cmd > 0) {
            SliderCommand = EXTENDNUBFACTOR * SliderCommand;
        }

        //If cmd is to retract and below the Lower snub encodeer psition, mutl the cmd by down snub factor
        if (SliderPosition < RETRACTSNUB && cmd < 0) {
            SliderCommand = RETRACTSNUBFACTOR * SliderCommand;
        }
        RobotLog.i(String.format("Elevator encoder %d", slider_motor.getCurrentPosition()));
        slider_motor.setPower(SliderCommand);
    }

    //This method is used to set the position of the elevator in auto mode.
    public void extendSliderToPosition_AUTO(double cmd, int desiredPos) {
        boolean extend = false;

        this.ExtendRetract_Slider(cmd);

        // if current position is less then desired position keep going

        //  while (((LinearOpMode) _opMode).opModeIsActive() && (abs(elevator_motor.getCurrentPosition()-desiredPos) > 5))
        while ((((LinearOpMode) _opMode).opModeIsActive())
                &&
                ((slider_motor.getCurrentPosition() < desiredPos && cmd > 0)
                        ||
                        (slider_motor.getCurrentPosition() > desiredPos && cmd < 0))) {

//            RobotLog.d(String.format("COMMAND: %.03f DESIRED POSITION :  %d ",cmd,desiredPos));
        }
        this.ExtendRetract_Slider(0);
    }


    public int getCurrentPostion() {
        int Slider_motorCurrentPosition = slider_motor.getCurrentPosition();
        return Slider_motorCurrentPosition;
    }

    //This function sets what is called the set-point of the elevator.  The set-point is the desired
    //position you would like the elevator to reach.  This is intended be set by the teli op mode for
    //buttons that are preset to command elevaotor to a specific position.
    //NOTE:  YOU MUST USE THE UPDATE METHOD TO ACTIVATE THE PID CONTROLLER TO GO TO THE DESIRED POSITION.
    public void set_slider_desired_position(int hold_set_point) {
        if (hold_set_point >= EXTEND_LIMIT) {
            hold_position_setpoint = EXTEND_LIMIT;
        } else if (hold_set_point <= RETRACT_LIMIT) {
            hold_position_setpoint = RETRACT_LIMIT;
        } else {
            hold_position_setpoint = hold_set_point;
        }
    }

    //This is the update method that is needed to activate the PID to control the elevator to a position.
    //This method is called from teliop mode and you must pass it the elapsed time since the last update.
    //The elapsed time is using the "elapsedTime" class.
    public void updatePosControl(double dt) {
//        RobotLog.i(String.format("desPosition %.2f",desPosition));
        //pos_pid.update:  pass the desired position (set-point) and the current encoder position
        //with the dt and max and min commands to the update method will do the math to figure out
        //what the elevCmd needs to be to get to and hold to the desired position.

        int Slider_current_position = slider_motor.getCurrentPosition();
        double minCmd = -.75;
        double maxCmd = 1;
        if (Slider_current_position < RETRACTSNUB && hold_position_setpoint == 0) {
            minCmd = -RETRACTSNUBFACTOR;
            maxCmd = 1.0;
        }


//        if (hold_position_setpoint == 0 &&
//                bucket.getPosition() < 10  &&
//                Slider_current_position >= RESET_BUCKET_POSITION_HIGH){
//            reset_pixle_bucket();
//        }

        double elevCmd = pos_pid.update(
                hold_position_setpoint,
                slider_motor.getCurrentPosition(),
                minCmd,
                maxCmd,
                dt);

//        RobotLog.i(String.format("setPoint %.2f, actual postition: %d, dt: %.2f",
//                hold_position_setpoint,elevator_motor.getCurrentPosition(),dt));

        slider_motor.setPower(elevCmd);
    }

    //Opens/close commands for Veticale Claw
    public void closeClaw(double command) {
        HorzClawControl.CCWCmd(command);
    }
    public void openClaw(double command) {
        HorzClawControl.CWCmd(command);
    }

    //Sets the Deliver and Recieve positions of the Vertical Claw
    public void clawDeliverPosition(double command) {
        HorizontalClawRotate.CCWCmd(command);
    }
    public void clawReceivePosition(double command) {
        HorizontalClawRotate.CWCmd(command);
    }


}


