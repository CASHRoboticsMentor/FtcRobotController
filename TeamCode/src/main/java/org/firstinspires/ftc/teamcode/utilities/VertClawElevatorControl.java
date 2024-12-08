package org.firstinspires.ftc.teamcode.utilities;

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

public class VertClawElevatorControl {
    public VertClawElevatorControl(OpMode opMode, String configName){
        m_configName = configName;
        _opMode = opMode;
        elevator_motor = _opMode.hardwareMap.get(DcMotorEx.class, configName);
        this.resetElevatorMotorEncoder();
        VertClawControl = new CashServo(_opMode,"vert_claw");
        VerticalClawRotate = new CashServo(_opMode,"vert_claw_rotate");
        pos_pid = new pid_controller(); // PID control for position controls
        pos_pid.init_pid(0.000, 0, 0);  //Position controller to control elevator to a specific position.

        vel_pid = new pid_controller();
        vel_pid.init_pid(0.0000,0,0);
    }
    String m_configName;
    public OpMode _opMode;
    //This is variable of the motor to control the motor
    public DcMotor elevator_motor;
    private final CashServo VertClawControl;
    private final CashServo VerticalClawRotate;

    //  Added PID controller that will be used to control the teliop to a position the elevator to desired position
    public pid_controller pos_pid;

    public pid_controller vel_pid;
    private double hold_position_setpoint;  //This is the set point to control elevator to get to position.

    ///This area is for config values that set limits of the elevator control
    private double MAX_POWER = 1; //will not let the motor faster than this
    private double RAISE_LIMIT = 830;  // encoder value that is the highest we want the elevator to go.
    private double LOWER_LIMIT = 0;  // encoder value for the lowest we want the elevator to go.

    //This is the area that we create the snubbing of the end of travel for the elevator.
    //Snubbing is slowing the motor close to the end of travel so that we don't slam the elevator to the ends.
    private double UPPERSNUB = 900; // Encoder value when we slow the motor down when extending to max
    private double UPPERSNUBFACTOR = .75; // The factor that we slow the command down by when extending
    private double LOWERSNUB = 300; // Encoder value when we slow the motor down when retracting
    private double LOWERSNUBFACTOR = 0.25;// The factor that we slow the command down by when retracting

    private double previousElevCmd = 0;
    private int previousElevatorPosition = 0;




    public void resetElevatorMotorEncoder(){
        elevator_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        elevator_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //Elevator Controls for Teliop mode from joystick command
    public void raiseLowerElevator_T(double cmd) {
        int ElevatorPosition = elevator_motor.getCurrentPosition();
        double elevatorCommand;
        //If command is to raise elevator and we are past the limit of the extention, set cmd to zero
        //else if command is to lower and less that zero set command to zero.
        //otherwise just set the command to what is passed in.
        if (ElevatorPosition > RAISE_LIMIT && cmd > 0) {
            elevatorCommand = 0;
            this.set_elevator_desired_position((int)RAISE_LIMIT);
        } else if (ElevatorPosition < LOWER_LIMIT && cmd < 0) {
            elevatorCommand = 0;

        } else {
            elevatorCommand = cmd;
        }

        //Snubbing functionality.
        //If cmd is extend and we are above the Upper snub encoder position, mult the cmd by the snub factor
        if (ElevatorPosition > UPPERSNUB && cmd > 0) {
            elevatorCommand = UPPERSNUBFACTOR * elevatorCommand;
        }

        //If cmd is to retract and below the Lower snub encodeer psition, mutl the cmd by down snub factor
        if (ElevatorPosition < LOWERSNUB && cmd < 0) {
            elevatorCommand = LOWERSNUBFACTOR * elevatorCommand;
        }
        RobotLog.i(String.format("Elevator encoder %d", elevator_motor.getCurrentPosition()));
        elevator_motor.setPower(elevatorCommand);
     //   previousElevCmd = elevatorCommand;
    }

    //This method is used to set the position of the elevator in auto mode.
    public void raiseLowerElevatorToPosition_AUTO(double cmd, int desiredPos) {
        boolean extend = false;

        this.raiseLowerElevator_T(cmd);

        // if current position is less then desired position keep going

        //  while (((LinearOpMode) _opMode).opModeIsActive() && (abs(elevator_motor.getCurrentPosition()-desiredPos) > 5))
        while ((((LinearOpMode) _opMode).opModeIsActive())
                &&
                ((elevator_motor.getCurrentPosition() < desiredPos && cmd > 0)
                        ||
                        (elevator_motor.getCurrentPosition() > desiredPos && cmd < 0))) {

//            RobotLog.d(String.format("COMMAND: %.03f DESIRED POSITION :  %d ",cmd,desiredPos));
        }
        this.raiseLowerElevator_T(0);
    }


    public int getCurrentPostion() {
        int elevator_motorCurrentPosition = elevator_motor.getCurrentPosition();
        return elevator_motorCurrentPosition;
    }

    //This function sets what is called the set-point of the elevator.  The set-point is the desired
    //position you would like the elevator to reach.  This is intended be set by the teli op mode for
    //buttons that are preset to command elevaotor to a specific position.
    //NOTE:  YOU MUST USE THE UPDATE METHOD TO ACTIVATE THE PID CONTROLLER TO GO TO THE DESIRED POSITION.
    public void set_elevator_desired_position(int hold_set_point) {
        if (hold_set_point >= RAISE_LIMIT) {
            hold_position_setpoint = RAISE_LIMIT;
        } else if (hold_set_point <= LOWER_LIMIT) {
            hold_position_setpoint = LOWER_LIMIT;
        } else {
            hold_position_setpoint = hold_set_point;

        }
//        RobotLog.i(String.format("ELEV:  Set position: %i",hold_set_point));
    }

    //This is the update method that is needed to activate the PID to control the elevator to a position.
    //This method is called from teliop mode and you must pass it the elapsed time since the last update.
    //The elapsed time is using the "elapsedTime" class.
    public void updatePosControl(double dt) {
//        RobotLog.i(String.format("desPosition %.2f",desPosition));
        //pos_pid.update:  pass the desired position (set-point) and the current encoder position
        //with the dt and max and min commands to the update method will do the math to figure out
        //what the elevCmd needs to be to get to and hold to the desired position.

        int elevator_current_position = elevator_motor.getCurrentPosition();
        double minCmd = -.75;
        double maxCmd = 1;
        if (elevator_current_position < LOWERSNUB && hold_position_setpoint == 0) {
            minCmd = -LOWERSNUBFACTOR;
            maxCmd = 1.0;
        }


//        if (hold_position_setpoint == 0 &&
//                bucket.getPosition() < 10  &&
//                elevator_current_position >= RESET_BUCKET_POSITION_HIGH){
//            reset_pixle_bucket();
//        }

        double elevCmd = pos_pid.update(
                hold_position_setpoint,
                elevator_current_position,
                minCmd,
                maxCmd,
                dt);

        RobotLog.i(String.format("setPoint %.2f, actual postition: %d, dt: %.5f,  ELEVCMD: %.2f",
                hold_position_setpoint,elevator_motor.getCurrentPosition(),dt,elevCmd));
//        if (Math.abs(hold_position_setpoint - elevator_motor.getCurrentPosition()) < 2 ){
//            elevCmd = previousElevCmd;
//        }
        elevator_motor.setPower(elevCmd);
    }

    public void updateVelControl(double desiredVel, double dt) {
//        RobotLog.i(String.format("desPosition %.2f",desPosition));
        //pos_pid.update:  pass the desired position (set-point) and the current encoder position
        //with the dt and max and min commands to the update method will do the math to figure out
        //what the elevCmd needs to be to get to and hold to the desired position.

        double elevator_current_velocity = (elevator_motor.getCurrentPosition()-previousElevatorPosition)/dt;
        double minCmd = -.75;
        double maxCmd = 1;
//        if (elevator_motor.getCurrentPosition() < LOWERSNUB && hold_position_setpoint == 0) {
//            minCmd = -LOWERSNUBFACTOR;
//            maxCmd = 1.0;
//        if ( (elevator_motor.getCurrentPosition() < 5) && (desiredVel < 0) ) {
//            desiredVel = 0;
//        }else if( ( elevator_motor.getCurrentPosition() > RAISE_LIMIT) && (desiredVel > 0) ){
//            desiredVel = 0;
//        }


//        if (hold_position_setpoint == 0 &&
//                bucket.getPosition() < 10  &&
//                elevator_current_position >= RESET_BUCKET_POSITION_HIGH){
//            reset_pixle_bucket();
//        }

        double elevCmd = vel_pid.update(
                desiredVel,
                elevator_current_velocity,
                minCmd,
                maxCmd,
                dt);

        RobotLog.i(String.format("desiredVel %.2f, actual velocity: %.2fdes, dt: %.5f, outputCmd: %.2f",
                desiredVel,elevator_current_velocity,dt,elevCmd));
//        if (Math.abs(hold_position_setpoint - elevator_motor.getCurrentPosition()) < 2 ){
//            elevCmd = previousElevCmd;
//        }
        elevator_motor.setPower(elevCmd);
        previousElevatorPosition = elevator_motor.getCurrentPosition();
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


