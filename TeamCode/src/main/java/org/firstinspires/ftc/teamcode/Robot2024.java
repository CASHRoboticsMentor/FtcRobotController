package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.utilities.CASH_Drive_Library;
import org.firstinspires.ftc.teamcode.utilities.ElevatorControl;
import org.firstinspires.ftc.teamcode.utilities.IMUUtility;
import org.firstinspires.ftc.teamcode.utilities.IMUUtility2;
import org.firstinspires.ftc.teamcode.utilities.ServoControl_Claw;
import org.firstinspires.ftc.teamcode.utilities.ServoControl_ClawRotate;
import org.firstinspires.ftc.teamcode.utilities.SweeperControl;
import org.firstinspires.ftc.teamcode.utilities.WinchControl;
import org.firstinspires.ftc.teamcode.utilities.pid_controller;

public class Robot2024<_opMode> {
    public final OpMode _opMode; //holds opmode object

    //Constructor for our robot so this object knows about the opMode
    public Robot2024(OpMode opMode) {
        _opMode = opMode;
    }

    //Define Robot Motor variables
    public DcMotor leftFrontMotor = null;
    public DcMotor rightFrontMotor = null;
    public DcMotor leftRearMotor = null;
    public DcMotor rightRearMotor = null;

    //Creates a new object for the robot IMU
    public IMUUtility2 robotIMU = new IMUUtility2();

    //Creates a new object for the Drive Library
    public CASH_Drive_Library CASHDriveLibrary = new CASH_Drive_Library();

    private ServoControl_Claw ClawControl;
    {
        ClawControl = new ServoControl_Claw();
    }

    private ServoControl_ClawRotate RotateControl;
    {
        RotateControl = new ServoControl_ClawRotate();
    }



    //Creates a new object of type Elevator Control
    //All Elevator Controls will be writen in Elevator Control

     private ElevatorControl elevatorCode;
    {
        elevatorCode = new ElevatorControl();
    }

    private WinchControl winchCode;
    {
        winchCode = new WinchControl();
    }

    //Creates a new object of type Sweeper Control
    //All Sweeper Controls will be written in Sweeper Control
    private SweeperControl SweeperCode;
    {
       SweeperCode = new SweeperControl();
    }

    /*
    //Control directions of the robot.  These should not be changed as these are specific to how the robot is designed
    */
    public double FORWARD = CASHDriveLibrary.FORWARD;
    public double REVERSE = CASHDriveLibrary.REVERSE;
    public double RIGHT = CASHDriveLibrary.RIGHT;
    public double LEFT = CASHDriveLibrary.LEFT;

    public double TURN_RIGHT = CASHDriveLibrary.TURN_RIGHT;
    public double TURN_LEFT = CASHDriveLibrary.TURN_LEFT;

    public int DELIVER_PIXLE_POSITION = elevatorCode.PIXLE_DELIVER_POSITION;
    public int ELEVATOR_MID_POSITION = elevatorCode.MID_POSITION;
    public int ELEVATOR_HIGH_POSITION = elevatorCode.HIGH_POSITION;

    public int WINCH_RAISE_POSITION = winchCode.RAISE_LIMIT;

    //This is the initialization for this years robot.
    //It initializes the following:
    // All drive motors
    // The IMU
    //
    public void initializeRobot() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        rightRearMotor = _opMode.hardwareMap.get(DcMotor.class, "right_rear_drive");
        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftRearMotor = _opMode.hardwareMap.get(DcMotor.class, "left_rear_drive");
        leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftFrontMotor = _opMode.hardwareMap.get(DcMotor.class, "left_front_drive");
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rightFrontMotor = _opMode.hardwareMap.get(DcMotor.class, "right_front_drive");
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //Initialize objects in common lib
        CASHDriveLibrary.init(_opMode);
        CASHDriveLibrary.leftFrontMotor = leftFrontMotor;
        CASHDriveLibrary.rightFrontMotor = rightFrontMotor;
        CASHDriveLibrary.leftRearMotor = leftRearMotor;
        CASHDriveLibrary.rightRearMotor = rightRearMotor;
        CASHDriveLibrary.EnableEncoders();

        robotIMU.initialize(_opMode,"imu");
        CASHDriveLibrary.imu = robotIMU;

        CASHDriveLibrary.init_rotations_pid();
        CASHDriveLibrary.init_distanceToWall_pid();
    }

    public void resetIMU(){
        robotIMU.resetAngle();
    }

    // This intitilizes the elevator and sweeper control.
    // initializeImplements:  For All autonomous programse
    //ONLY USE THIS FOR AUTO PROGRAMS
    public void initializeImplements() {
//        elevatorCode.init(_opMode, "elevator_motor");
//        SweeperCode.init(_opMode, "sweeper_motor");
//        winchCode.init(_opMode,"winch_motor");
        ClawControl.init(_opMode, "claw_servo");
        RotateControl.init(_opMode, "claw_rotate_servo");
    }

    public void DownPosition(double command){
        RotateControl.DownPosition(command);


    }

    public void UpPosition(double command){
        RotateControl.UpPosition(command);


    }

    public void openClaw(double command){
        ClawControl.OpenClaw(command);


    }
    public void closeClaw(double command) {
        ClawControl.CloseClaw(command);
    }

    //ONLY USE THIS FOR TELIPOP
    //elevatorCode.intit2:  This initializes the implements but doesn't reset the encoders.
    public void initializeImplements2() {
        elevatorCode.init2(_opMode, "elevator_motor");
        SweeperCode.init(_opMode, "sweeper_motor");
        winchCode.init2(_opMode,"winch_motor");
    }


    ////////////////////////////////////////New Navigation Methods////////////////////////////////
    public void moveRobotteli(double leftjoyx, double leftjoyy, double rightjoyx) {
        CASHDriveLibrary.MoveRobotTeliOp(leftjoyx, leftjoyy, rightjoyx,  false, false);
    }

    //Used to move the robot forward/revers/left/right.  This also uses fore/aft encoder and imu
    //for corrections.  IMU keeps the heading constant and fore/aft encoder helps keeps the
    //robot going in a straight line left and right.
    public void moveRobotAuto(double direction_deg, double power, double distance_inch) {
        CASHDriveLibrary.EnableEncoders();
//        CASHDriveLibrary.resetForeAftEncoder();  Use only if fore/aft encoder installed
        CASHDriveLibrary.MoveRobotAuto(direction_deg, power, distance_inch, false, _opMode, false);
    }

    public void moveRobotAuto_DistanceFromWall(double direction_deg, double power, double distance_inch,
                                               double distanceFromWall, DistanceSensor distSensor) {
        CASHDriveLibrary.EnableEncoders();
        CASHDriveLibrary.MoveRobotAuto_DistanceFromWall(direction_deg, power, distance_inch, false, _opMode, false,
                distanceFromWall,distSensor);
    }
    //This method is used to rotate the robot by as specified amount of degrees.  It uses the
    //IMU for feedback
    public void rotateRobotAuto2(double direction, double rotationAngle_d, double power) {
        CASHDriveLibrary.RotateRobotAuto2(direction, rotationAngle_d, power);
    }
    //Get Ticks is for knowing where the robot has traveled
    public int getTicks(){
        return  CASHDriveLibrary.getRightRearEncoderTick();
    }
    public void navWTgs(double headingError){
        CASHDriveLibrary.navigateAprilTags(headingError);
    }

    ////////////////////////////////All Functions to manipulate apparatuses on robot//////////////

    //elevator control functions
    public void raiseLowerElevator(double cmd) {
        elevatorCode.raiseLowerElevator_T(cmd);
    }
    public int getElevatorPositition(){
        return elevatorCode.getCurrentPostion();
    }
    public void raiseElevatorToPosition_Autonomous(double cmd, int DesiredPos) {
        elevatorCode.raiseLowerElevatorToPosition_AUTO(cmd, DesiredPos);
    }

    public void setDesElevatorPosition_Teliop(int desHoldPosition){
        elevatorCode.set_elevator_desired_position(desHoldPosition);
    }
    public void elevatorUpdate(double dt){
        elevatorCode.updatePosControl(dt);
    }

    public void sweeperCommand(double cmd) {
       SweeperCode.setSweeperCommand(cmd);
    }
    public void dump_pixle() {
        elevatorCode.dump_pixle();
    }
    public void reset_pixle_bucket() {
        elevatorCode.reset_pixle_bucket();
    }

    public void launch_drone () {
        elevatorCode.release();
            }
    public void reset_launch () {
        elevatorCode.reset_launch_pin();
    }

    public void setDesWinchPosition_Teliop(int desHoldPosition){
        winchCode.set_winch_desired_position(desHoldPosition);
    }
    public void winchUpdate(double dt) {winchCode.updatePosControl(dt);}

    public void raiseLowerWinch(double cmd) {
        winchCode.raiseLowerWinch_T(cmd);
    }

    public int getWinchPositition(){
        return winchCode.getCurrentPostion();
    }

    public void raise_hook(){
        winchCode.raise_hook();
    }
    public void lower_hook(){
        winchCode.reset_hookServo();
    }


    //Teliop Auto controls
    public void setDesiredOrientation(double desOrenetationOfRobot){
        CASHDriveLibrary.setTeliopDesiredOrientation(desOrenetationOfRobot);
    }
    public void updateRobotRotation(double dt){
        CASHDriveLibrary.updateRobotRotation(dt);
    }


    public void setDesDistFromWall(double desiredDistFromWall){
        CASHDriveLibrary.setTeliopDesiredDistanceFromWall(desiredDistFromWall);
    }
    public void updateDesDistFromWall(double dt, double distSensor1, double distSensor2){
        CASHDriveLibrary.updateTeliopDesDistFromWall(dt,distSensor1, distSensor2);
    }
}

