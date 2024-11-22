package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.utilities.CASH_Drive_Library;
import org.firstinspires.ftc.teamcode.utilities.CashServo;
import org.firstinspires.ftc.teamcode.utilities.ElevatorControl;
import org.firstinspires.ftc.teamcode.utilities.IMUUtility;
import org.firstinspires.ftc.teamcode.utilities.IMUUtility2;
import org.firstinspires.ftc.teamcode.utilities.SweeperControl;
import org.firstinspires.ftc.teamcode.utilities.VertClawElevatorControl;
import org.firstinspires.ftc.teamcode.utilities.HorizontalSliderControl;
import org.firstinspires.ftc.teamcode.utilities.WinchControl;
import org.firstinspires.ftc.teamcode.utilities.pid_controller;

public class Robot2024<_opMode> {

    public Robot2024(OpMode opMode) {
        _opMode = opMode;
        elevatorControl = new VertClawElevatorControl(_opMode,"vert_elev_motor");
        horizontalSlideControl = new HorizontalSliderControl(_opMode,"horz_slide_motor");
    }

    ///// Vertical Elevator Controls /////
    /* The vertical elevator controls the elevator up and down as well as the servos to move samples
    and specimens into oposition.  All the functions that can be call by the robot will be defined here
     */
    private VertClawElevatorControl elevatorControl;
    //elevator control functions
    public void raiseLowerElevator(double cmd) {
        elevatorControl.raiseLowerElevator_T(cmd);
    }
    public int getElevatorPositition(){
        return elevatorControl.getCurrentPostion();
    }
    public void raiseElevatorToPosition_Autonomous(double cmd, int DesiredPos) {
        elevatorControl.raiseLowerElevatorToPosition_AUTO(cmd, DesiredPos);
    }
    public void setDesElevatorPosition_Teliop(int desHoldPosition){
        elevatorControl.set_elevator_desired_position(desHoldPosition);
    }
    public void elevatorUpdate(double dt){
        elevatorControl.updatePosControl(dt);
    }
    public void closeVertClaw (){
        elevatorControl.closeClaw(.02);
    }
    public void openVertClaw(){
        elevatorControl.openClaw(.1666666);
    }
    public void vertClawToDeliverPosition(double cmd){
        elevatorControl.clawDeliverPosition(cmd);
    }
    public void vertClawToReceivePosition(double cmd){
        elevatorControl.clawReceivePosition(cmd);
    }

    private HorizontalSliderControl horizontalSlideControl;
    public int EXTEND_POSITION = 840;
    public int EXTEND_FOR_SPECIMEN = (int)(.75*(840*.5));
    public void extendSlider(double cmd) {
        horizontalSlideControl.ExtendRetract_Slider(cmd);
    }
    public int getSliderPositition(){
        return horizontalSlideControl.getCurrentPostion();
    }
    public void extentSliderToPosition_Autonomous(double cmd, int DesiredPos) {
        horizontalSlideControl.extendSliderToPosition_AUTO(cmd, DesiredPos);
    }
    public void setDesSliderPosition(int desHoldPosition){
        horizontalSlideControl.set_slider_desired_position(desHoldPosition);
    }
    public void sliderUpdate(double dt){
        horizontalSlideControl.updatePosControl(dt);
    }
//    public void closeGrabber (double cmd){
//        horizontalSlideControl.closeClaw(cmd);
//    }
//    public void openGrabber(double cmd){
//        horizontalSlideControl.openClaw(cmd);
//    }
    public void setToDliverPosition(double cmd){
        horizontalSlideControl.clawDeliverPosition(cmd);
    }
    public void setToGetSample(double cmd){
        horizontalSlideControl.clawReceivePosition(cmd);
    }

    public int AUTO_VERT_DELIVER_UPPER_POSITION = 1550;
    public int AUTO_VERT_DELIVER_LOWER_POSITION = 840;



    public final OpMode _opMode; //holds opmode object


    //Define Robot Motor variables
    public DcMotor leftFrontMotor = null;
    public DcMotor rightFrontMotor = null;
    public DcMotor leftRearMotor = null;
    public DcMotor rightRearMotor = null;

    //Creates a new object for the robot IMU
    public IMUUtility2 robotIMU = new IMUUtility2();

    //Creates a new object for the Drive Library
    public CASH_Drive_Library CASHDriveLibrary = new CASH_Drive_Library();

    //Creates a new object of type Elevator Control
    //All Elevator Controls will be writen in Elevator Control


    /*
    //Control directions of the robot.  These should not be changed as these are specific to how the robot is designed
    */
    public double FORWARD = CASHDriveLibrary.FORWARD;
    public double REVERSE = CASHDriveLibrary.REVERSE;
    public double RIGHT = CASHDriveLibrary.RIGHT;
    public double LEFT = CASHDriveLibrary.LEFT;

    public double TURN_RIGHT = CASHDriveLibrary.TURN_RIGHT;
    public double TURN_LEFT = CASHDriveLibrary.TURN_LEFT;
//
    public int HIGH_RUNG_POSITION = 2000;
    public int LOWER_RUNG_POSITION = 1000;
    public int HIGH_BASKET_POSITION = 2000;
    public int LOW_BASKET_POSITION = 1000;
    public int SAMPLE_RECEIVE_POSITION = 200;
//    public int ELEVATOR_HIGH_POSITION = elevatorControl.HIGH_POSITION;

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
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftRearMotor = _opMode.hardwareMap.get(DcMotor.class, "left_rear_drive");
        leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontMotor = _opMode.hardwareMap.get(DcMotor.class, "left_front_drive");
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFrontMotor = _opMode.hardwareMap.get(DcMotor.class, "right_front_drive");
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Initialize objects in common lib
        CASHDriveLibrary.init(_opMode);
        CASHDriveLibrary.leftFrontMotor = leftFrontMotor;
        CASHDriveLibrary.rightFrontMotor = rightFrontMotor;
        CASHDriveLibrary.leftRearMotor = leftRearMotor;
        CASHDriveLibrary.rightRearMotor = rightRearMotor;
        CASHDriveLibrary.EnableEncoders();

        robotIMU.initialize(_opMode,"imu");
        CASHDriveLibrary.imu = robotIMU;
//        CASHDriveLibrary.imu.resetAngle();

        CASHDriveLibrary.init_rotations_pid();
        CASHDriveLibrary.init_distanceToWall_pid();

        this.elevatorControl.resetElevatorMotorEncoder();
        this.horizontalSlideControl.resetElevatorMotorEncoder();
    }

    public void resetIMU(){
        robotIMU.resetAngle();
    }

    public void GrabberUp(){
        horizontalSlideControl.clawDeliverPosition(.1);
    }

    public void GrabberToPostion(double desPos){
        horizontalSlideControl.clawDeliverPosition(desPos);
    }
    public void GrabberDown(){
        horizontalSlideControl.clawReceivePosition(1);
    }
    public void GrabberOpen(){
        horizontalSlideControl.openClaw(.35);
    }
    public void GrabberClose() {
        horizontalSlideControl.closeClaw(.51);
    }


    //ONLY USE THIS FOR TELIPOP
    //elevatorCode.intit2:  This initializes the implements but doesn't reset the encoders.
//    public void initializeImplements() {
//        elevatorControl.resetElevatorMotorEncoder();
//    }


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

    public int getDrivingEncoderPosition(){
        return CASHDriveLibrary.getRightRearEncoderTick();
    }
    public int getRREncoder(){
        return CASHDriveLibrary.getRREncoder();
    }
    public int getLREncoder(){
        return CASHDriveLibrary.getLREncoder();
    }
    public int getLFEncoder(){
        return CASHDriveLibrary.getLFEncoder();
    }
    public int getRFEncoder(){
        return CASHDriveLibrary.getRFEncoder();
    }
}

