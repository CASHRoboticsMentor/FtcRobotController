/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.utilities.CASH_Drive_Library;

/**
 * This is the TeleOp Mode control class for 2021 robot.
 */

@TeleOp(name = "CASH 2024-2025 Meet1", group = "Iterative Opmode")
//@Disabled
public class CASH2024_25_Meet1 extends OpMode {
    /*
     * Code to run ONCE when the driver hits INIT
     */
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime loopTime = new ElapsedTime();

    private final ElapsedTime vertReceiveWaitTimer = new ElapsedTime();

    private final ElapsedTime clawRotateDownTimer = new ElapsedTime();
    boolean clawRotateTimerActive = false;

    private final ElapsedTime clawRetractTimer = new ElapsedTime();
    boolean clawRetractTimerActive = false;

    //Horizontal Servo control params
    private final ElapsedTime horzServoCmdTimer = new ElapsedTime();
    private boolean horzServoCmdTimerIsActive = false;
    private boolean grabberInUpPosition = true;

    private final ElapsedTime grabberServoCmdTimer = new ElapsedTime();
    private boolean grabberServoCmdTimerIsActive = false;
    private boolean grabberInOpenPosition = true;

    //Vertical Servo control params
    private final ElapsedTime vertRotateServoTimer = new ElapsedTime();
    private boolean vertRotateServoCmdTimerIsActive = false;
    private boolean vertInRecievePosition = true;

    private final ElapsedTime vertClawServoTimer = new ElapsedTime();
    private boolean vertClawServoCmdTimerIsActive = false;
    private boolean vertClawInOpenPosition = true;

    //Auto Controls
    boolean AutoExtendSlider = false;
    boolean AutoExtendSlider_GP2 = false;
    boolean AutoRetractSlider = false;
    private boolean RecieveTimerStarted = false;
    private Robot2024 robot;
    public CASH_Drive_Library CASHDriveLibrary;

    //Teleop variables
    private double percentTurnPower = .5;
    private boolean AutoDeliverSample = false;
    private boolean AutoReceiveSample = false;

    double slowFactor = 1;
    boolean slowModeActive = false;
    float prevSlowButtonState = 0;
    private final ElapsedTime slowModeTimer = new ElapsedTime();
    boolean isSlowModeTimerActive = false;

    boolean inStartup = true;
    boolean autoraiseactive = false;
    boolean transferTimerActive = false;

    boolean transferClawOpenIsActive = false;

    int HOLDPOS = 0;
    double previousElevatorCommand = 0;
    boolean putElevInHoldControl = false;
    @Override

    public void init() {

        robot = new Robot2024(this);

        robot.initializeRobot();
//        robot.initializeImplements();
        CASHDriveLibrary = robot.CASHDriveLibrary;

        //Set the robot up to use encoders on all 4 wheels.  This is then velocity controlled which
        //can be a bit better than just open loop.
//        CASHDriveLibrary.EnableEncoders();

        //This resets the angles to 0 degrees and will represent zero after robot is inicialized
        //This method can be used anytime you want to reset the angles to zero
//        robot.robotIMU.resetAngle();

        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY.
     */
    @Override
    public void start() {
        runtime.reset();
        loopTime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        /*
         *Place your Teliop code here.
         * 1) Take inputs from joysticks
         * 2) Use the inputs from the joystick to move the robot forward/rev
         */
        // INPUTS

        if (inStartup){
            robot.GrabberUp();
            robot.closeVertClaw();
            robot.GrabberOpen();
            inStartup = false;
        }
        boolean slowMode = gamepad1.b;
        if (slowMode && !isSlowModeTimerActive){

            isSlowModeTimerActive = true;
            slowModeTimer.reset();
        }
        if (isSlowModeTimerActive && slowModeTimer.milliseconds() > 250 ){
            slowModeActive = !slowModeActive;
            isSlowModeTimerActive = false;
        }

        if (slowModeActive){
            slowFactor = .25;
        }
        else{
            slowFactor = 1;
        }
        double drive_y = -gamepad1.left_stick_y * slowFactor;
        double drive_x = gamepad1.left_stick_x * slowFactor;
        double turn_x =  gamepad1.right_stick_x * slowFactor;

        //Sample/Specimen Grabber
//        boolean horzRotateAction = gamepad1.left_bumper;
        boolean horzClawAction = gamepad1.right_bumper;
        boolean horzRotateAction = false;
//        boolean horzClawAction = false;

        //Vertical transfer claw
        boolean vertClawRotateAction = gamepad2.left_bumper;
        boolean vertClawAction = gamepad2.right_bumper;

        //Vertical Elevator Control
        double elevatorCommand = -gamepad2.left_stick_y*1.0;
        double horizSlideCommand = gamepad2.right_stick_x*.25;

        float autoExtend = gamepad1.right_trigger;
        float autoExtend_GP2 = gamepad2.right_trigger;
        float autoRetract = gamepad1.left_trigger;
        boolean autoraise = gamepad2.a;

//        RobotLog.i(String.format("elevator Postioing %.2f",robot.getElevatorPositition()));
//        RobotLog.i(String.format("horizontal Position %.2f",));





        //  Teliop for the Horizontal claw rotate
        if (vertClawRotateAction && !vertRotateServoCmdTimerIsActive){
            vertRotateServoTimer.reset();
            vertRotateServoCmdTimerIsActive = true;
        }
        if (vertRotateServoCmdTimerIsActive) {
            if (vertInRecievePosition) {
                robot.vertClawToDeliverPosition(1);
                if (vertRotateServoTimer.milliseconds() > 250){
                    vertInRecievePosition = false;
                    vertRotateServoCmdTimerIsActive = false;
                }
            } else {
                robot.vertClawToReceivePosition(0);
                if (vertRotateServoTimer.milliseconds() > 250){
                    vertInRecievePosition = true;
                    vertRotateServoCmdTimerIsActive = false;
                }
            }
        }
        if (vertClawAction && !vertClawServoCmdTimerIsActive){
            vertClawServoTimer.reset();
            vertClawServoCmdTimerIsActive = true;
        }
        if (vertClawServoCmdTimerIsActive) {
            if (vertClawInOpenPosition) {
                robot.closeVertClaw();
                if (vertClawServoTimer.milliseconds() > 250){
                    vertClawInOpenPosition = false;
                    vertClawServoCmdTimerIsActive = false;
                }
            } else {
                robot.openVertClaw();
                if (vertClawServoTimer.milliseconds() > 250){
                    vertClawInOpenPosition = true;
                    vertClawServoCmdTimerIsActive = false;
                }
            }
        }

//        if (transferClawOpen>0) {
//            robot.openVertClaw(0.02);
//        }
//        if (transferClawClose>0) {
//            robot.closeVertClaw(.1666666666666666666666666666666666666666666666666666666666666666666);
//        }
//        if (transferClawUpPostion) {
//            robot.vertClawToDeliverPosition(1);
//        }
//        if (recievePosition) {
//            robot.vertClawToReceivePosition(0);
//        }


        //  Teliop for the Horizontal claw rotate
        if (horzRotateAction && !horzServoCmdTimerIsActive){
            horzServoCmdTimer.reset();
            horzServoCmdTimerIsActive = true;
        }
        if (horzServoCmdTimerIsActive) {
            if (grabberInUpPosition) {
                robot.GrabberDown();
                if (horzServoCmdTimer.milliseconds() > 250){
                    grabberInUpPosition = false;
                    horzServoCmdTimerIsActive = false;
                }
            } else {
                robot.GrabberUp();
                if (horzServoCmdTimer.milliseconds() > 250){
                    grabberInUpPosition = true;
                    horzServoCmdTimerIsActive = false;
                }
            }
        }
        //  Teliop for the Horizontal claw open close
        if (horzClawAction && !grabberServoCmdTimerIsActive){
            grabberServoCmdTimer.reset();
            grabberServoCmdTimerIsActive = true;
        }
        if (grabberServoCmdTimerIsActive) {
            if (grabberInOpenPosition) {
                robot.GrabberClose();
                if (grabberServoCmdTimer.milliseconds() > 250){
                    grabberInOpenPosition = false;
                    grabberServoCmdTimerIsActive = false;
                }
            } else {
                robot.GrabberOpen();
                if (grabberServoCmdTimer.milliseconds() > 250){
                    grabberInOpenPosition = true;
                    grabberServoCmdTimerIsActive = false;
                }
            }
        }


        //Elevator Control

        if (AutoDeliverSample) {
            robot.setDesElevatorPosition_Teliop(robot.HIGH_BASKET_POSITION);
            if (robot.getElevatorPositition() >= robot.HIGH_BASKET_POSITION) {
                robot.vertClawToDeliverPosition(1);
            }
        } else if (AutoReceiveSample) {
            if (!RecieveTimerStarted) {
                vertReceiveWaitTimer.reset();
                RecieveTimerStarted = true;
            }
            robot.vertClawToReceivePosition(0);
            robot.openVertClaw();
            if (vertReceiveWaitTimer.milliseconds() > 1000) {
                robot.setDesElevatorPosition_Teliop(robot.SAMPLE_RECEIVE_POSITION);
                RecieveTimerStarted = false;
            }
        }
        if (autoraise){
            robot.setDesElevatorPosition_Teliop(robot.HIGH_RUNG_POSITION);
            autoraiseactive = true;
        }

        double elevFinalCmd = 0;
        //if joystick is centered or is on its way to center we want to hold the last position when the command changes direction
        //cmd changing logic
//
//        if (elevatorCommand < previousElevatorCommand ){
//            //we need to set the elevaotr position to the enconder position at the prev command
//            robot.setDesElevatorPosition_Teliop(robot.getElevatorPositition());
//            putElevInHoldControl = true;
//        }
//
//        if (!putElevInHoldControl){
//            robot.raiseLowerElevator(elevatorCommand);
//        }else{
//            robot.elevatorUpdate(loopTime.seconds());
//        }
//
//        previousElevatorCommand = elevatorCommand;


//


        if (Math.abs(elevatorCommand) > .025) {
//            AutoElevatorActive = false;
            robot.raiseLowerElevator(elevatorCommand);
            HOLDPOS = robot.getElevatorPositition();
            autoraiseactive = false;
        }

        if (elevatorCommand > previousElevatorCommand) {
            
        }
            else {
            robot.setDesElevatorPosition_Teliop(HOLDPOS);
            robot.elevatorUpdate(loopTime.seconds());
        }

        RobotLog.i(String.format("ELEV: raw Command: %.4f Last Des Postion: %d HOLD POS: %d",elevatorCommand,robot.getElevatorPositition(), HOLDPOS));
//        RobotLog.i(String.format("driver position %d",robot.getDrivingEncoderPosition()));
        RobotLog.i(String.format("elevator position %d",robot.getElevatorPositition()));

        // Control of the Slider
        if (autoExtend >0.1){
            AutoExtendSlider = true;
            AutoRetractSlider = false;
            AutoExtendSlider_GP2 = false;

        }else if (autoExtend_GP2 > 0.1){
            AutoExtendSlider = true;
            AutoRetractSlider = false;
            AutoExtendSlider_GP2 = true;
        }else if (autoRetract > 0.1){
            AutoExtendSlider = false;
            AutoRetractSlider = true;
            AutoExtendSlider_GP2 = false;
        }
        else {
            AutoExtendSlider = false;
            AutoRetractSlider = false;
            AutoExtendSlider_GP2 = false;
        }

        if (AutoExtendSlider  && !clawRotateTimerActive){
            clawRotateDownTimer.reset();
            clawRotateTimerActive = true;
            if (AutoExtendSlider_GP2) {
                robot.setDesSliderPosition(robot.EXTEND_FOR_SPECIMEN);
                robot.GrabberOpen();
            }else {
                robot.setDesSliderPosition(robot.EXTEND_POSITION);
                robot.GrabberOpen();

            }
        }
        if (clawRotateTimerActive && clawRotateDownTimer.milliseconds()>500){
            robot.GrabberDown();
            clawRotateTimerActive = false;
        }
//        RobotLog.i(String.format("AutoRetractSlider %.3f  ",AutoRetractSlider));
        if (AutoRetractSlider  && !clawRetractTimerActive){
            clawRetractTimer.reset();
            clawRetractTimerActive = true;
//            clawRotateDownTimer.reset();
//            clawRetractTimerActive = true;
//            robot.setDesSliderPosition(0);
            robot.GrabberUp();


        }
//        RobotLog.i(String.format("clawRetractTimer %.3f",clawRetractTimer.milliseconds()));
        if (clawRetractTimerActive && clawRetractTimer.milliseconds()>1250){
            robot.setDesSliderPosition(0);
            robot.openVertClaw();
            clawRetractTimerActive = false;
            transferTimerActive = true;

        }
        if (robot.getSliderPositition() < 100 && clawRetractTimer.milliseconds() > 2750 && transferTimerActive){
            robot.closeVertClaw();
//            robot.GrabberOpen();
            transferTimerActive = false;
            transferClawOpenIsActive = true;
        }
        if (clawRetractTimer.milliseconds()> 3000 && transferClawOpenIsActive){
            robot.GrabberOpen();
            transferClawOpenIsActive = false;
        }


//        if (AutoExtendSlider || AutoRetractSlider){
////            grabberInUpPosition = true;
////            robot.GrabberToPostion(.6666);

        if (Math.abs(horizSlideCommand) > .1 || horzClawAction || horzRotateAction) {
            AutoExtendSlider = false;
            AutoRetractSlider = false;
            robot.extendSlider(horizSlideCommand);
            robot.setDesSliderPosition(robot.getSliderPositition());
        } else {
            robot.sliderUpdate(loopTime.seconds());
        }
        RobotLog.i(String.format("LF:%d RF:%d LR:%d RR:%d",robot.getLFEncoder(),robot.getRFEncoder(),robot.getLREncoder(),robot.getRREncoder()));

//        RobotLog.i(String.format("elapsed Time %.3f",loopTime.seconds()));
        robot.moveRobotteli(drive_y, drive_x, turn_x);
        telemetry.addData("Status", "Running");
//            telemetry.addData("#ofTicks", robot.getTicks());
        telemetry.update();
        loopTime.reset();


    }
}
