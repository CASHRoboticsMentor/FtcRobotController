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

package org.firstinspires.ftc.teamcode.OldCode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Robot2024;
import org.firstinspires.ftc.teamcode.utilities.CASH_Drive_Library;

/**
 * This is the TeleOp Mode control class for 2021 robot.
 */

@TeleOp(name = "CASH 2024-2025 DrPepper", group = "Iterative Opmode")
@Disabled
public class CASH2024_25_DrPepper extends OpMode {
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
    boolean AutoExtendSlider;
    boolean AutoRetractSlider;
    private boolean RecieveTimerStarted = false;
    private Robot2024 robot;
    public CASH_Drive_Library CASHDriveLibrary;

    //Teleop variables
    private double percentTurnPower = .5;
    private boolean AutoDeliverSample = false;
    private boolean AutoReceiveSample = false;

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
        double powerfactor;
        if (gamepad1.right_bumper == true) {
            powerfactor = 0.5;
        } else {
            powerfactor = 1;

        }
        // INPUTS
        double drive_y = -gamepad1.left_stick_y * powerfactor;
        double drive_x = gamepad1.left_stick_x * powerfactor;
        double turn_x = 0.75 * gamepad1.right_stick_x * powerfactor;

        //Sample/Specimen Grabber
//        boolean horzRotateAction = gamepad1.left_bumper;
        boolean horzClawAction = gamepad1.right_bumper;
        boolean horzRotateAction = false;
//        boolean horzClawAction = false;

        //Vertical transfer claw
        boolean vertClawRotateAction = gamepad2.left_bumper;
        boolean vertClawAction = gamepad2.right_bumper;

        //Vertical Elevator Control
        double elevatorCommand = -gamepad2.left_stick_y;
        double horizSlideCommand = gamepad2.right_stick_x*.5;

        float autoExtend = gamepad1.right_trigger;
        float autoRetract = gamepad1.left_trigger;

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

        if (Math.abs(elevatorCommand) > .025) {
//            AutoElevatorActive = false;
            robot.raiseLowerElevator(elevatorCommand);
            robot.setDesElevatorPosition_Teliop(robot.getElevatorPositition());
        } else {
            robot.elevatorUpdate(loopTime.seconds());
        }
        RobotLog.i(String.format("elevator position %d",robot.getElevatorPositition()));


        // Control of the Slider
        if (autoExtend >0.1){
            AutoExtendSlider = true;
            AutoRetractSlider = false;

        }else if (autoRetract > 0.1){
            AutoExtendSlider = false;
            AutoRetractSlider = true;
        }

        if (AutoExtendSlider  && !clawRotateTimerActive){
            clawRotateDownTimer.reset();
            clawRotateTimerActive = true;
            robot.setDesSliderPosition(robot.EXTEND_POSITION);
            robot.GrabberOpen();
        }
        if (clawRotateTimerActive && clawRotateDownTimer.milliseconds()>500){
            robot.GrabberDown();
            clawRotateTimerActive = false;
        }

        if (AutoRetractSlider  && !clawRetractTimerActive){
            clawRetractTimer.reset();
            clawRetractTimerActive = true;
//            clawRotateDownTimer.reset();
//            clawRetractTimerActive = true;
//            robot.setDesSliderPosition(0);
            robot.GrabberUp();
            robot.openVertClaw();
        }
        if (clawRetractTimerActive && clawRetractTimer.milliseconds()>500){
            robot.setDesSliderPosition(0);
            clawRetractTimerActive = false;
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
        RobotLog.i(String.format("slider position %d",robot.getSliderPositition()));

        robot.moveRobotteli(drive_y, drive_x, turn_x);
        telemetry.addData("Status", "Running");
//            telemetry.addData("#ofTicks", robot.getTicks());
        telemetry.update();
//        loopTime.reset();


    }
}
