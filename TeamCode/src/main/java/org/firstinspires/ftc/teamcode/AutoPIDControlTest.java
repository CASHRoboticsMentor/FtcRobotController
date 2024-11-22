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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.utilities.CASH_Drive_Library;

/**
 * This is the TeleOp Mode control class for 2021 robot.
 */

@Autonomous(name = "AutoPIDControlTest", group = "Iterative Opmode")
//@Disabled
public class AutoPIDControlTest extends OpMode {
    /*
     * Code to run ONCE when the driver hits INIT
     */
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime loopTime = new ElapsedTime();

    private Robot2024 robot;
    public CASH_Drive_Library CASHDriveLibrary;

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

        robot.setDesiredOrientation(90);
        robot.updateRobotRotation(loopTime.seconds());

        telemetry.addData("Status", "Running");
//            telemetry.addData("#ofTicks", robot.getTicks());
        telemetry.update();
        loopTime.reset();


    }
}
