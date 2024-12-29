//package org.firstinspires.ftc.teamcode.Manual;/* Copyright (c) 2017 FIRST. All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted (subject to the limitations in the disclaimer below) provided that
// * the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright notice, this list
// * of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright notice, this
// * list of conditions and the following disclaimer in the documentation and/or
// * other materials provided with the distribution.
// *
// * Neither the name of FIRST nor the names of its contributors may be used to endorse or
// * promote products derived from this software without specific prior written permission.
// *
// * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// */
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.HardwareRobot;
//import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.Subsystems.OuttakeSubsystem;
//
///*
// * This file contains an example of an iterative (Non-Linear) "OpMode".
// * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
// * The names of OpModes appear on the menu of the FTC Driver Station.
// * When a selection is made from the menu, the corresponding OpMode
// * class is instantiated on the Robot Controller and executed.
// *
// * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
// * It includes all the skeletal structure that all iterative OpModes contain.
// *
// * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
// * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
// */
//@Config
//@TeleOp(name="PIDSetter", group="Movement")
//public class PIDSetter extends OpMode
//{
//    // Declare OpMode members.
//
//
//    public static double kP = 0.025;
//    public static double kI = 0;
//    public static double kD = 0.0005;
//
//    public static double target = -200;
//
//
//    public static double YawPos = 90;
//    public static double PitchPos = 90;
//
//    public static double servoPos1 = 0.5;
//
//    public static double servoPos2 = 0.5;
//
//
//    public static double GRIPPER = 0.5;
//
//    public static double YAW = 0.5;
//    public static double PITCH = 0.6;
//    public static double armAngle = 0.85;
//
//    public static double ARTICULATION = 0.58;
//
//    public static double GRIPPER2 = 0.3;
//
//    public static double ARM2 = 0.62;
//
//    private IntakeSubsystem intake;
//
//    private OuttakeSubsystem outtake;
//
//    private Telemetry telemetryA;
//    PIDController pd;
//
//
//
//    /*
//     * Code to run ONCE when the driver hits INIT
//     */
//    @Override
//    public void init() {
//        pd = new PIDController(kP, kI, kD);
//
//        intake = new IntakeSubsystem(hardwareMap);
//        outtake = new OuttakeSubsystem(hardwareMap);
//
//        outtake.initDiffy();
//
//        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
//    }
//
//    /*
//     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
//     */
//    @Override
//    public void init_loop() {
//    }
//
//    /*
//     * Code to run ONCE when the driver hits START
//     */
//    @Override
//    public void start() {
//
//    }
//
//    /*
//     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
//     */
//    @Override
//    public void loop() {
////        pd.setPID(kP, kI, kD);
////        double output = pd.calculate(outtake.getPosition(), target);
//
////        outtake.setDiffyYaw(YawPos);
//        outtake.setDiffyYaw(YawPos);
//        outtake.setDiffyPitch(PitchPos);
//
//        intake.setSliderPosition(servoPos1);
//        intake.setServoPos2(servoPos2);
//        intake.setServoPos1(servoPos1);
//        intake.setPitch(PITCH);
//        intake.setYaw(YAW);
//        intake.gripState(GRIPPER);
//        intake.ArmAngle(armAngle);
//        outtake.setArmAngle(ARM2);
////        outtake.setGripperState(GRIPPER2);
//        outtake.setArticulationAngle(ARTICULATION);
////        outtake.setPower(output);
//
//        telemetryA.addData("currentPosition", outtake.getPosition());
//        telemetryA.addData("targetPosition", target);
//
//        telemetryA.addData("target1", outtake.getTarget1());
//        telemetryA.addData("target2", outtake.getTarget2());
//        telemetryA.update();
//    }
//
//    /*
//     * Code to run ONCE after the driver hits STOP
//     */
//    @Override
//    public void stop() {
//
//    }
//
//}
