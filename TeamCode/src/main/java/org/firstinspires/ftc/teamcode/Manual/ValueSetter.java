package org.firstinspires.ftc.teamcode.Manual;
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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.IOSubsystem;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@Config
@TeleOp(name="ValueSetter", group="Movement")
public class ValueSetter extends OpMode
{
    // Declare OpMode members.
    public static double kP = 0.0013;
    public static double kI = 0;
    public static double kD = 0.00014;
    public static double kF = 0.1;

    public static double kP2 = 0.017;
    public static double kI2 = 0;
    public static double kD2 = 0.0006;

    public static double targetSlider = 0;

    public static double YawPos = 90;
    public static double PitchPos = 90;

    public static double GRIPPER = 0.5;

    public static double ARM_POS = 0.5;




    private ElapsedTime timeAngle;

    private IOSubsystem IO;

    private Telemetry telemetryA;
    PIDController anglePID;
    PIDController sliderPID;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        anglePID = new PIDController(kP, kI, kD);
        sliderPID = new PIDController(kP2, kI2, kD2);

        timeAngle = new ElapsedTime();
        timeAngle.reset();

        IO = new IOSubsystem(hardwareMap);
//
        IO.initDiffy();

//        IO.asc_stage = IOSubsystem.ASCENT_STAGE.READY;

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        timeAngle.reset();
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {

    }

//    public static double trapezoidalMotionProfile(double maxAcceleration, double maxVelocity, double distance, double elapsedTime)
//    {
//        // 1) Time to accelerate from 0 to maxVelocity
//        double accelTime = maxVelocity / maxAcceleration;               // s
//        double accelDistance = 0.5 * maxAcceleration * accelTime * accelTime;
//
//        // 2) Check if we actually reach maxVelocity
//        //    If the distance needed for accel + decel is more than total distance,
//        //    we can't reach maxVelocity (triangle profile).
//        double halfDistance = distance / 2.0;
//        if (accelDistance > halfDistance) {
//            // Recompute accelTime so that we accelerate half the distance
//            accelTime = Math.sqrt(halfDistance / (0.5 * maxAcceleration));
//            accelDistance = 0.5 * maxAcceleration * (accelTime * accelTime);
//        }
//
//        // The actual maxVelocity we can reach
//        double actualMaxVelocity = maxAcceleration * accelTime;
//
//        // 3) Deceleration time is symmetric in this example
//        double decelTime = accelTime;
//
//        // 4) Cruise (constant velocity) distance
//        double cruiseDistance = distance - 2.0 * accelDistance;
//        double cruiseTime = 0.0;
//        if (cruiseDistance > 0) {
//            cruiseTime = cruiseDistance / actualMaxVelocity;
//        }
//
//        // 5) Total time
//        double totalTime = accelTime + cruiseTime + decelTime;
//
//        // --- PHASE CHECKS ---
//
//        // A) If we're past the total time, we're at the end
//        if (elapsedTime >= totalTime) {
//            return distance;
//        }
//
//        // B) Acceleration phase: t in [0, accelTime)
//        if (elapsedTime < accelTime) {
//            // x(t) = 0.5 * a * t^2
//            return 0.5 * maxAcceleration * elapsedTime * elapsedTime;
//        }
//
//        // C) Cruise phase: t in [accelTime, accelTime + cruiseTime)
//        double cruiseStart = accelTime;
//        double cruiseEnd = cruiseStart + cruiseTime;
//        if (elapsedTime < cruiseEnd) {
//            // Distance traveled after full accel
//            double distanceAtAccelEnd = 0.5 * maxAcceleration * accelTime * accelTime;
//            // Time spent in cruise
//            double tCruise = elapsedTime - cruiseStart;
//            // x(t) = distanceAtAccelEnd + v_max * tCruise
//            return distanceAtAccelEnd + actualMaxVelocity * tCruise;
//        }
//
//        // D) Deceleration phase: t in [cruiseEnd, totalTime)
//        double decelStart = cruiseEnd;
//        double tDecel = elapsedTime - decelStart;
//
//        // Distance traveled up to decel start
//        double distanceAtAccelEnd = 0.5 * maxAcceleration * accelTime * accelTime;
//        double distanceAtCruiseEnd = distanceAtAccelEnd + actualMaxVelocity * cruiseTime;
//
//        // x_decel(t) = v_max * tDecel - 0.5 * a * tDecel^2
//        return distanceAtCruiseEnd
//                + actualMaxVelocity * tDecel
//                - 0.5 * maxAcceleration * tDecel * tDecel;
//    }
//
//
//    public double motion_profile_position(double max_acceleration, double max_velocity, double distance, double elapsed_time) {
//        // Return the current reference position based on the given motion profile times, maximum acceleration, velocity, and current time.
//        // Calculate the time it takes to accelerate to max velocity
//
//        double acceleration_dt = max_velocity / max_acceleration;
//
//        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
//        double halfway_distance = distance / 2;
//        double acceleration_distance = 0.5 * max_acceleration * (acceleration_dt*acceleration_dt);
//
//        if (acceleration_distance > halfway_distance) {
//            acceleration_dt = Math.sqrt(halfway_distance / (0.5 * max_acceleration));
//        }
//
//        acceleration_distance = 0.5 * max_acceleration * (acceleration_dt*acceleration_dt);
//
//        // recalculate max velocity based on the time we have to accelerate and decelerate
//        max_velocity = max_acceleration * acceleration_dt;
//
//        // we decelerate at the same rate as we accelerate
//        double deceleration_dt = acceleration_dt;
//
//        // calculate the time that we're at max velocity
//        double cruise_distance = distance - 2 * acceleration_distance;
//        double cruise_dt = cruise_distance / max_velocity;
//        double deceleration_time = acceleration_dt + cruise_dt;
//
//        // check if we're still in the motion profile
//        double entire_dt = acceleration_dt + cruise_dt + deceleration_dt;
//        if (elapsed_time > entire_dt) {
//            return distance;
//        }
//
//        // if we're accelerating
//        if (elapsed_time < acceleration_dt) {
//            // use the kinematic equation for acceleration
//            return 0.5 * max_acceleration * (elapsed_time*elapsed_time);
//        }
//
//        // if we're cruising
//        else if (elapsed_time < deceleration_time) {
//            acceleration_distance = 0.5 * max_acceleration * (acceleration_dt*acceleration_dt);
//            double cruise_current_dt = elapsed_time - acceleration_dt;
//
//            // use the kinematic equation for constant velocity
//            return acceleration_distance + max_velocity * cruise_current_dt;
//        }
//
//        // if we're decelerating
//        else {
//            acceleration_distance = 0.5 * max_acceleration * (acceleration_dt*acceleration_dt);
//            cruise_distance = max_velocity * cruise_dt;
//            deceleration_time = elapsed_time - deceleration_time;
//
//            // use the kinematic equations to calculate the instantaneous desired position
//            return acceleration_distance + cruise_distance + max_velocity * deceleration_time - 0.5 * max_acceleration * (deceleration_time*deceleration_time);
//        }
//    }

    double ticks_in_degrees = 8192.00 / 360.00;

    public static double targetAngle = 0;




    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        sliderPID.setPID(kP2, kI2, kD2);
        anglePID.setPID(kP, kI, kD);

//        double timeSeconds = timeAngle.seconds();
//        double instantTargetPosition = trapezoidalMotionProfile(150,150, targetAngle-IO.getAngleMeasurement(), timeSeconds);
        double output = anglePID.calculate(IO.getAngleMeasurement(), targetAngle);
        double ff = Math.cos(Math.toRadians(targetAngle / ticks_in_degrees)) * kF;
//        double output = anglePID.calculate(IO.getAngleMeasurement(), instantTargetPosition);
        double output2 = sliderPID.calculate(IO.getSliderPosition(), targetSlider);


//        IO.setDiffyYaw(YawPos);
//        IO.setDiffyPitch(PitchPos);

        double powerF = output + ff;

        double powerF2 = output2;

//        IO.setSliderPower(powerF2);
//        IO.setArmPosition(ARM_POS);
//
//        IO.setGripperState(GRIPPER);
//
//        IO.DiffyLeft(PitchPos);
//        IO.DiffyRight(YawPos);

//        IO.setAnglePower(powerF * powerF * powerF);

//        IO.setPid(kP, kI, kD);
//

//        IO.setAnglePower(powerF*powerF*powerF);
//        IO.setAngleTarget(targetAngle);
//        IO.updateAngle();


//        IO.setArmPosition(0.3);
//
//        IO.setDiffyYaw(YawPos);
//
//        IO.setDiffyPitch(PitchPos);


//        IO.setArmPosition(IO.ARM_INIT);

        IO.setArmPosition(ARM_POS);
//
//          IO.setArmPosition(ARM_POS);
//          IO.setDiffyPitch(PitchPos);

//        IO.setArmPosition(0.5);
//        IO.setGripperState(GRIPPER);


//        IO.arm1(0.5);



        telemetryA.addData("currentAnglePosition", IO.getAngleMeasurement());
        telemetryA.addData("angleTarget", targetAngle);
        telemetryA.addData("currentSliderPosition", IO.getSliderPosition());
        telemetryA.addData("sliderTarget", targetSlider);
//        telemetryA.addData("instantTargetPosition", instantTargetPosition);
        telemetryA.addData("ff", Math.cos(Math.toRadians(targetAngle / ticks_in_degrees)));
        telemetryA.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

}
