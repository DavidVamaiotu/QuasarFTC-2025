package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

public class IOSubsystem extends SubsystemBase {
    private final CachingDcMotorEx slider1;

    private final CachingDcMotorEx slider2;

    private final DcMotorEx MoveAndDestroy;

    private final CachingServo Diffy1;

    private final CachingServo Diffy2;

    private ElapsedTime time = new ElapsedTime();

    public enum IO_STAGE {
        INTAKE,
        OUTTAKE
    }

    public IO_STAGE stage = IO_STAGE.INTAKE;

    public enum SPECIMEN_STAGE {
        UNINITIALIZED,
        LOADING_SPECIMEN,
        PLACING_SPECIMEN,
    }

    public SPECIMEN_STAGE specStage = SPECIMEN_STAGE.UNINITIALIZED;

    private double kP2 = 0;
    private double kI2 = 0;
    private double kD2 = 0;


    private double kP = 0;
    private double kI = 0;
    private double kD = 0;
    private double kF = 0;


    private double currentDiffyYaw = 90;
    private double currentDiffyPitch = 90;
    private double currentDiffy1Position = 0.5;
    private double currentDiffy2Position = 0.5;
    private final double TICK_PER_DEGREE = 0.0028169014084507;

    PIDFController anglePID;
    PIDController sliderPID;

    double targetAngle = 0;

    double targetSlider = 0;

    public IOSubsystem(final HardwareMap hMap) {
        anglePID = new PIDFController(kP, kI, kD, kF);
        sliderPID = new PIDController(kP2, kI2, kD2);
        slider1 = new CachingDcMotorEx(hMap.get(DcMotorEx.class, "sld1"));
        slider2 = new CachingDcMotorEx(hMap.get(DcMotorEx.class, "sld2"));
        MoveAndDestroy = hMap.get(DcMotorEx.class, "angle");
        Diffy1 = new CachingServo(hMap.get(Servo.class, "diffy1"));
        Diffy2 = new CachingServo(hMap.get(Servo.class, "diffy2"));

        Diffy2.setDirection(Servo.Direction.REVERSE);

        slider1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MoveAndDestroy.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slider1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MoveAndDestroy.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setPower(double power) {
         slider1.setPower(power);
         slider2.setPower(power);
    }


    public double motion_profile_position(double max_acceleration, double max_velocity, double distance, double elapsed_time) {
       // Return the current reference position based on the given motion profile times, maximum acceleration, velocity, and current time.
        // Calculate the time it takes to accelerate to max velocity

        double acceleration_dt = max_velocity / max_acceleration;

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        double halfway_distance = distance / 2;
        double acceleration_distance = 0.5 * max_acceleration * (acceleration_dt*acceleration_dt);

        if (acceleration_distance > halfway_distance) {
            acceleration_dt = Math.sqrt(halfway_distance / (0.5 * max_acceleration));
        }

        acceleration_distance = 0.5 * max_acceleration * (acceleration_dt*acceleration_dt);

        // recalculate max velocity based on the time we have to accelerate and decelerate
        max_velocity = max_acceleration * acceleration_dt;

        // we decelerate at the same rate as we accelerate
        double deceleration_dt = acceleration_dt;

        // calculate the time that we're at max velocity
        double cruise_distance = distance - 2 * acceleration_distance;
        double cruise_dt = cruise_distance / max_velocity;
        double deceleration_time = acceleration_dt + cruise_dt;

        // check if we're still in the motion profile
        double entire_dt = acceleration_dt + cruise_dt + deceleration_dt;
        if (elapsed_time > entire_dt) {
            return distance;
        }

        // if we're accelerating
        if (elapsed_time < acceleration_dt) {
            // use the kinematic equation for acceleration
            return 0.5 * max_acceleration * (elapsed_time*elapsed_time);
        }

        // if we're cruising
        else if (elapsed_time < deceleration_time) {
            acceleration_distance = 0.5 * max_acceleration * (acceleration_dt*acceleration_dt);
            double cruise_current_dt = elapsed_time - acceleration_dt;

            // use the kinematic equation for constant velocity
            return acceleration_distance + max_velocity * cruise_current_dt;
        }

        // if we're decelerating
        else {
            acceleration_distance = 0.5 * max_acceleration * (acceleration_dt*acceleration_dt);
            cruise_distance = max_velocity * cruise_dt;
            deceleration_time = elapsed_time - deceleration_time;

            // use the kinematic equations to calculate the instantaneous desired position
            return acceleration_distance + cruise_distance + max_velocity * deceleration_time - 0.5 * max_acceleration * (deceleration_time*deceleration_time);
        }
    }

    public void updateAngle() {

        double instantTargetPosition = motion_profile_position(3, 3, targetAngle-MoveAndDestroy.getCurrentPosition(), time.seconds());

        double output = anglePID.calculate(
                MoveAndDestroy.getCurrentPosition(), instantTargetPosition
        );

        MoveAndDestroy.setPower(output);
    }

    public void updatePosition() {

        double output = sliderPID.calculate(
                slider1.getCurrentPosition(), targetSlider
        );

        slider1.setPower(output);
        slider2.setPower(output);
    }

    public void setAngleTarget(double angleTicks) {
        time.reset();
        targetAngle = angleTicks;
    }

    public void setSliderTarget(double sliderTicks) {
        targetSlider = sliderTicks;
    }

    public void toPosition()
    {
        switch(stage) {
            case INTAKE:
//                targetAngle = 1300;
//                targetSlider = 0;
                specStage = SPECIMEN_STAGE.UNINITIALIZED;
                stage = IO_STAGE.OUTTAKE;
                break;
            case OUTTAKE:
//                targetAngle = 0;
//                setSliderTarget(1900);
                specStage = SPECIMEN_STAGE.UNINITIALIZED;
                stage = IO_STAGE.INTAKE;
                break;
        }
    }


    public void specimenCycle()
    {
        switch(specStage) {
            case UNINITIALIZED:
                targetAngle = 700;
                targetSlider = 0;
                //schimba unghiul bratului aici
                specStage = SPECIMEN_STAGE.LOADING_SPECIMEN;
                break;
            case LOADING_SPECIMEN:
//                targetAngle = 1900;
//                targetSlider = 700;
                specStage = SPECIMEN_STAGE.PLACING_SPECIMEN;
                break;
            case PLACING_SPECIMEN:
//              targetSlider = 1300;
                // da drumu la gripper aici
                specStage = SPECIMEN_STAGE.UNINITIALIZED;
                break;
        }
    }

    public void initDiffy()
    {
        currentDiffyYaw = 90;
        currentDiffyPitch = 90;
        currentDiffy1Position = 0.5;
        currentDiffy2Position = 0.5;
        Diffy1.setPosition(0.5);
        Diffy2.setPosition(0.5);
    }
    public void setDiffyYaw(double deg)
    {
        double yawDifference = deg - currentDiffyYaw;
        double addedPosition = yawDifference * TICK_PER_DEGREE;

        Diffy1.setPosition(currentDiffy1Position + addedPosition);
        Diffy2.setPosition(currentDiffy2Position - addedPosition);

        currentDiffy1Position = currentDiffy1Position + addedPosition;
        currentDiffy2Position = currentDiffy2Position - addedPosition;
        currentDiffyYaw = deg;

    }
    public void setDiffyPitch(double deg)
    {
        double pitchDifference = deg - currentDiffyPitch;
        double addedPosition = pitchDifference * TICK_PER_DEGREE;

        Diffy1.setPosition(currentDiffy1Position + addedPosition);
        Diffy2.setPosition(currentDiffy2Position + addedPosition);

        currentDiffy1Position = currentDiffy1Position + addedPosition;
        currentDiffy2Position = currentDiffy2Position + addedPosition;
        currentDiffyPitch = deg;
    }

    public double getAnglePosition()
    {
        return MoveAndDestroy.getCurrentPosition();
    }

    public void setAnglePower(double power)
    {
        MoveAndDestroy.setPower(power);
    }

    public void setSlidersPower(double power)
    {
        slider1.setPower(power);
        slider2.setPower(power);
    }

    public double getSliderPosition()
    {
        return slider1.getCurrentPosition();
    }

}