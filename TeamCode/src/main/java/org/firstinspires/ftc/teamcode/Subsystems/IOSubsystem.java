package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
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

    private final CachingDcMotorEx MoveAndDestroy;

    private final CachingServo Diffy1;

    private final CachingServo arm;

    private final CachingServo gripper;

    private final CachingServo arm2;

    private final DcMotorEx enc;

    private final CachingServo Diffy2;
    public enum IO_STAGE {
        INTAKE,
        OUTTAKE,
        OUTTAKE_UNLOADING,
        INTAKE_LOADING
    }

    public IO_STAGE stage = IO_STAGE.INTAKE;

    public enum SPECIMEN_STAGE {
        UNINITIALIZED,
        LOADING_SPECIMEN,
        PLACING_SPECIMEN,
    }

    public SPECIMEN_STAGE specStage = SPECIMEN_STAGE.UNINITIALIZED;

    private double kP2 = 0.02;
    private double kI2 = 0;
    private double kD2 = 0.0003;


    private double kP = 0.001;
    private double kI = 0;
    private double kD = 0.0001;
    public double kF = 0.25;

    public boolean HoldPosition = false;


    private double currentDiffyYaw = 90;
    private double currentDiffyPitch = 90;
    private double currentDiffy1Position = 0.5;
    private double currentDiffy2Position = 0.5;
    private final double TICK_PER_DEGREE = 0.0028169014084507;

    PIDController anglePID;
    PIDController sliderPID;

    double targetAngle = 0;

    double targetSlider = 0;

    public double GRIPPING = 0.24;
    public double NOT_GRIPPING = 0;
    public double PLACING_SAMPLE = 0.4;
    public double LOADING_SAMPLE = 0.48;
    public double LOADING_SPECIMEN = 0.6;

    public double ARM_INIT = 0.1;
    public double PITCH_TAKING_SAMPLE = 0;


    public IOSubsystem(final HardwareMap hMap) {
        anglePID = new PIDController(kP, kI, kD);
        sliderPID = new PIDController(kP2, kI2, kD2);
        slider1 = new CachingDcMotorEx(hMap.get(DcMotorEx.class, "sldSt"));
        slider2 = new CachingDcMotorEx(hMap.get(DcMotorEx.class, "sldDr"));
        MoveAndDestroy = new CachingDcMotorEx(hMap.get(DcMotorEx.class, "angle"));
        enc = hMap.get(DcMotorEx.class, "feed");
        Diffy1 = new CachingServo(hMap.get(Servo.class, "difL"));
        Diffy2 = new CachingServo(hMap.get(Servo.class, "difD"));
        arm = new CachingServo(hMap.get(Servo.class, "arm"));
        arm2 = new CachingServo(hMap.get(Servo.class, "armX"));
        gripper = new CachingServo(hMap.get(Servo.class, "grip"));


        slider1.setDirection(DcMotorSimple.Direction.REVERSE);
        slider1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        Diffy2.setDirection(Servo.Direction.REVERSE);
        arm2.setDirection(Servo.Direction.REVERSE);
        enc.setDirection(DcMotorSimple.Direction.FORWARD);

        enc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slider1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setSliderPower(double power) {
         slider1.setPower(power);
         slider2.setPower(power);
    }


    double ticks_in_degrees = 8192.00 / 360.00;

    public void updateAngle() {

//        double instantTargetPosition = motion_profile_position(3, 3, targetAngle-MoveAndDestroy.getCurrentPosition(), time.seconds());

        double output = anglePID.calculate(
                getAngleMeasurement(), targetAngle
        );

        double ff = targetAngle <= 10 ? 0 : (HoldPosition == false ? (Math.cos(Math.toRadians(targetAngle / ticks_in_degrees)) * kF) : 0.2);

        setAnglePower(output + ff);
    }

    public void updatePosition() {

        double output = sliderPID.calculate(
                getSliderPosition(), targetSlider
        );

        setSliderPower(output);
    }

    public void setAngleTarget(double angleTicks) {
        targetAngle = angleTicks;
    }

    public void setArmPosition(double pos) {
        arm.setPosition(pos+0.01);
        arm2.setPosition(pos);
    }

    public void setGripperState(double pos) {
        gripper.setPosition(pos);
    }

    public void setSliderTarget(double sliderTicks) {
        targetSlider = sliderTicks;
    }



    public void specimenCycle()
    {
        switch(specStage) {
            case UNINITIALIZED:
                setAngleTarget(1200);
                setSliderTarget(0);
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

    public double getAngleMeasurement()
    {
        return enc.getCurrentPosition();
    }

    public void setAnglePower(double power)
    {
        MoveAndDestroy.setPower(power);
    }

    public double getSliderPosition()
    {
        return slider1.getCurrentPosition();
    }

}