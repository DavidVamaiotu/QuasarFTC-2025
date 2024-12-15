package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

public class OuttakeSubsystem extends SubsystemBase {
    private final CachingDcMotorEx slider1;

    private final CachingServo arm2;

    private final CachingServo articulation;

    private final CachingServo gripper;

//    private final CachingServo sldServo;

    private double kP = 0.03;
    private double kI = 0;
    private double kD = 0.0005;

    public double SLIDER_EXTENDED = 1320;


    public enum OUTTAKE_STAGE {
        READY,
        SHELLED
    }

    public OuttakeSubsystem.OUTTAKE_STAGE stageo = OUTTAKE_STAGE.SHELLED;



    public double GRIPPING = 0.3;
    public double NOT_GRIPPING = 0;

    public double ARTICULATION_WALL = 0.5;

    public double ARM_WALL = 0.92;

    public double ARTICULATION_TRANSFER = 0.6;

    public double ARTICULATION_ELEMENT = 0.64;
    public double ARM_TRANSFER = 0.24;
    public double ARM_PLACE_ELEMENT = 0.65;
    public double ARM_INIT = 0.53;
    public double SLIDER_SHELLED = 5;

    PIDController pid;

    double target = 5;

    public OuttakeSubsystem(final HardwareMap hMap) {
        pid = new PIDController(kP, kI, kD);
        slider1 = new CachingDcMotorEx(hMap.get(DcMotorEx.class, "sld1"));
        arm2 = new CachingServo(hMap.get(Servo.class, "arm2"));
        gripper = new CachingServo(hMap.get(Servo.class, "grip2"));
        articulation = new CachingServo(hMap.get(Servo.class, "artic"));

        slider1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slider1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm2.setDirection(Servo.Direction.FORWARD);
//        slider1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        slider1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        sldServo = new CachingServo(hMap.get(Servo.class, "srv2"));
    }

    public void setPower(double power) {
         slider1.setPower(power);
    }

    public void extendSlider() {
        setTarget(SLIDER_EXTENDED);
    }


    public void readyPosition()
    {
        extendSlider();
        arm2.setPosition(ARM_PLACE_ELEMENT);
        articulation.setPosition(ARTICULATION_ELEMENT);
        stageo = OUTTAKE_STAGE.READY;
    }


    public void readySetPosition(double offset)
    {
        arm2.setPosition(ARM_PLACE_ELEMENT - offset);
        articulation.setPosition(ARTICULATION_ELEMENT);
        stageo = OUTTAKE_STAGE.READY;
    }


    public void transferPosition()
    {
        arm2.setPosition(ARM_TRANSFER);
        gripper.setPosition(NOT_GRIPPING);
        articulation.setPosition(ARTICULATION_TRANSFER);
        shellSlider();
        stageo = OUTTAKE_STAGE.SHELLED;
    }


    public void toPosition()
    {
       switch(stageo)
       {
           case SHELLED:
               readyPosition();
               break;
           case READY:
               transferPosition();
               break;
           default:
               throw new IllegalStateException("Unexpected value: " + stageo);
       }
    }

    public void setArmAngle(double pos) {
        arm2.setPosition(pos);
    }

    public void armInit() {
        setArmAngle(ARM_TRANSFER);
        setArticulationAngle(ARTICULATION_TRANSFER);
        stageo = OUTTAKE_STAGE.SHELLED;
    }

    public void setGripperState(double pos) {
        gripper.setPosition(pos);
    }

    public void setArticulationAngle(double pos) {
        articulation.setPosition(pos);
    }

    public void shellSlider() {
        setTarget(SLIDER_SHELLED);
    }

    public void updatePID() {
        double output = pid.calculate(
                slider1.getCurrentPosition(), target
        );

        slider1.setPower(output);
    }

    public void setTarget(double targetsa) {
        target = targetsa;
    }

//    public void setServoPosition(double position) {
//        sldServo.setPosition(position);
//    }

    public double getPosition()
    {
        return slider1.getCurrentPosition();
    }

}