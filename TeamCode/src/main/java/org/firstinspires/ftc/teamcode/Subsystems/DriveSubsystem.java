package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

public class DriveSubsystem extends SubsystemBase {

    private double powerLimit = 1.0;
    private final MecanumDrive drive;
    private DoubleSupplier forward, strafe, rotation;
    private Double gyro;

    public DriveSubsystem(final HardwareMap hardwareMap) {
        this(hardwareMap, "FL", "FR",
                "BL", "BR");
    }

    @Override
    public void periodic() {
        if (Arrays.asList(forward, strafe, rotation, gyro).contains(null))
            return;

        updateSpeeds(forward.getAsDouble(), strafe.getAsDouble(), rotation.getAsDouble(), gyro);
    }

    private DriveSubsystem(HardwareMap hardwareMap, String leftFront, String rightFront, String leftBack, String rightBack) {

        Motor FL = new Motor(hardwareMap, leftFront, Motor.GoBILDA.RPM_435);
        Motor FR = new Motor(hardwareMap, rightFront, Motor.GoBILDA.RPM_435);
        Motor BL = new Motor(hardwareMap, leftBack, Motor.GoBILDA.RPM_435);
        Motor BR = new Motor(hardwareMap, rightBack, Motor.GoBILDA.RPM_435);

        FL.encoder.reset();
        FR.encoder.reset();
        BL.encoder.reset();
        BR.encoder.reset();


        FR.setRunMode(Motor.RunMode.VelocityControl);
        FL.setRunMode(Motor.RunMode.VelocityControl);
        BR.setRunMode(Motor.RunMode.VelocityControl);
        BL.setRunMode(Motor.RunMode.VelocityControl);

        FL.setInverted(true);
        BL.setInverted(true);


        FR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        drive = new MecanumDrive(
                FL,
                FR,
                BL,
                BR
        );

    }

    public void setAxes(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotation, Double gyro) {
        this.forward = forward;
        this.strafe = strafe;
        this.rotation = rotation;
        this.gyro = gyro;
    }

    public void updateSpeeds(double fwd, double st, double rot, double gyro) {
        drive.driveFieldCentric(st, fwd, rot, gyro);
    }

    public void setPowerLimit(double limit) {
        powerLimit = MathUtils.clamp(Math.abs(limit), 0, 1);
        drive.setMaxSpeed(powerLimit);
    }
}