package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

public class DriveSubsystem extends SubsystemBase {

    private double powerLimit = 1.0;
    private final CachingDcMotorEx FL;
    private final CachingDcMotorEx FR;
    private final CachingDcMotorEx BL;
    private final CachingDcMotorEx BR;

    public DriveSubsystem(final HardwareMap hardwareMap) {
        this(hardwareMap, "FL", "FR",
                "BL", "BR");
    }

    private DriveSubsystem(HardwareMap hardwareMap, String leftFront, String rightFront, String leftBack, String rightBack) {

        FL = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, leftFront));
        FR = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, rightFront));
        BL = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, leftBack));
        BR = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, rightBack));

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void updateSpeeds(double y, double x, double rx, double botHeading) {
        // The equivalent button is start on Xbox-style controllers.
        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        FL.setPower(frontLeftPower * powerLimit);
        BL.setPower(backLeftPower * powerLimit);
        FR.setPower(frontRightPower * powerLimit);
        BR.setPower(backRightPower * powerLimit);
    }

    public void setPowerLimit(double limit) {
        powerLimit = limit;
    }
}