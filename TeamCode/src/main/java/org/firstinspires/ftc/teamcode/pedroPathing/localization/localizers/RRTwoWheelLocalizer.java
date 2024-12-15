package org.firstinspires.ftc.teamcode.pedroPathing.localization.localizers;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Localizer;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;

import java.lang.Math;

@Config
public class RRTwoWheelLocalizer extends Localizer {

    public static class Params {
        public double parYTicks = 1700.223980254964; // y position of the parallel encoder (in tick units)
        public double perpXTicks = 2407.2370241499116; // x position of the perpendicular encoder (in tick units)
    }

    public static Params PARAMS = new Params();

    public final Encoder par, perp;
    public final IMU imu;

    private int lastParPos, lastPerpPos;
    private Rotation2d lastHeading;

    private final double inPerTick = 0.00295256;

    private double lastRawHeadingVel, headingVelOffset;
    private boolean initialized;

    public Pose2d pose = new Pose2d(0,0,0);


    public RRTwoWheelLocalizer(HardwareMap hardwareMap) {
        // TODO: make sure your config has **motors** with these names (or change them)
        //   the encoders should be plugged into the slot matching the named motor
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        par = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "par")));
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "perp")));

        // TODO: reverse encoder directions if needed
        //   par.setDirection(DcMotorSimple.Direction.REVERSE);
        // perp.setDirection(DcMotorSimple.Direction.REVERSE);
        LazyImu lazyImu;
        lazyImu = new LazyImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu = lazyImu.get();
        imu.resetYaw();

        lastParPos = par.getPositionAndVelocity().position;
        lastPerpPos = perp.getPositionAndVelocity().position;

    }

    public void update() {
        ElapsedTime loopTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        PositionVelocityPair parPosVel = par.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS);

        // FlightRecorder.write("TWO_DEAD_WHEEL_INPUTS", new TwoDeadWheelInputsMessage(parPosVel, perpPosVel, angles, angularVelocity));

        Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));

        // see https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/617
        double rawHeadingVel = angularVelocity.zRotationRate;
        if (Math.abs(rawHeadingVel - lastRawHeadingVel) > Math.PI) {
            headingVelOffset -= Math.signum(rawHeadingVel) * 2 * Math.PI;
        }
        lastRawHeadingVel = rawHeadingVel;
        double headingVel = headingVelOffset + rawHeadingVel;

        if (!initialized) {
            initialized = true;

            lastParPos = parPosVel.position;
            lastPerpPos = perpPosVel.position;
            lastHeading = heading;

//            return new Twist2dDual<>(
//                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
//                    DualNum.constant(0.0, 2)
//            );
        }

        int parPosDelta = parPosVel.position - lastParPos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;
        double headingDelta = heading.minus(lastHeading);

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                parPosDelta - PARAMS.parYTicks * headingDelta,
                                parPosVel.velocity - PARAMS.parYTicks * headingVel,
                        }).times(inPerTick),
                        new DualNum<Time>(new double[] {
                                perpPosDelta - PARAMS.perpXTicks * headingDelta,
                                perpPosVel.velocity - PARAMS.perpXTicks * headingVel,
                        }).times(inPerTick)
                ),
                new DualNum<>(new double[] {
                        headingDelta,
                        headingVel,
                })
        );

        lastParPos = parPosVel.position;
        lastPerpPos = perpPosVel.position;
        lastHeading = heading;

        pose = pose.plus(twist.value());
        loopTimer.reset();
    }


    public void setPoseEstimate(Pose2d poseEstimate) {
        this.pose = poseEstimate;
    }

    public Pose2d getPoseEstimate() {
        return this.pose;
    }

    public void resetHeading(double newHeading) {
        this.pose = new Pose2d(this.pose.position.x, this.pose.position.y, newHeading);
    }

    @Override
    public Pose getPose() {
        return new Pose(pose.position.x, pose.position.y, pose.heading.toDouble());
    }

    @Override
    public Pose getVelocity() {
        return null;
    }

    @Override
    public Vector getVelocityVector() {
        return null;
    }

    @Override
    public void setStartPose(Pose setStart) {
        pose =  new Pose2d(setStart.getX(), setStart.getY(), setStart.getHeading());
    }

    @Override
    public void setPose(Pose setPose) {
        pose =  new Pose2d(setPose.getX(), setPose.getY(), setPose.getHeading());
    }
    @Override
    public double getTotalHeading() {
        return pose.heading.toDouble();
    }

    @Override
    public double getForwardMultiplier() {
        return 0;
    }

    @Override
    public double getLateralMultiplier() {
        return 0;
    }

    @Override
    public double getTurningMultiplier() {
        return 0;
    }

    @Override
    public void resetIMU() {
        imu.resetYaw();
    }
}