package org.firstinspires.ftc.teamcode.pedroPathing.tuners_tests.pid;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.teamcode.Subsystems.IOSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

/**
 * This is the StraightBackAndForth autonomous OpMode. It runs the robot in a specified distance
 * straight forward. On reaching the end of the forward Path, the robot runs the backward Path the
 * same distance back to the start. Rinse and repeat! This is good for testing a variety of Vectors,
 * like the drive Vector, the translational Vector, and the heading Vector. Remember to test your
 * tunings on CurvedBackAndForth as well, since tunings that work well for straight lines might
 * have issues going in curves.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
@Config
@Autonomous (name = "AutonomieDreapta   ", group = "PIDF Tuning")
public class AutonomieDreapta extends CommandOpMode {
    private Telemetry telemetryA;

    public static double DISTANCE = 40;

    private boolean forward = true;

    private Follower follower;

    private IOSubsystem IO;

    private Pose startPose = new Pose(7.2, 59, Math.toRadians(0));

    private PathChain highBar;
    private PathChain firstElement;

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void initialize() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);

        IO = new IOSubsystem(hardwareMap);

        follower.setStartingPose(startPose);

        follower.setMaxPower(0.8);


        highBar = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(startPose.getX(), startPose.getY(), Point.CARTESIAN),
                                new Point(31, 65, Point.CARTESIAN)
                        )
                )
                .setPathEndTimeoutConstraint(250)
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();

        firstElement = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(31, 65, Point.CARTESIAN),
                                new Point(23, 24, Point.CARTESIAN)
                        )
                )
                .setPathEndTimeoutConstraint(200)
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();


//        IO.setArmPosition(IO.ARM_INIT-0.35);


        schedule(
                new RunCommand(follower::update),
                new RunCommand(IO::updatePosition),
                new RunCommand(IO::updateAngle),
                new InstantCommand(IO::initDiffy),
                new InstantCommand(() -> IO.setArmPosition(IO.ARM_INIT-0.2)),
                new InstantCommand(() -> IO.setGripperState(IO.GRIPPING)),
                new SequentialCommandGroup(
                        new WaitUntilCommand(this::opModeIsActive),
                        new FollowPathCommand(follower, highBar)
                        .alongWith(
                            new SequentialCommandGroup(
                                    new WaitCommand(800),
                                    new InstantCommand(() -> IO.setAngleTarget(2100)),
                                    new InstantCommand(() -> IO.setArmPosition(IO.ARM_INIT)),
                                    new InstantCommand(() -> IO.setDiffyPitch(150))
                            )
                        ),
                        new InstantCommand(() -> IO.HoldPosition = 0.5),
                        new InstantCommand(() -> follower.setMaxPower(1)),
                        new InstantCommand(() -> IO.setSliderTarget(900)),
                        new WaitUntilCommand(() -> IO.getSliderPosition() >= 800),
                        new InstantCommand(() -> IO.setGripperState(IO.NOT_GRIPPING)),
                        new InstantCommand(() -> IO.HoldPosition = 0),
                        new WaitCommand(250),
                        new InstantCommand(() -> IO.setSliderTarget(0)),
                        new InstantCommand(() -> IO.setArmPosition(0.5)),
                        new WaitUntilCommand(() -> IO.getSliderPosition() <= 300),
                        new InstantCommand(() -> IO.setAngleTarget(0)),
                        new WaitCommand(450),
                        new InstantCommand(() -> IO.setArmPosition(IO.ARM_INIT)),
                        new InstantCommand(() -> IO.setDiffyPitch(180)),
                        new FollowPathCommand(follower, firstElement),
                        new InstantCommand(() -> IO.setSliderTarget(500)),
                        new InstantCommand(() -> IO.setDiffyPitch(0)),
                        new WaitUntilCommand(() -> IO.getSliderPosition() >= 400),
                        new WaitCommand(150),
                        new InstantCommand(() -> IO.setArmPosition(IO.LOADING_SAMPLE + 0.06)),
                        new WaitCommand(200),
                        new InstantCommand(() -> IO.setGripperState(IO.GRIPPING)),
                        new WaitCommand(200),
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, follower.pathBuilder()
                                        .addPath(
                                                new BezierLine(
                                                        new Point(23, 24, Point.CARTESIAN),
                                                        new Point(23, 25, Point.CARTESIAN)
                                                )
                                        )
                                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(15))
                                        .build()
                                ),
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> IO.setSliderTarget(0)),
                                        new InstantCommand(() -> IO.setAngleTarget(2100)),
                                        new InstantCommand(() -> IO.setArmPosition(IO.ARM_INIT - 0.2)),
                                        new InstantCommand(() -> IO.setDiffyPitch(120)),
                                        new WaitUntilCommand(() -> IO.getAngleMeasurement() >= 1950),
                                        new WaitCommand(150),
                                        new InstantCommand(() -> IO.setGripperState(IO.NOT_GRIPPING))
                                )
                        ),
                        new WaitCommand(250),
                        new InstantCommand(() -> IO.setAngleTarget(0)),
                        new WaitUntilCommand(() -> IO.getAngleMeasurement() <= 50),
                        new FollowPathCommand(follower, follower.pathBuilder()
                                .addPath(
                                        new BezierLine(
                                                new Point(23, 25, Point.CARTESIAN),
                                                new Point(23, 24, Point.CARTESIAN)
                                        )
                                )
                                .setLinearHeadingInterpolation(Math.toRadians(15), Math.toRadians(-25))
                                .build()
                        ),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> IO.setArmPosition(IO.ARM_INIT)),
                                new InstantCommand(() -> IO.setSliderTarget(750)),
                                new InstantCommand(() -> IO.setDiffyPitch(0)),
                                new InstantCommand(() -> IO.setDiffyYaw(45)),
                                new WaitUntilCommand(() -> IO.getSliderPosition() >= 650),
                                new WaitCommand(500),
                                new InstantCommand(() -> IO.setArmPosition(IO.LOADING_SAMPLE + 0.06)),
                                new WaitCommand(200),
                                new InstantCommand(() -> IO.setGripperState(IO.GRIPPING)),
                                new WaitCommand(200)

                        ),
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, follower.pathBuilder()
                                        .addPath(
                                                new BezierLine(
                                                        new Point(23, 24, Point.CARTESIAN),
                                                        new Point(23, 25, Point.CARTESIAN)
                                                )
                                        )
                                        .setLinearHeadingInterpolation(Math.toRadians(-25), Math.toRadians(15))
                                        .build()
                                ),
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> IO.setSliderTarget(0)),
                                        new InstantCommand(() -> IO.setAngleTarget(2100)),
                                        new InstantCommand(() -> IO.setArmPosition(IO.ARM_INIT - 0.2)),
                                        new InstantCommand(() -> IO.setDiffyYaw(90)),
                                        new InstantCommand(() -> IO.setDiffyPitch(120)),
                                        new WaitUntilCommand(() -> IO.getAngleMeasurement() >= 1950),
                                        new WaitCommand(150),
                                        new InstantCommand(() -> IO.setGripperState(IO.NOT_GRIPPING))
                                )
                        ),
                        new WaitCommand(250),
                        new InstantCommand(() -> IO.setAngleTarget(0)),
                        new WaitUntilCommand(() -> IO.getAngleMeasurement() <= 50),
                        new FollowPathCommand(follower, follower.pathBuilder()
                                .addPath(
                                        new BezierLine(
                                                new Point(23, 25, Point.CARTESIAN),
                                                new Point(23, 24, Point.CARTESIAN)
                                        )
                                )
                                .setLinearHeadingInterpolation(Math.toRadians(15), Math.toRadians(-40))
                                .build()
                        ),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> IO.setArmPosition(IO.ARM_INIT)),
                                new InstantCommand(() -> IO.setSliderTarget(1100)),
                                new InstantCommand(() -> IO.setDiffyPitch(0)),
                                new InstantCommand(() -> IO.setDiffyYaw(20)),
                                new WaitUntilCommand(() -> IO.getSliderPosition() >= 1000),
                                new WaitCommand(500),
                                new InstantCommand(() -> IO.setArmPosition(IO.LOADING_SAMPLE + 0.06)),
                                new WaitCommand(200),
                                new InstantCommand(() -> IO.setGripperState(IO.GRIPPING)),
                                new WaitCommand(200)

                        ),
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, follower.pathBuilder()
                                        .addPath(
                                                new BezierLine(
                                                        new Point(23, 25, Point.CARTESIAN),
                                                        new Point(23, 24, Point.CARTESIAN)
                                                )
                                        )
                                        .setLinearHeadingInterpolation(Math.toRadians(-40), Math.toRadians(15))
                                        .build()
                                ),
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> IO.setSliderTarget(0)),
                                        new InstantCommand(() -> IO.setAngleTarget(2100)),
                                        new InstantCommand(() -> IO.setArmPosition(IO.ARM_INIT - 0.2)),
                                        new InstantCommand(() -> IO.setDiffyYaw(90)),
                                        new InstantCommand(() -> IO.setDiffyPitch(120)),
                                        new WaitUntilCommand(() -> IO.getAngleMeasurement() >= 1950),
                                        new WaitCommand(150),
                                        new InstantCommand(() -> IO.setGripperState(IO.NOT_GRIPPING))
                                )
                        )



                )
        );


        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
}
