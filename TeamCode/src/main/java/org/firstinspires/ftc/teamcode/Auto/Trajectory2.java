package org.firstinspires.ftc.teamcode.Auto;

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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;


@Config
@Autonomous (name = "AutonomieDreapta", group = "Autonomous")
public class Trajectory2 extends CommandOpMode {
    private Telemetry telemetryA;

    public static double RADIUS = 10;

    private Follower follower;

    private IntakeSubsystem intake;
    private OuttakeSubsystem outtake;
    public static double SliderPosition = 0.68;

    private PathBuilder circle;

    private PathBuilder circle2;

    private Pose startPose = new Pose(7.2, 59.88785046728972, Math.toRadians(180));

    /**
     * This initializes the Follower and creates the PathChain for the "circle". Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void initialize() {
        follower = new Follower(hardwareMap);

        intake = new IntakeSubsystem(hardwareMap);
        outtake = new OuttakeSubsystem(hardwareMap);

        follower.setStartingPose(startPose);

        follower.setMaxPower(0.7);

        schedule(
                new InstantCommand(intake::armInit),
                new WaitCommand(300),
                new InstantCommand(() -> outtake.setArmAngle(0.4)),
                new InstantCommand(() -> outtake.setArticulationAngle(0.6)),
                new InstantCommand(() -> outtake.setGripperState(outtake.GRIPPING)),
                new InstantCommand(intake::sliderShell),
                new RunCommand(follower::update),
                new RunCommand(outtake::updatePID),
                new SequentialCommandGroup(
            new WaitUntilCommand(this::opModeIsActive),
                        new FollowPathCommand(follower, new PathBuilder()
                                .addPath(
                                        // Line 1
                                        new BezierLine(
                                                new Point(7.2, 59.88785046728972, Point.CARTESIAN),
                                                new Point(31.08785046728972, 76.22429906542055, Point.CARTESIAN)
                                        )
                                )
                                .setPathEndTimeoutConstraint(200)
                                .setConstantHeadingInterpolation(Math.toRadians(180)).build())
                                .alongWith(
                                    new InstantCommand(() -> outtake.setTarget(-700)),
                                    new InstantCommand(() -> outtake.setArmAngle(outtake.ARM_PLACE_ELEMENT + 0.07)),
                                    new InstantCommand(() -> outtake.setArticulationAngle(outtake.ARTICULATION_ELEMENT+0.4))
                                ),
                        new InstantCommand(() -> outtake.setTarget(-260)),
                        new WaitCommand(270),
                        new InstantCommand(() -> outtake.setGripperState(outtake.NOT_GRIPPING)),
                        new InstantCommand(() -> outtake.setTarget(0)),
                        new InstantCommand(() -> {
                            outtake.setArmAngle(outtake.ARM_TRANSFER);
                            outtake.setArticulationAngle(outtake.ARTICULATION_TRANSFER);
                        }),
                        new FollowPathCommand(follower, new PathBuilder()
                                .addPath(
                                        // Line 1
                                        new BezierLine(
                                                new Point(31.08785046728972, 76.22429906542055, Point.CARTESIAN),
                                                new Point(35.888, 41.047, Point.CARTESIAN)
                                        )
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(180))
                                .addPath(
                                        // Line 2
                                        new BezierLine(
                                                new Point(35.888, 41.047, Point.CARTESIAN),
                                                new Point(47.729, 38, Point.CARTESIAN)
                                        )
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(180))
                                .addPath(
                                        // Line 3
                                        new BezierLine(
                                                new Point(47.729, 38, Point.CARTESIAN),
                                                new Point(54, 28, Point.CARTESIAN)
                                        )
                                )
                                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                                .addPath(
                                        // Line 4
                                        new BezierLine(
                                                new Point(54, 28, Point.CARTESIAN),
                                                new Point(15, 28, Point.CARTESIAN)
                                        )
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(90))
                                .addPath(
                                        // Line 3
                                        new BezierLine(
                                                new Point(15, 28, Point.CARTESIAN),
                                                new Point(54, 28, Point.CARTESIAN)
                                        )
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(90))
                                .addPath(
                                        // Line 3
                                        new BezierLine(
                                                new Point(54, 28, Point.CARTESIAN),
                                                new Point(54, 18, Point.CARTESIAN)
                                        )
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(90))
                                .addPath(
                                        // Line 3
                                        new BezierLine(
                                                new Point(54, 18, Point.CARTESIAN),
                                                new Point(15, 18, Point.CARTESIAN)
                                        )
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(90))
                                .addPath(
                                        // Line 3
                                        new BezierLine(
                                                new Point(15, 18, Point.CARTESIAN),
                                                new Point(54, 19, Point.CARTESIAN)
                                        )
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(90))
                                .addPath(
                                        // Line 3
                                        new BezierLine(
                                                new Point(15, 18, Point.CARTESIAN),
                                                new Point(54, 16.5, Point.CARTESIAN)
                                        )
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(90))
                                .build()
                        )
                )
        );

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */

}
