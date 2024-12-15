package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.commands.FollowPathCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Vision.SolvePerspective;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@Config
@Autonomous (name = "AutonomieStanga", group = "Autonomous")
public class Trajectory extends CommandOpMode {
    private Telemetry telemetryA;

    public static double RADIUS = 10;

    private Follower follower;

    private IntakeSubsystem intake;
    private OuttakeSubsystem outtake;
    public static double SliderPosition = 0.68;

    private PathBuilder circle;

    private PathBuilder circle2;

    private Pose startPose = new Pose(7.2, 83.983, Math.toRadians(180));


    private Point highBarPos = new Point(30.08785046728972, 76.22429906542055, Point.CARTESIAN);

    private Point firstElementPos = new Point(22.206, 120, Point.CARTESIAN);

    private Point secondElementPos =  new Point(22.206, 130.6, Point.CARTESIAN);

    private Point thirdElementPos = new Point(43.5, 122.7, Point.CARTESIAN);

    private Point basket = new Point(14.552, 126.578, Point.CARTESIAN);

    private PathChain highBar;

    private PathChain firstElement;

    private PathChain firstToBasket;

    private PathChain basketToSecond;

    private PathChain secondToBasket;

    private PathChain basketToThird;

    private PathChain thirdToBasket;

    /**
     * This initializes the Follower and creates the PathChain for the "circle". Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void initialize() {

        highBar = new PathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(startPose.getX(), startPose.getY(), Point.CARTESIAN),
                                highBarPos
                        )
                )
                .setPathEndTimeoutConstraint(200)
                .setConstantHeadingInterpolation(Math.toRadians(180)).build();

        firstElement = new PathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                highBarPos,
                                firstElementPos
                        )
                )
                .setPathEndTimeoutConstraint(1000)
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-1)).build();


        firstToBasket = new PathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                firstElementPos,
                                basket
                        )
                )
                .setPathEndTimeoutConstraint(400)
                .setLinearHeadingInterpolation(Math.toRadians(-1), Math.toRadians(-45)).build();


        basketToSecond = new PathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                basket,
                                secondElementPos
                        )
                )
                .setPathEndTimeoutConstraint(400)
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-1)).build();

        secondToBasket = new PathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                secondElementPos,
                                basket
                        )
                )
                .setPathEndTimeoutConstraint(1000)
                .setLinearHeadingInterpolation(Math.toRadians(-1), Math.toRadians(-45)).build();


        basketToThird = new PathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                basket,
                                thirdElementPos
                        )
                )
                .setPathEndTimeoutConstraint(600)
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(91)).build();

        thirdToBasket = new PathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                thirdElementPos,
                                basket
                        )
                )
                .setPathEndTimeoutConstraint(600)
                .setLinearHeadingInterpolation(Math.toRadians(91), Math.toRadians(-45)).build();




        follower = new Follower(hardwareMap);

        intake = new IntakeSubsystem(hardwareMap);
        outtake = new OuttakeSubsystem(hardwareMap);

        follower.setMaxPower(0.7);

        follower.setStartingPose(startPose);


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
                        new FollowPathCommand(follower, highBar)
                                .alongWith(
                                    new InstantCommand(() -> outtake.setTarget(-750)),
                                    new InstantCommand(() -> outtake.setArmAngle(outtake.ARM_PLACE_ELEMENT + 0.12)),
                                    new InstantCommand(() -> outtake.setArticulationAngle(outtake.ARTICULATION_ELEMENT+0.5))
                                ),
                        new InstantCommand(() -> outtake.setTarget(-260)),
                        new WaitCommand(270),
                        new InstantCommand(() -> outtake.setGripperState(outtake.NOT_GRIPPING)),
                        new InstantCommand(() -> outtake.setTarget(0)),
                        new InstantCommand(() -> {
                            outtake.setArmAngle(outtake.ARM_TRANSFER);
                            outtake.setArticulationAngle(outtake.ARTICULATION_TRANSFER);
                        }),
                        new FollowPathCommand(follower, firstElement),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> intake.readySetPosition(0.665)),
                                new WaitCommand(300),
                                new InstantCommand(() ->  intake.ArmAngle(intake.ARM_READY_POS+0.07)),
                                new WaitCommand(600),
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> intake.setPitch(intake.PITCH_GRIP + 0.05)),
                                        new InstantCommand(() -> intake.gripState(intake.NOT_GRIPPING)),
                                        new WaitCommand(100),
                                        new InstantCommand(intake::armToElement),
                                        new WaitCommand(150),
                                        new InstantCommand(() -> intake.gripState(intake.GRIPPING)),
                                        new WaitCommand(250),
                                        new InstantCommand(intake::armToReady)
                                ),
                                new WaitCommand(450),
                                new InstantCommand(() -> {
                                    intake.ArmAngle(intake.ARM_TRANSFER_POS);
                                    intake.setPitch(intake.PITCH_TRANSFER);
                                }),
                                new InstantCommand(intake::transferPosition),
                                new WaitCommand(450),
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> {
                                            intake.gripState(intake.NOT_GRIPPING);
                                            outtake.setGripperState(outtake.GRIPPING);
                                        }),
                                        new WaitCommand(150),
                                        new InstantCommand(() -> intake.setSliderPosition(intake.SLIDER_SHELLED + 0.15)),
                                        new WaitCommand(250),
                                        new InstantCommand(() -> outtake.readySetPosition(0.15))
                                )
                        ),
                        new FollowPathCommand(follower, firstToBasket)
                        .alongWith(
                            new SequentialCommandGroup(
                                    new InstantCommand(() -> outtake.setTarget(-1350)),
                                    new WaitUntilCommand(() -> outtake.getPosition() <= -1300)
                                            .andThen(
                                                    new InstantCommand(() -> outtake.setArmAngle(outtake.ARM_PLACE_ELEMENT+0.09)),
                                                    new WaitCommand(100),
                                                    new InstantCommand(() -> outtake.setGripperState(outtake.NOT_GRIPPING)),
                                                    new WaitCommand(250)
                                            ),
                                    new InstantCommand(() -> {
                                        outtake.setArmAngle(outtake.ARM_TRANSFER);
                                        outtake.setArticulationAngle(outtake.ARTICULATION_TRANSFER);
                                    }),
                                    new WaitCommand(250),
                                    new InstantCommand(() -> outtake.setTarget(2))
                            )
                        ),
                        new WaitCommand(800),
                        new FollowPathCommand(follower, basketToSecond),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> intake.readySetPosition(0.675)),
                                new WaitCommand(300),
                                new InstantCommand(() ->  intake.ArmAngle(intake.ARM_READY_POS+0.07)),
                                new WaitCommand(600),
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> intake.setPitch(intake.PITCH_GRIP + 0.05)),
                                        new InstantCommand(() -> intake.gripState(intake.NOT_GRIPPING)),
                                        new WaitCommand(100),
                                        new InstantCommand(intake::armToElement),
                                        new WaitCommand(150),
                                        new InstantCommand(() -> intake.gripState(intake.GRIPPING)),
                                        new WaitCommand(250),
                                        new InstantCommand(intake::armToReady)
                                ),
                                new WaitCommand(450),
                                new InstantCommand(() -> {
                                    intake.ArmAngle(intake.ARM_TRANSFER_POS);
                                    intake.setPitch(intake.PITCH_TRANSFER);
                                }),
                                new InstantCommand(intake::transferPosition),
                                new WaitCommand(450),
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> {
                                            intake.gripState(intake.NOT_GRIPPING);
                                            outtake.setGripperState(outtake.GRIPPING);
                                        }),
                                        new WaitCommand(150),
                                        new InstantCommand(() -> intake.setSliderPosition(intake.SLIDER_SHELLED + 0.15)),
                                        new WaitCommand(250),
                                        new InstantCommand(() -> outtake.readySetPosition(0.15))
                                )
                        ),
                        new FollowPathCommand(follower, secondToBasket)
                        .alongWith(
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> outtake.setTarget(-1350)),
                                        new WaitUntilCommand(() -> outtake.getPosition() <= -1330)
                                                .andThen(
                                                        new InstantCommand(() -> outtake.setArmAngle(outtake.ARM_PLACE_ELEMENT+0.09)),
                                                        new WaitCommand(100),
                                                        new InstantCommand(() -> outtake.setGripperState(outtake.NOT_GRIPPING)),
                                                        new WaitCommand(100)
                                                ),
                                        new InstantCommand(() -> {
                                            outtake.setArmAngle(outtake.ARM_TRANSFER);
                                            outtake.setArticulationAngle(outtake.ARTICULATION_TRANSFER);
                                        }),
                                        new WaitCommand(300),
                                        new InstantCommand(() -> outtake.setTarget(2))
                                )
                        ),
                        new WaitCommand(800),
                        new FollowPathCommand(follower, basketToThird),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> intake.readySetPosition(0.55)),
                                new WaitCommand(300),
                                new InstantCommand(() ->  intake.ArmAngle(intake.ARM_READY_POS+0.07)),
                                new InstantCommand(() -> intake.setYaw(intake.YAW_180)),
                                new WaitCommand(600),
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> {
                                            intake.setPitch(intake.PITCH_GRIP + 0.05);
                                            intake.gripState(intake.NOT_GRIPPING);
                                        }),
                                        new WaitCommand(100),
                                        new InstantCommand(intake::armToElement),
                                        new WaitCommand(150),
                                        new InstantCommand(() -> intake.gripState(intake.GRIPPING)),
                                        new WaitCommand(250),
                                        new InstantCommand(intake::armToReady)
                                ),
                                new WaitCommand(450),
                                new InstantCommand(intake::transferPosition),
                                new WaitCommand(200),
                                new InstantCommand(() -> {
                                    intake.ArmAngle(intake.ARM_TRANSFER_POS);
                                    intake.setPitch(intake.PITCH_TRANSFER);
                                }),
                                new WaitCommand(450),
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> {
                                            intake.gripState(intake.NOT_GRIPPING);
                                            outtake.setGripperState(outtake.GRIPPING);
                                        }),
                                        new WaitCommand(150),
                                        new InstantCommand(() -> intake.setSliderPosition(intake.SLIDER_SHELLED + 0.15)),
                                        new WaitCommand(250),
                                        new InstantCommand(() -> outtake.readySetPosition(0.15))
                                )
                        ),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> outtake.setTarget(-1350)),
                                        new WaitCommand(1600),
                                        new WaitUntilCommand(() -> outtake.getPosition() <= -1330)
                                                .andThen(
                                                        new InstantCommand(() -> outtake.setArmAngle(outtake.ARM_PLACE_ELEMENT+0.09)),
                                                        new WaitCommand(100),
                                                        new InstantCommand(() -> outtake.setGripperState(outtake.NOT_GRIPPING)),
                                                        new WaitCommand(100)
                                                ),
                                        new InstantCommand(() -> {
                                            outtake.setArmAngle(outtake.ARM_TRANSFER);
                                            outtake.setArticulationAngle(outtake.ARTICULATION_TRANSFER);
                                        }),
                                        new WaitCommand(300),
                                        new InstantCommand(() -> outtake.setTarget(2))
                                ),
                                new FollowPathCommand(follower, thirdToBasket)
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
