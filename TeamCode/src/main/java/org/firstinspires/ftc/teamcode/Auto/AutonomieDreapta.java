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
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.GoBildaPinpointDriver;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
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
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
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
@Autonomous (name = "AutonomieDreapta", group = "Auto")
public class AutonomieDreapta extends CommandOpMode {
    private Telemetry telemetryA;

    public static double DISTANCE = 40;

    private boolean forward = true;

    private Follower follower;

    private IOSubsystem IO;

    private Pose startPose = new Pose(7.1, 59, Math.toRadians(180));

    private GoBildaPinpointDriver odo;

    private VoltageSensor vSensor;

    private PathChain TohighBar;
    private PathChain TofirstElement;

    private PathChain FirstElementPlace;

    private PathChain SecondElementPlace;

    private PathChain ThirdElementPlace;

    private PathChain ToSecondElement;

    private PathChain ToThirdElement;

    private PathChain ToSpecimenIntake;

    private PathChain IntakeSpecimen;

    private PathChain CycleToBar;

    private PathChain CycleToBar2;

    private PathChain CycleToBar3;

    private PathChain CycleToBar4;

    private PathChain CycleToSpecimen;

    private PathChain CycleToSpecimen2;

    private PathChain CycleToSpecimen3;

    private Point highBar = new Point(32.75, 63.5, Point.CARTESIAN);

    private Point specimenIntake = new Point(9.44953, 33.21495327102804, Point.CARTESIAN);

    private Point firstElement = new Point(23, 23, Point.CARTESIAN);


    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void initialize() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);

        IO = new IOSubsystem(hardwareMap);

//        vSensor = hardwareMap.voltageSensor.iterator().next();

        IO.resetEncoder();

        follower.setStartingPose(startPose);
//
//        follower.setMaxPower(1 * (12.0 / vSensor.getVoltage()));
//
//        follower.setStartingPose(startPose);

//        follower.setMaxPower(0.8);

        // pozitie highBar

        TohighBar = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(startPose.getX(), startPose.getY(), Point.CARTESIAN),
                                new Point(highBar.getX() - 0.5, highBar.getY(), Point.CARTESIAN)
                        )
                )
                .setPathEndTimeoutConstraint(100)
                .setPathEndTValueConstraint(0.97)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();


        // pozitia primului element

        TofirstElement = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(highBar.getX() - 0.5, highBar.getY(), Point.CARTESIAN),
                                firstElement
                        )
                )
                .setPathEndTimeoutConstraint(100) // cat timp sta sa se corecteze robotul
                .setPathEndTValueConstraint(0.97)
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0)) // unghiul cu care porneste si cel cu care termina
                .build();

        ToSecondElement = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(firstElement.getX(), firstElement.getY()+1, Point.CARTESIAN),
                                firstElement
                        )
                )
                .setPathEndTimeoutConstraint(100) // cat timp sta sa se corecteze robotul
                .setLinearHeadingInterpolation(Math.toRadians(15), Math.toRadians(-29)) // unghiul cu care porneste si cel cu care termina
                .build();


        ToThirdElement = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(firstElement.getX(), firstElement.getY()+1, Point.CARTESIAN),
                                firstElement
                        )
                )
                .setPathEndTimeoutConstraint(100) // cat timp sta sa se corecteze robotul
                .setLinearHeadingInterpolation(Math.toRadians(15), Math.toRadians(-48)) // unghiul cu care porneste si cel cu care termina
                .build();

        FirstElementPlace = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                firstElement,
                                new Point(firstElement.getX(), firstElement.getY()+1, Point.CARTESIAN)
                        )
                )
                .setPathEndTimeoutConstraint(100) // cat timp sta sa se corecteze robotul
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(15)) // unghiul cu care porneste si cel cu care termina
                .build();

        SecondElementPlace = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                firstElement,
                                new Point(firstElement.getX(), firstElement.getY()+1, Point.CARTESIAN)
                        )
                )
                .setPathEndTimeoutConstraint(100) // cat timp sta sa se corecteze robotul
                .setLinearHeadingInterpolation(Math.toRadians(-29), Math.toRadians(15)) // unghiul cu care porneste si cel cu care termina
                .build();

        ThirdElementPlace = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                firstElement,
                                new Point(firstElement.getX(), firstElement.getY()+1, Point.CARTESIAN)
                        )
                )
                .setPathEndTimeoutConstraint(100) // cat timp sta sa se corecteze robotul
                .setLinearHeadingInterpolation(Math.toRadians(-48), Math.toRadians(15)) // unghiul cu care porneste si cel cu care termina
                .build();

        ToSpecimenIntake = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(firstElement.getX(), firstElement.getY()+1, Point.CARTESIAN),
                                specimenIntake
                        )
                )
                .setPathEndTimeoutConstraint(100)
                .setPathEndTValueConstraint(0.97)// cat timp sta sa se corecteze robotul
                .setLinearHeadingInterpolation(Math.toRadians(15), Math.toRadians(180)) // unghiul cu care porneste si cel cu care termina
                .build();

        IntakeSpecimen = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(firstElement.getX(), firstElement.getY()+1, Point.CARTESIAN),
                                new Point(24.673, 35.215, Point.CARTESIAN),
                                new Point(specimenIntake.getX() - 0.45, specimenIntake.getY()+0.5, Point.CARTESIAN)
                        )
                )
                .setPathEndTimeoutConstraint(100)
                .setPathEndTValueConstraint(0.95)// cat timp sta sa se corecteze robotul
                .setConstantHeadingInterpolation(Math.toRadians(180)) // unghiul cu care porneste si cel cu care termina
                .build();

        CycleToBar = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(specimenIntake.getX() - 0.2, specimenIntake.getY(), Point.CARTESIAN),
                                new Point(22.36448598130841, 53.159, Point.CARTESIAN),
                                new Point(highBar.getX() + 0.2, highBar.getY() + 2, Point.CARTESIAN)
                        )
                )
                .setPathEndTimeoutConstraint(100)
                .setPathEndTValueConstraint(0.97)// cat timp sta sa se corecteze robotul
                .setConstantHeadingInterpolation(Math.toRadians(180)) // unghiul cu care porneste si cel cu care termina
                .build();

        CycleToSpecimen = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(highBar.getX() + 0.2, highBar.getY() + 2, Point.CARTESIAN),
                                new Point(22.36448598130841, 41.7196261682243, Point.CARTESIAN),
                                new Point(specimenIntake.getX(), specimenIntake.getY()-0.8, Point.CARTESIAN)
                        )
                )
                .setPathEndTimeoutConstraint(100)
                .setPathEndTValueConstraint(0.95)// cat timp sta sa se corecteze robotul
                .setConstantHeadingInterpolation(Math.toRadians(180)) // unghiul cu care porneste si cel cu care termina
                .build();

        CycleToSpecimen2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(highBar.getX(), highBar.getY() + 4, Point.CARTESIAN),
                                new Point(22.36448598130841, 41.7196261682243, Point.CARTESIAN),
                                new Point(specimenIntake.getX(), specimenIntake.getY(), Point.CARTESIAN)
                        )
                )
                .setPathEndTimeoutConstraint(100)
                .setPathEndTValueConstraint(0.95)// cat timp sta sa se corecteze robotul
                .setConstantHeadingInterpolation(Math.toRadians(180)) // unghiul cu care porneste si cel cu care termina
                .build();

        CycleToBar2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(specimenIntake.getX(), specimenIntake.getY(), Point.CARTESIAN),
                                new Point(22.36448598130841, 53.159, Point.CARTESIAN),
                                new Point(highBar.getX(), highBar.getY() + 4, Point.CARTESIAN)
                        )
                )
                .setPathEndTimeoutConstraint(100)
                .setPathEndTValueConstraint(0.97)// cat timp sta sa se corecteze robotul
                .setConstantHeadingInterpolation(Math.toRadians(180)) // unghiul cu care porneste si cel cu care termina
                .build();

        CycleToBar3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(specimenIntake.getX(), specimenIntake.getY(), Point.CARTESIAN),
                                new Point(22.36448598130841, 53.159, Point.CARTESIAN),
                                new Point(highBar.getX(), highBar.getY() + 6, Point.CARTESIAN)
                        )
                )
                .setPathEndTimeoutConstraint(100)
                .setPathEndTValueConstraint(0.97)// cat timp sta sa se corecteze robotul
                .setConstantHeadingInterpolation(Math.toRadians(180)) // unghiul cu care porneste si cel cu care termina
                .build();

        CycleToSpecimen3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(highBar.getX(), highBar.getY() + 6, Point.CARTESIAN),
                                new Point(22.36448598130841, 41.7196261682243, Point.CARTESIAN),
                                new Point(specimenIntake.getX(), specimenIntake.getY(), Point.CARTESIAN)
                        )
                )
                .setPathEndTimeoutConstraint(100)
                .setPathEndTValueConstraint(0.95)// cat timp sta sa se corecteze robotul
                .setConstantHeadingInterpolation(Math.toRadians(180)) // unghiul cu care porneste si cel cu care termina
                .build();

        CycleToBar4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(specimenIntake.getX(), specimenIntake.getY(), Point.CARTESIAN),
                                new Point(22.66448598130841, 53.159, Point.CARTESIAN),
                                new Point(highBar.getX() + 0.2, highBar.getY() + 8, Point.CARTESIAN)
                        )
                )
                .setPathEndTimeoutConstraint(50)
                .setPathEndTValueConstraint(0.97)// cat timp sta sa se corecteze robotul
                .setConstantHeadingInterpolation(Math.toRadians(180)) // unghiul cu care porneste si cel cu care termina
                .build();


//        IO.setArmPosition(IO.ARM_INIT-0.35);


        // SLIDERUL MERGE DE LA 0 LA 1850 tick-uri
        // UNGHIUL SLIDERULUI MERGE DE LA 0 la 2100 de tick-uri
        // DIFFYPITCH se refera la miscarea sus jos a gripper-ului
        // DIFFYYAW se refera la rotatia gripper-ului


        schedule(
                new RunCommand(IO::updatePosition),
                new RunCommand(IO::updateAngle),
                new RunCommand(follower::update),
                new InstantCommand(IO::initDiffy),
                new InstantCommand(() -> IO.setDiffyPitch(110)),
                new InstantCommand(() -> IO.setArmPosition(IO.ARM_INIT+0.06)),
                new InstantCommand(() -> IO.setGripperState(IO.GRIPPING)),
                new SequentialCommandGroup(
                        new WaitUntilCommand(this::opModeIsActive),
                        new FollowPathCommand(follower, TohighBar)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new WaitCommand(200),
                                                new InstantCommand(() -> IO.setAngleTarget(2100)),
                                                new InstantCommand(() -> IO.setArmPosition(IO.PLACE_SPECIMEN)),
                                                new InstantCommand(() -> IO.setDiffyPitch(175)),
                                                new WaitUntilCommand(() -> IO.getAngleMeasurement() >= 1700),
                                                new InstantCommand(() -> IO.setSliderTarget(300)),
                                                new InstantCommand(() -> IO.HoldPosition = 0.5)
                                        )
                                ),
                        new WaitUntilCommand(() -> IO.getSliderPosition() >= 200),
//                        new InstantCommand(() -> IO.HoldPosition = 0.5),b jgmbf ed43ew2
                        new InstantCommand(() -> IO.setSliderTarget(850)),
                        new WaitUntilCommand(() -> IO.getSliderPosition() >= 800),
                        new InstantCommand(() -> IO.setGripperState(IO.NOT_GRIPPING)),
                        new InstantCommand(() -> IO.HoldPosition = 0),
                        new WaitCommand(250),
                        new InstantCommand(() -> IO.setDiffyPitch(90)),
                        new InstantCommand(() -> IO.setSliderTarget(0)),
                        new InstantCommand(() -> IO.setArmPosition(0.5)),
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, TofirstElement),
                                new SequentialCommandGroup(
                                        new WaitUntilCommand(() -> IO.getSliderPosition() <= 300),
                                        new InstantCommand(() -> IO.setAngleTarget(900)),
                                        new WaitUntilCommand(() -> IO.getAngleMeasurement() <= 1000),
                                        new InstantCommand(() -> IO.setAngleTarget(0)),
                                        new WaitCommand(250),
                                        new InstantCommand(() -> IO.setArmPosition(IO.ARM_INIT)),
                                        new InstantCommand(() -> IO.setDiffyPitch(180))
                                )
                        ),
                        new InstantCommand(() -> IO.setSliderTarget(400)),
                        new InstantCommand(() -> {
                            IO.setArmPosition(IO.LOADING_SAMPLE+0.1);
                            IO.setDiffyPitch(IO.PITCH_TAKING_SAMPLE);
                        }),
                        new WaitUntilCommand(() -> IO.getSliderPosition() >= 380),
                        new InstantCommand(() -> IO.setDiffyPitch(IO.PITCH_TAKING_SAMPLE + 13)),
                        new InstantCommand(() -> IO.setArmPosition(IO.LOADING_SAMPLE - 0.08)),
                        new WaitCommand(150),
                        new InstantCommand(() -> IO.setGripperState(IO.GRIPPING)),
                        new WaitCommand(350),
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, FirstElementPlace),
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> IO.setSliderTarget(0)),
                                        new InstantCommand(() -> IO.setAngleTarget(2100)),
                                        new InstantCommand(() -> IO.setArmPosition(IO.STRAIGHT)),
                                        new InstantCommand(() -> IO.setDiffyYaw(90)),
                                        new InstantCommand(() -> IO.setDiffyPitch(190)),
                                        new WaitUntilCommand(() -> IO.getAngleMeasurement() >= 1950),
                                        new InstantCommand(() -> IO.setGripperState(IO.NOT_GRIPPING))
                                )
                        ),
                        new WaitCommand(250),
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, ToSecondElement),
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> IO.setAngleTarget(900)),
                                        new WaitUntilCommand(() -> IO.getAngleMeasurement() <= 1000),
                                        new InstantCommand(() -> IO.setAngleTarget(0))
                                )
                        ),
                        new WaitUntilCommand(() -> IO.getAngleMeasurement() <= 100),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> IO.setSliderTarget(560)),
                                new InstantCommand(() -> {
                                    IO.setArmPosition(IO.LOADING_SAMPLE);
                                    IO.setDiffyPitch(IO.PITCH_TAKING_SAMPLE);
                                    IO.setDiffyYaw(40);
                                }),
                                new WaitUntilCommand(() -> IO.getSliderPosition() >= 540),
                                new InstantCommand(() -> IO.setDiffyPitch(IO.PITCH_TAKING_SAMPLE + 13)),
                                new InstantCommand(() -> IO.setArmPosition(IO.LOADING_SAMPLE - 0.08)),
                                new WaitCommand(150),
                                new InstantCommand(() -> IO.setGripperState(IO.GRIPPING)),
                                new WaitCommand(250)
                        ),
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, SecondElementPlace),
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> IO.setSliderTarget(0)),
                                        new InstantCommand(() -> IO.setAngleTarget(2100)),
                                        new InstantCommand(() -> IO.setArmPosition(IO.STRAIGHT)),
                                        new InstantCommand(() -> IO.setDiffyYaw(90)),
                                        new InstantCommand(() -> IO.setDiffyPitch(190)),
                                        new WaitUntilCommand(() -> IO.getAngleMeasurement() >= 1950),
                                        new InstantCommand(() -> IO.setGripperState(IO.NOT_GRIPPING))
                                )
                        ),
                        new WaitCommand(300),
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, ToThirdElement),
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> IO.setAngleTarget(900)),
                                        new WaitUntilCommand(() -> IO.getAngleMeasurement() <= 1000),
                                        new InstantCommand(() -> IO.setAngleTarget(0))
                                )
                        ),
                        new WaitUntilCommand(() -> IO.getAngleMeasurement() <= 100),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> IO.setSliderTarget(1050)),
                                new InstantCommand(() -> {
                                    IO.setArmPosition(IO.LOADING_SAMPLE+0.1);
                                    IO.setDiffyPitch(IO.PITCH_TAKING_SAMPLE);
                                    IO.setDiffyYaw(20);
                                }),
                                new WaitUntilCommand(() -> IO.getSliderPosition() >= 1000),
                                new InstantCommand(() -> IO.setDiffyPitch(IO.PITCH_TAKING_SAMPLE + 13)),
                                new InstantCommand(() -> IO.setArmPosition(IO.LOADING_SAMPLE - 0.08)),
                                new WaitCommand(150),
                                new InstantCommand(() -> IO.setGripperState(IO.GRIPPING)),
                                new WaitCommand(250)
                        ),
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, ThirdElementPlace),
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> IO.setSliderTarget(0)),
                                        new InstantCommand(() -> IO.setAngleTarget(2100)),
                                        new InstantCommand(() -> IO.setArmPosition(IO.STRAIGHT)),
                                        new InstantCommand(() -> IO.setDiffyYaw(90)),
                                        new InstantCommand(() -> IO.setDiffyPitch(190)),
                                        new WaitUntilCommand(() -> IO.getAngleMeasurement() >= 1950),
                                        new InstantCommand(() -> IO.setGripperState(IO.NOT_GRIPPING))
                                )
                        ),
                        new WaitCommand(250),
                        new FollowPathCommand(follower, IntakeSpecimen)
                            .alongWith(
                                    new SequentialCommandGroup(
                                            new InstantCommand(() -> IO.setAngleTarget(900)),
                                            new WaitUntilCommand(() -> IO.getAngleMeasurement() <= 1000),
                                            new InstantCommand(() -> IO.setAngleTarget(0)),
                                            new InstantCommand(() -> IO.setArmPosition(IO.LOADING_SPECIMEN)),
                                            new InstantCommand(() -> IO.setGripperState(IO.NOT_GRIPPING)),
                                            new InstantCommand(() -> IO.setDiffyPitch(IO.PITCH_LOAD_SPECIMEN))
                                    )
                            ),
                        new InstantCommand(() -> IO.setGripperState(IO.GRIPPING)),
                        new WaitCommand(300),
                        new InstantCommand(() -> IO.setDiffyPitch(90)),
                        new InstantCommand(() -> IO.setAngleTarget(2100)),
                        new InstantCommand(() -> IO.setArmPosition(IO.ARM_INIT)),
                        new FollowPathCommand(follower, CycleToBar)
                                .alongWith(
                                new SequentialCommandGroup(
                                        new WaitUntilCommand(() -> IO.getAngleMeasurement() >= 1850),
                                        new InstantCommand(() -> IO.setArmPosition(IO.PLACE_SPECIMEN)),
                                        new InstantCommand(() -> IO.setDiffyPitch(208)),
                                        new InstantCommand(() -> IO.setSliderTarget(500)),
                                        new InstantCommand(() -> IO.HoldPosition = 0.3)

                                )
                        ),
                        new InstantCommand(() -> IO.setSliderTarget(850)),
                        new WaitUntilCommand(() -> IO.getSliderPosition() >= 800),
                        new InstantCommand(() -> IO.setGripperState(IO.NOT_GRIPPING)),
                        new InstantCommand(() -> IO.HoldPosition = 0),
                        new WaitCommand(250),
                        new InstantCommand(() -> IO.setDiffyPitch(90)),
                        new InstantCommand(() -> IO.setSliderTarget(0)),
                        new InstantCommand(() -> IO.setArmPosition(0.5)),
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, CycleToSpecimen),
                                new SequentialCommandGroup(
                                        new WaitUntilCommand(() -> IO.getSliderPosition() <= 300),
                                        new InstantCommand(() -> IO.setAngleTarget(900)),
                                        new WaitUntilCommand(() -> IO.getAngleMeasurement() <= 1000),
                                        new InstantCommand(() -> IO.setAngleTarget(0)),
                                        new InstantCommand(() -> IO.setArmPosition(IO.LOADING_SPECIMEN)),
                                        new InstantCommand(() -> IO.setGripperState(IO.NOT_GRIPPING)),
                                        new InstantCommand(() -> IO.setDiffyPitch(IO.PITCH_LOAD_SPECIMEN))
                                )
                        ),
                        new InstantCommand(() -> IO.setGripperState(IO.GRIPPING)),
                        new WaitCommand(300),
                        new InstantCommand(() -> IO.setDiffyPitch(90)),
                        new InstantCommand(() -> IO.setAngleTarget(2100)),
                        new InstantCommand(() -> IO.setArmPosition(IO.ARM_INIT)),
                        new FollowPathCommand(follower, CycleToBar2)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new WaitUntilCommand(() -> IO.getAngleMeasurement() >= 1850),
                                                new InstantCommand(() -> IO.setArmPosition(IO.PLACE_SPECIMEN)),
                                                new InstantCommand(() -> IO.setDiffyPitch(208)),
                                                new InstantCommand(() -> IO.setSliderTarget(500)),
                                                new InstantCommand(() -> IO.HoldPosition = 0.3)

                                        )
                                ),
                        new InstantCommand(() -> IO.setSliderTarget(850)),
                        new WaitUntilCommand(() -> IO.getSliderPosition() >= 800),
                        new InstantCommand(() -> IO.setGripperState(IO.NOT_GRIPPING)),
                        new InstantCommand(() -> IO.HoldPosition = 0),
                        new WaitCommand(250),
                        new InstantCommand(() -> IO.setDiffyPitch(90)),
                        new InstantCommand(() -> IO.setSliderTarget(0)),
                        new InstantCommand(() -> IO.setArmPosition(0.5)),
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, CycleToSpecimen2),
                                new SequentialCommandGroup(
                                        new WaitUntilCommand(() -> IO.getSliderPosition() <= 300),
                                        new InstantCommand(() -> IO.setAngleTarget(900)),
                                        new WaitUntilCommand(() -> IO.getAngleMeasurement() <= 1000),
                                        new InstantCommand(() -> IO.setAngleTarget(0)),
                                        new InstantCommand(() -> IO.setArmPosition(IO.LOADING_SPECIMEN)),
                                        new InstantCommand(() -> IO.setGripperState(IO.NOT_GRIPPING)),
                                        new InstantCommand(() -> IO.setDiffyPitch(IO.PITCH_LOAD_SPECIMEN))
                                )
                        ),
                        new InstantCommand(() -> IO.setGripperState(IO.GRIPPING)),
                        new WaitCommand(300),
                        new InstantCommand(() -> IO.setDiffyPitch(90)),
                        new InstantCommand(() -> IO.setAngleTarget(2100)),
                        new InstantCommand(() -> IO.setArmPosition(IO.ARM_INIT)),
                        new FollowPathCommand(follower, CycleToBar3)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new WaitUntilCommand(() -> IO.getAngleMeasurement() >= 1850),
                                                new InstantCommand(() -> IO.setArmPosition(IO.PLACE_SPECIMEN)),
                                                new InstantCommand(() -> IO.setDiffyPitch(208)),
                                                new InstantCommand(() -> IO.setSliderTarget(500)),
                                                new InstantCommand(() -> IO.HoldPosition = 0.3)

                                        )
                                ),
                        new InstantCommand(() -> IO.setSliderTarget(850)),
                        new WaitUntilCommand(() -> IO.getSliderPosition() >= 800),
                        new InstantCommand(() -> IO.setGripperState(IO.NOT_GRIPPING)),
                        new InstantCommand(() -> IO.HoldPosition = 0),
                        new WaitCommand(250),
                        new InstantCommand(() -> IO.setDiffyPitch(90)),
                        new InstantCommand(() -> IO.setSliderTarget(0)),
                        new InstantCommand(() -> IO.setArmPosition(0.5)),
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, CycleToSpecimen3),
                                new SequentialCommandGroup(
                                        new WaitUntilCommand(() -> IO.getSliderPosition() <= 300),
                                        new InstantCommand(() -> IO.setAngleTarget(900)),
                                        new WaitUntilCommand(() -> IO.getAngleMeasurement() <= 1000),
                                        new InstantCommand(() -> IO.setAngleTarget(0)),
                                        new InstantCommand(() -> IO.setArmPosition(IO.LOADING_SPECIMEN)),
                                        new InstantCommand(() -> IO.setGripperState(IO.NOT_GRIPPING)),
                                        new InstantCommand(() -> IO.setDiffyPitch(IO.PITCH_LOAD_SPECIMEN))
                                )
                        ),
                        new InstantCommand(() -> IO.setGripperState(IO.GRIPPING)),
                        new WaitCommand(300),
                        new InstantCommand(() -> IO.setDiffyPitch(90)),
                        new InstantCommand(() -> IO.setAngleTarget(2100)),
                        new InstantCommand(() -> IO.setArmPosition(IO.ARM_INIT)),
                        new FollowPathCommand(follower, CycleToBar4)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new WaitUntilCommand(() -> IO.getAngleMeasurement() >= 1850),
                                                new InstantCommand(() -> IO.setArmPosition(IO.PLACE_SPECIMEN)),
                                                new InstantCommand(() -> IO.setDiffyPitch(208)),
                                                new InstantCommand(() -> IO.setSliderTarget(500)),
                                                new InstantCommand(() -> IO.HoldPosition = 0.3)

                                        )
                                ),
                        new InstantCommand(() -> IO.setSliderTarget(850)),
                        new WaitUntilCommand(() -> IO.getSliderPosition() >= 800),
                        new InstantCommand(() -> IO.setGripperState(IO.NOT_GRIPPING)),
                        new InstantCommand(() -> IO.HoldPosition = 0),
                        new WaitCommand(250),
                        new InstantCommand(() -> IO.setSliderTarget(750)),
                        new InstantCommand(() -> IO.setDiffyPitch(90))
//                        new InstantCommand(() -> IO.setArmPosition(0.5))
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
