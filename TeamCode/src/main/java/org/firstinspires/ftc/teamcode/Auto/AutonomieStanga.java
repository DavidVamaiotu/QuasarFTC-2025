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
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.localization.localizers.PinpointLocalizer;
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
@Autonomous (name = "AutonomieStanga", group = "Auto")
public class AutonomieStanga extends CommandOpMode {
    private Telemetry telemetryA;
    private Follower follower;

    private IOSubsystem IO;

    private Pose startPose = new Pose(7.1, 84, Math.toRadians(180));

    private PathChain TohighBar;
    private PathChain TofirstElement;
    private PathChain ToBasket;
    private PathChain ToSecondElement;
    private PathChain ToThirdElement;

    private PathChain ToSecondBasket;

    private PathChain ToThirdBasket;

    private PathChain ToParking;


    private GoBildaPinpointDriver odo;


//    private VoltageSensor volt;



    private Point highBar = new Point(31.5, 77, Point.CARTESIAN);

    private Point firstElement = new Point(18, 120, Point.CARTESIAN);

    private Point Basket = new Point(12.028037383177569, 125.83177570093459, Point.CARTESIAN);

    private Point secondElement = new Point(31, 73, Point.CARTESIAN);

    private Point thirdElement = new Point(31, 73, Point.CARTESIAN);


    private Point Parking   = new Point(58, 95.37570093457945, Point.CARTESIAN);


    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void initialize() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"pin");

//        volt = hardwareMap.voltageSensor.iterator().next();

//        follower.setMaxPower(0.88 * (12 / volt.getVoltage()));

        IO = new IOSubsystem(hardwareMap);

        IO.resetEncoder();

//        follower.setStartingPose(startPose);

        odo.setPosition(new Pose2D(DistanceUnit.INCH, startPose.getX(), startPose.getY(), AngleUnit.RADIANS, startPose.getHeading()));




//        follower.setMaxPower(1);

        // pozitie highBar

        TohighBar = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(startPose.getX(), startPose.getY(), Point.CARTESIAN),
                                highBar
                        )
                )
                .setPathEndTimeoutConstraint(100)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();


        // pozitia primului element

        TofirstElement = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                highBar,
                                firstElement
                        )
                )
                .setPathEndTimeoutConstraint(100) // cat timp sta sa se corecteze robotul
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0)) // unghiul cu care porneste si cel cu care termina
                .build();


        ToBasket = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                firstElement,
                                Basket
                        )
                )
                .setPathEndTimeoutConstraint(100) // cat timp sta sa se corecteze robotul
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45)) // unghiul cu care porneste si cel cu care termina
                .build();


        ToSecondElement = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                Basket,
                                new Point(Basket.getX() + 2.5, Basket.getY(), Point.CARTESIAN)
                        )
                )
                .setPathEndTimeoutConstraint(100) // cat timp sta sa se corecteze robotul
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(8)) // unghiul cu care porneste si cel cu care termina
                .build();


        ToSecondBasket = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(Basket.getX() + 2.5, Basket.getY(), Point.CARTESIAN),
                                Basket
                        )
                )
                .setPathEndTimeoutConstraint(100) // cat timp sta sa se corecteze robotul
                .setLinearHeadingInterpolation(Math.toRadians(6), Math.toRadians(-45)) // unghiul cu care porneste si cel cu care termina
                .build();


        ToThirdElement = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                Basket,
                                new Point(Basket.getX() + 2.5, Basket.getY(), Point.CARTESIAN)
                        )
                )
                .setPathEndTimeoutConstraint(100) // cat timp sta sa se corecteze robotul
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(30)) // unghiul cu care porneste si cel cu care termina
                .build();

        ToThirdBasket = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(Basket.getX() + 2.5, Basket.getY(), Point.CARTESIAN),
                                Basket
                        )
                )
                .setPathEndTimeoutConstraint(100) // cat timp sta sa se corecteze robotul
                .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(-45)) // unghiul cu care porneste si cel cu care termina
                .build();


         ToParking = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                Basket,
                                new Point(63.252, 122.692, Point.CARTESIAN),
                                Parking
                        )
                )
                .setPathEndTimeoutConstraint(100) // cat timp sta sa se corecteze robotul
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(90)) // unghiul cu care porneste si cel cu care termina
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
                                                new WaitCommand(400),
                                                new InstantCommand(() -> IO.setAngleTarget(2100)),
                                                new InstantCommand(() -> IO.setArmPosition(IO.PLACE_SPECIMEN)),
                                                new InstantCommand(() -> IO.setDiffyPitch(175)),
                                                new WaitUntilCommand(() -> IO.getAngleMeasurement() >= 1700),
                                                new InstantCommand(() -> IO.setSliderTarget(300)),
                                                new InstantCommand(() -> IO.HoldPosition = 0.5)
                                        )
                                ),
                        new WaitUntilCommand(() -> IO.getSliderPosition() >= 250),
//                        new InstantCommand(() -> IO.HoldPosition = 0.5),b jgmbf ed43ew2
                        new InstantCommand(() -> IO.setSliderTarget(700)),
                        new WaitUntilCommand(() -> IO.getSliderPosition() >= 650),
                        new InstantCommand(() -> IO.setGripperState(IO.NOT_GRIPPING)),
                        new InstantCommand(() -> IO.HoldPosition = 0),
                        new WaitCommand(250),
                        new InstantCommand(() -> IO.setSliderTarget(0)),
                        new InstantCommand(() -> IO.setArmPosition(0.5)),
                        new WaitUntilCommand(() -> IO.getSliderPosition() <= 300),
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, TofirstElement),
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> IO.setAngleTarget(900)),
                                        new WaitUntilCommand(() -> IO.getAngleMeasurement() <= 1000),
                                        new InstantCommand(() -> IO.setAngleTarget(0)),
                                        new WaitCommand(450),
                                        new InstantCommand(() -> IO.setArmPosition(IO.ARM_INIT)),
                                        new InstantCommand(() -> IO.setDiffyPitch(180))
                                )
                        ),
                        new InstantCommand(() -> IO.setSliderTarget(820)),
                        new InstantCommand(() -> {
                            IO.setArmPosition(IO.LOADING_SAMPLE);
                            IO.setDiffyPitch(IO.PITCH_TAKING_SAMPLE);
                        }),
                        new WaitUntilCommand(() -> IO.getSliderPosition() >= 770),
                        new WaitCommand(150),
                        new InstantCommand(() -> IO.setDiffyPitch(IO.PITCH_TAKING_SAMPLE)),
                        new InstantCommand(() -> IO.setArmPosition(IO.LOADING_SAMPLE - 0.08)),
                        new WaitCommand(200),
                        new InstantCommand(() -> IO.setGripperState(IO.GRIPPING)),
                        new WaitCommand(350),
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, ToBasket),
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> IO.setSliderTarget(0)),
                                        new InstantCommand(() -> IO.setAngleTarget(2100)),
                                        new InstantCommand(() -> IO.setArmPosition(0.48)),
                                        new InstantCommand(() -> IO.setDiffyYaw(90)),
                                        new InstantCommand(() -> IO.setDiffyPitch(125)),
                                        new WaitUntilCommand(() -> IO.getAngleMeasurement() >= 1950),
                                        new InstantCommand(() -> IO.setSliderTarget(1800)),
                                        new WaitCommand(350),
                                        new InstantCommand(() -> IO.HoldPosition = 0.6),
                                        new WaitUntilCommand(() -> IO.getSliderPosition() >= 1750),
                                        new InstantCommand(() -> IO.setArmPosition(IO.STRAIGHT-0.15)),
                                        new InstantCommand(() -> IO.setDiffyYaw(90)),
                                        new InstantCommand(() -> IO.setDiffyPitch(150)),
                                        new WaitCommand(300),
                                        new InstantCommand(() -> IO.setGripperState(IO.NOT_GRIPPING)),
                                        new WaitCommand(250),
                                        new InstantCommand(() -> IO.HoldPosition = 0),
                                        new InstantCommand(() -> IO.setDiffyPitch(90)),
                                        new InstantCommand(() -> IO.setArmPosition(IO.LOADING_SAMPLE-0.08)),
                                        new InstantCommand(() -> IO.setSliderTarget(0))
                                )
                        ),
                        new WaitUntilCommand(() -> IO.getSliderPosition() <= 50),
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, ToSecondElement),
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> IO.setArmPosition(IO.ARM_INIT)),
                                        new InstantCommand(() -> IO.setAngleTarget(900)),
                                        new WaitUntilCommand(() -> IO.getAngleMeasurement() <= 1000),
                                        new InstantCommand(() -> IO.setAngleTarget(0))
                                )
                        ),
                        new WaitUntilCommand(() -> IO.getAngleMeasurement() <= 100),
                        new InstantCommand(() -> IO.setSliderTarget(1090)),
                        new InstantCommand(() -> {
                            IO.setArmPosition(IO.LOADING_SAMPLE);
                            IO.setDiffyPitch(IO.PITCH_TAKING_SAMPLE);
                        }),
                        new WaitUntilCommand(() -> IO.getSliderPosition() >= 1060),
                        new WaitCommand(150),
                        new InstantCommand(() -> IO.setDiffyPitch(IO.PITCH_TAKING_SAMPLE)),
                        new InstantCommand(() -> IO.setArmPosition(IO.LOADING_SAMPLE - 0.08)),
                        new WaitCommand(200),
                        new InstantCommand(() -> IO.setGripperState(IO.GRIPPING)),
                        new WaitCommand(350),
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, ToSecondBasket),
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> IO.setSliderTarget(0)),
                                        new InstantCommand(() -> IO.setAngleTarget(2100)),
                                        new InstantCommand(() -> IO.setArmPosition(0.53)),
                                        new InstantCommand(() -> IO.setDiffyYaw(90)),
                                        new InstantCommand(() -> IO.setDiffyPitch(125)),
                                        new WaitUntilCommand(() -> IO.getAngleMeasurement() >= 1950),
                                        new InstantCommand(() -> IO.setSliderTarget(1800)),
                                        new WaitUntilCommand(() -> IO.getSliderPosition() >= 1750),
                                        new InstantCommand(() -> IO.setArmPosition(IO.STRAIGHT-0.15)),
                                        new InstantCommand(() -> IO.setDiffyYaw(90)),
                                        new InstantCommand(() -> IO.setDiffyPitch(150)),
                                        new WaitCommand(300),
                                        new InstantCommand(() -> IO.setGripperState(IO.NOT_GRIPPING)),
                                        new WaitCommand(250),
                                        new InstantCommand(() -> IO.setDiffyPitch(90)),
                                        new InstantCommand(() -> IO.setArmPosition(IO.LOADING_SAMPLE-0.08)),
                                        new InstantCommand(() -> IO.setSliderTarget(0))
                                )
                        ),
                        new WaitUntilCommand(() -> IO.getSliderPosition() <= 50),
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, ToThirdElement),
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> IO.setArmPosition(IO.ARM_INIT)),
                                        new InstantCommand(() -> IO.setAngleTarget(900)),
                                        new WaitUntilCommand(() -> IO.getAngleMeasurement() <= 1000),
                                        new InstantCommand(() -> IO.setAngleTarget(0))
                                )
                        ),
                        new WaitUntilCommand(() -> IO.getAngleMeasurement() <= 100),
                        new InstantCommand(() -> IO.setSliderTarget(1320)),
                        new InstantCommand(() -> {
                            IO.setArmPosition(IO.LOADING_SAMPLE+0.05);
                            IO.setDiffyPitch(IO.PITCH_TAKING_SAMPLE);
                            IO.setDiffyYaw(135);
                        }),
                        new WaitUntilCommand(() -> IO.getSliderPosition() >= 1290),
                        new WaitCommand(150),
                        new InstantCommand(() -> IO.setDiffyPitch(IO.PITCH_TAKING_SAMPLE)),
                        new InstantCommand(() -> IO.setArmPosition(IO.LOADING_SAMPLE - 0.08)),
                        new WaitCommand(200),
                        new InstantCommand(() -> IO.setGripperState(IO.GRIPPING)),
                        new WaitCommand(350),
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, ToThirdBasket),
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> IO.setSliderTarget(0)),
                                        new InstantCommand(() -> IO.setAngleTarget(2100)),
                                        new InstantCommand(() -> IO.setArmPosition(0.53)),
                                        new InstantCommand(() -> IO.setDiffyYaw(90)),
                                        new InstantCommand(() -> IO.setDiffyPitch(125)),
                                        new WaitUntilCommand(() -> IO.getAngleMeasurement() >= 1950),
                                        new InstantCommand(() -> IO.setSliderTarget(1800)),
                                        new WaitUntilCommand(() -> IO.getSliderPosition() >= 1750),
                                        new InstantCommand(() -> IO.setArmPosition(IO.STRAIGHT-0.15)),
                                        new InstantCommand(() -> IO.setDiffyYaw(90)),
                                        new InstantCommand(() -> IO.setDiffyPitch(150)),
                                        new WaitCommand(300),
                                        new InstantCommand(() -> IO.setGripperState(IO.NOT_GRIPPING)),
                                        new WaitCommand(250),
                                        new InstantCommand(() -> IO.setDiffyPitch(90)),
                                        new InstantCommand(() -> IO.setArmPosition(IO.LOADING_SAMPLE-0.08)),
                                        new InstantCommand(() -> IO.setSliderTarget(0))
                                )
                        ),
                        new WaitUntilCommand(() -> IO.getSliderPosition() <= 50),
                        new InstantCommand(() -> IO.setArmPosition(IO.ARM_INIT)),
                        new InstantCommand(() -> IO.setAngleTarget(900)),
                        new WaitUntilCommand(() -> IO.getAngleMeasurement() <= 1000),
                        new InstantCommand(() -> IO.setAngleTarget(0)),
                        new FollowPathCommand(follower, ToParking),
                        new InstantCommand(() -> IO.setArmPosition(IO.ARM_INIT - 0.16)),
                        new InstantCommand(() -> IO.setAngleTarget(2100))
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
