package org.firstinspires.ftc.teamcode.Manual;

import android.graphics.Camera;
import android.graphics.Path;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IOSubsystem;

import java.util.HashMap;
import java.util.List;

@Config
@TeleOp(name = "ClassyMovement <3 (tudor & franci)")
public class ClassyMovement extends CommandOpMode {
    private List<LynxModule> hubs;

    private ElapsedTime loopTimes;

    private DriveSubsystem chassis;

    private IOSubsystem IO;

    private CameraSubsystem cam;

    boolean ALLIANCE = false;

    GamepadEx driver2;

    GamepadEx driver1;

    IMU imu;

    /**
     * Code to run during the initialization phase of the OpMode.
     */
    public void initialize() {

        hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

        imu.resetYaw();

        loopTimes = new ElapsedTime();
        loopTimes.reset();

        chassis = new DriveSubsystem(hardwareMap);
        IO = new IOSubsystem(hardwareMap);
        cam = new CameraSubsystem(hardwareMap);

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        register(IO, chassis, cam);

        cam.startCamera();

        schedule(
                new RunCommand(IO::updateAngle),
                new RunCommand(IO::updatePosition),
                new SequentialCommandGroup(
                        new WaitUntilCommand(this::opModeIsActive),
                        new InstantCommand(IO::initDiffy),
                        new InstantCommand(() -> IO.setDiffyPitch(180)),
                        new InstantCommand(() -> IO.setArmPosition(IO.ARM_INIT))
                )
        );

        driver1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(() -> chassis.setPowerLimit(0.45))
                .whenReleased(() -> chassis.setPowerLimit(1.0));

        driver1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(() -> chassis.setPowerLimit(0.25))
                .whenReleased(() -> chassis.setPowerLimit(1.0));

        driver2.getGamepadButton(GamepadKeys.Button.A)
                .and(new Trigger(() -> IO.stage != IOSubsystem.IO_STAGE.OUTTAKE_UNLOADING))
                .and(new Trigger(() -> !driver2.isDown(GamepadKeys.Button.RIGHT_BUMPER)))
                .whenActive(
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> {
                                            IO.specStage = IOSubsystem.SPECIMEN_STAGE.UNINITIALIZED;
                                            IO.stage = IOSubsystem.IO_STAGE.OUTTAKE;
                                            IO.setArmPosition(IO.ARM_INIT);
                                            IO.setDiffyPitch(180);
                                            IO.setDiffyYaw(90);
                                            IO.setSliderTarget(0);
                                        }),
                                        new SequentialCommandGroup(
                                                new WaitUntilCommand(() -> IO.getSliderPosition() <= 20),
                                                new InstantCommand(() -> IO.setAngleTarget(2100)),
                                                new WaitUntilCommand(() -> IO.getAngleMeasurement() >= 1900),
                                                new WaitCommand(350),
                                                new InstantCommand(() -> IO.HoldPosition = 0.25)
                                        )
//                                                .interruptOn(() -> IO.stage == IOSubsystem.IO_STAGE.INTAKE)
                                ),
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> {
                                            IO.specStage = IOSubsystem.SPECIMEN_STAGE.UNINITIALIZED;
                                            IO.stage = IOSubsystem.IO_STAGE.INTAKE;
                                            IO.HoldPosition = 0;
                                            IO.setDiffyPitch(90);
                                            IO.setDiffyYaw(90);
                                            IO.setAngleTarget(10);
                                        }),
                                        new SequentialCommandGroup(
                                                new WaitUntilCommand(() -> IO.getAngleMeasurement() <= 50),
                                                new InstantCommand(() -> {
                                                    IO.setArmPosition(IO.LOADING_SAMPLE);
                                                    IO.setGripperState(IO.NOT_GRIPPING);
                                                })
                                        )
//                                                .interruptOn(() -> IO.stage == IOSubsystem.IO_STAGE.OUTTAKE)
                                ),
                                () -> (IO.stage == IOSubsystem.IO_STAGE.INTAKE)
                        )
                );

        new Trigger(() -> driver2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= 0.2)
                .and(new Trigger(() -> IO.stage == IOSubsystem.IO_STAGE.INTAKE))
                .whenActive(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> IO.setSliderTarget(1000)),
                                new WaitUntilCommand(() -> IO.getSliderPosition() >= 700),
                                new InstantCommand(() -> {
                                    IO.setArmPosition(IO.LOADING_SAMPLE);
                                    IO.setDiffyPitch(IO.PITCH_TAKING_SAMPLE);
                                })
                    )
                );


        driver2.getGamepadButton(GamepadKeys.Button.A)
                .and(new Trigger(() -> IO.stage == IOSubsystem.IO_STAGE.INTAKE))
                .and(new Trigger(() -> driver2.isDown(GamepadKeys.Button.RIGHT_BUMPER)))
                .whenActive(() -> { IO.setDiffyYaw(90); IO.setDiffyPitch(-25); });

        driver2.getGamepadButton(GamepadKeys.Button.X)
                .and(new Trigger(() -> IO.stage == IOSubsystem.IO_STAGE.INTAKE))
                .and(new Trigger(() -> driver2.isDown(GamepadKeys.Button.RIGHT_BUMPER)))
                .whenActive(() -> { IO.setDiffyYaw(45); IO.setDiffyPitch(-25); });

        driver2.getGamepadButton(GamepadKeys.Button.B)
                .and(new Trigger(() -> IO.stage == IOSubsystem.IO_STAGE.INTAKE))
                .and(new Trigger(() -> driver2.isDown(GamepadKeys.Button.RIGHT_BUMPER)))
                .whenActive(() -> { IO.setDiffyYaw(135); IO.setDiffyPitch(-25); });

        driver2.getGamepadButton(GamepadKeys.Button.Y)
                .and(new Trigger(() -> IO.stage == IOSubsystem.IO_STAGE.INTAKE))
                .and(new Trigger(() -> driver2.isDown(GamepadKeys.Button.RIGHT_BUMPER)))
                .whenActive(() -> { IO.setDiffyYaw(0); IO.setDiffyPitch(-25); });



        driver2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenActive(() -> cam.EnableRendering())
                .whenActive(
                        new InstantCommand(() -> {
                            try {
                                double degreesCalculated = ALLIANCE ? cam.GetDegreesBlue() : cam.GetDegreesRed();
                                double addedValue = degreesCalculated - IO.currentDiffyYaw;
                                IO.setDiffyYaw(IO.currentDiffyYaw + addedValue);
                                IO.setDiffyPitch(-25);
                            } catch(Exception ex) {
                                telemetry.addLine("Not found");
                            }
                        })
                )
                .whenInactive(() -> cam.DisableRendering());

        new Trigger(() -> driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >= 0.2)
                .and(new Trigger(() -> IO.stage == IOSubsystem.IO_STAGE.INTAKE))
                .whenActive(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> IO.setDiffyPitch(90)),
                                new InstantCommand(() -> IO.setSliderTarget(0))
                        )
                );

//        driver2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
//                        .and(new Trigger(() -> IO.stage == IOSubsystem.IO_STAGE.INTAKE || IO.stage == IOSubsystem.IO_STAGE.INTAKE_LOADING))
//                                .whenActive(
//                                        new ConditionalCommand(
//                                                new SequentialCommandGroup(
//                                                        new InstantCommand(() -> IO.setDiffyPitch(90)),
//                                                        new InstantCommand(() -> IO.setSliderTarget(0)),
//                                                        new InstantCommand(() -> {
//                                                            IO.stage = IOSubsystem.IO_STAGE.INTAKE;
//                                                            IO.specStage = IOSubsystem.SPECIMEN_STAGE.UNINITIALIZED;
//                                                        })
//                                                ),
//                                                new SequentialCommandGroup(
//                                                        new InstantCommand(() -> IO.setSliderTarget(800)),
//                                                        new WaitUntilCommand(() -> IO.getSliderPosition() >= 700),
//                                                        new InstantCommand(() -> {
//                                                            IO.setArmPosition(IO.LOADING_SAMPLE);
//                                                            IO.setDiffyPitch(IO.PITCH_TAKING_SAMPLE);
//                                                            IO.stage = IOSubsystem.IO_STAGE.INTAKE_LOADING;
//                                                            IO.specStage = IOSubsystem.SPECIMEN_STAGE.UNINITIALIZED;
//                                                        })
//                                                ),
//                                                () -> IO.stage == IOSubsystem.IO_STAGE.INTAKE_LOADING
//                                        )
//                                );

        driver2.getGamepadButton(GamepadKeys.Button.B)
                .and(new Trigger(() -> IO.stage == IOSubsystem.IO_STAGE.INTAKE))
                .and(new Trigger(() -> IO.getSliderPosition() >= 700))
                .and(new Trigger(() -> !driver2.isDown(GamepadKeys.Button.RIGHT_BUMPER)))
                .whenActive(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> gamepad1.rumble(50)),
                                new InstantCommand(() -> IO.setGripperState(IO.NOT_GRIPPING)),
                                new InstantCommand(() -> IO.setDiffyPitch(IO.PITCH_TAKING_SAMPLE)),
                                new WaitCommand(100),
                                new InstantCommand(() -> IO.setArmPosition(IO.LOADING_SAMPLE+0.06)),
                                new WaitCommand(100),
                                new InstantCommand(() -> IO.setGripperState(IO.GRIPPING)),
                                new WaitCommand(150),
                                new InstantCommand(() -> IO.setArmPosition(IO.LOADING_SAMPLE)),
                                new InstantCommand(() -> IO.setDiffyPitch(IO.PITCH_TAKING_SAMPLE+5))
                        )
                );

        driver2.getGamepadButton(GamepadKeys.Button.Y)
                .and(new Trigger(() -> IO.stage == IOSubsystem.IO_STAGE.OUTTAKE || IO.stage == IOSubsystem.IO_STAGE.OUTTAKE_UNLOADING))
                .toggleWhenActive(
                        new InstantCommand(() ->  {
                            IO.HoldPosition = 0.65;
                            IO.setSliderTarget(1850);
                            // fa sa miste bratul aici
                            IO.setArmPosition(IO.PLACING_SAMPLE);
                            IO.setDiffyPitch(120);
                            IO.stage = IOSubsystem.IO_STAGE.OUTTAKE_UNLOADING;
                        }),
                        new SequentialCommandGroup(
                                // fa sa puna piesa aici
                                new InstantCommand(() -> {
                                    IO.specStage = IOSubsystem.SPECIMEN_STAGE.UNINITIALIZED;
                                    IO.stage = IOSubsystem.IO_STAGE.INTAKE;
                                }),
                                new InstantCommand(() -> IO.setGripperState(IO.NOT_GRIPPING)),
                                new WaitCommand(350),
                                new InstantCommand(() -> IO.setDiffyPitch(90)),
                                new WaitCommand(50),
                                new InstantCommand(() -> IO.setArmPosition(IO.LOADING_SAMPLE+0.3)),
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> {
                                            IO.setSliderTarget(0);
                                            IO.setDiffyPitch(180);
                                            IO.setDiffyYaw(90);
                                        }),
                                        new WaitUntilCommand(() -> IO.getSliderPosition() <= 50),
                                        new InstantCommand(() -> IO.setArmPosition(IO.ARM_INIT)),
                                        new InstantCommand(() -> IO.HoldPosition = 0),
                                        new InstantCommand(() -> IO.setAngleTarget(10))
//                                        new SequentialCommandGroup(
//                                              new WaitUntilCommand(() -> IO.getSliderPosition() <= 10),
//                                              new InstantCommand(() -> IO.setArmPosition(IO.ARM_INIT))
//                                        )
                                )
//                                        .interruptOn(() -> IO.stage == IOSubsystem.IO_STAGE.OUTTAKE)
                                // fa sa se puna bratul inapoi aici
                        )
                );

        driver2.getGamepadButton(GamepadKeys.Button.X)
                .and(new Trigger(() -> !driver2.isDown(GamepadKeys.Button.RIGHT_BUMPER)))
                .whenActive(
                        new SelectCommand(
                                // the first parameter is a map of commands
                                new HashMap<Object, Command>() {{
                                    put(IOSubsystem.SPECIMEN_STAGE.UNINITIALIZED, new InstantCommand(()-> telemetry.addLine("Test1")));
                                    put(IOSubsystem.SPECIMEN_STAGE.LOADING_SPECIMEN, new InstantCommand(()-> telemetry.addLine("Test2")));
                                    put(IOSubsystem.SPECIMEN_STAGE.PLACING_SPECIMEN, new InstantCommand(()-> telemetry.addLine("Test3")));
                                }},
                                // the selector
                                this::getSpecStage
                        )
                );

        while(opModeInInit())
        {
            telemetry.addLine("CHOOSE ALLIANCE");

            if (gamepad1.dpad_up)
            {
                ALLIANCE = true;
            }
            if (gamepad2.dpad_down)
            {
                ALLIANCE = false;
            }

            if (ALLIANCE)
            {
                telemetry.addLine(">BLUE ALLIANCE");
                telemetry.addLine("RED ALLIANCE");
            }
            else {
                telemetry.addLine("BLUE ALLIANCE");
                telemetry.addLine(">RED ALLIANCE");
            }

            telemetry.update();

        }

    }

    public IOSubsystem.SPECIMEN_STAGE getSpecStage()
    {
        return IO.specStage;
    }


    /**
     * Loop that runs for every iteration of the OpMode after start is pressed.
     */
    @Override
    public void run() {
        hubs.forEach(LynxModule::clearBulkCache);
        super.run();

        if (gamepad1.options)
            imu.resetYaw();

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        chassis.updateSpeeds(driver1.getLeftY(), driver1.getLeftX(), driver1.getRightX(), botHeading);

        telemetry.addData("Loop Times", loopTimes.milliseconds());
        telemetry.update();
        loopTimes.reset();
    }
}