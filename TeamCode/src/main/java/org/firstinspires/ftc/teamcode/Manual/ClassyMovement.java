package org.firstinspires.ftc.teamcode.Manual;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
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

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        register(IO, chassis);

        driver1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(() -> chassis.setPowerLimit(0.45))
                .whenReleased(() -> chassis.setPowerLimit(1.0));

        driver1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(() -> chassis.setPowerLimit(0.25))
                .whenReleased(() -> chassis.setPowerLimit(1.0));

        driver2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(IO::toPosition);

        driver2.getGamepadButton(GamepadKeys.Button.Y)
                .and(new Trigger(() -> IO.stage == IOSubsystem.IO_STAGE.OUTTAKE))
                .toggleWhenActive(
                        new InstantCommand(() ->  {
                            IO.setSliderTarget(1900);
                            // fa sa miste bratul aici
                        }),
                        new SequentialCommandGroup(
                                // fa sa puna piesa aici
                                new WaitCommand(300),
                                // fa sa se puna bratul inapoi aici
                                new InstantCommand(() -> IO.setSliderTarget(0))
                        )
                );

        driver2.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(
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

        driver2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(
                        new InstantCommand(() -> {
                            IO.specStage = IOSubsystem.SPECIMEN_STAGE.UNINITIALIZED;
                            //cod sa revina la pozitia de outtake
                        })
                );

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