package org.firstinspires.ftc.teamcode.Manual;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Vision.SolvePerspective;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;
import java.util.concurrent.TimeUnit;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@Config
@TeleOp(name = "ClassyMovementOP")

public class ClassyMovementOP extends CommandOpMode {
    private List<LynxModule> hubs;
    private IntakeSubsystem intake = null;
    private OuttakeSubsystem outtake;

    public CachingDcMotorEx FL = null;
    public CachingDcMotorEx FR = null;
    public CachingDcMotorEx BL = null;
    public CachingDcMotorEx BR = null;

    SolvePerspective AI;

    private OpenCvCamera camera;

    OpenCvWebcam phoneCam;

    double kF = -0.125;


    private ElapsedTime elapsedTime;

    private DriveSubsystem chassis;

    public static double unghi = 45;

    GamepadEx driver2;

    double pow = 1;

    GamepadEx driver1;

    double finalValue = 0;

    double power2 = 1;

    double sliderOffset = 0.355;

    public static int temp = 200;


    ElapsedTime edgeDetector;

    boolean LastAction = false;
    IMU imu;

    /**
     * Code to run during the initialization phase of the OpMode.
     */
    public void initialize() {


        elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        edgeDetector = new ElapsedTime();
        edgeDetector.reset();

        FL = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "FL"));
        FR = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "FR"));
        BL = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "BL"));
        BR = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "BR"));

        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FL.setDirection(DcMotorEx.Direction.REVERSE);
        BL.setDirection(DcMotorEx.Direction.REVERSE);

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));


        AI = new SolvePerspective();

        outtake = new OuttakeSubsystem(hardwareMap);
//        chassis = new DriveSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);



        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        Trigger OuttakeTrigger = new Trigger(() -> driver2.getLeftY() != 0 && intake.stage == IntakeSubsystem.INTAKE_STAGE.SHELLED);
        Trigger IntakeTrigger = new Trigger(() -> driver2.getLeftY() != 0 && intake.stage == IntakeSubsystem.INTAKE_STAGE.READY);

//        chassis.setAxes(driver1::getLeftY, driver1::getLeftX, driver1::getRightX, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));



        register(outtake, intake);


        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        AI = new SolvePerspective();
        phoneCam.setPipeline(AI);


        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.getWhiteBalanceControl().setMode(WhiteBalanceControl.Mode.MANUAL);
                phoneCam.getWhiteBalanceControl().setWhiteBalanceTemperature(temp);
                phoneCam.startStreaming(640,480, OpenCvCameraRotation.SENSOR_NATIVE);

            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        FtcDashboard.getInstance().startCameraStream(phoneCam, 0);


        schedule(
                new InstantCommand(intake::armInit),
                new WaitCommand(300),
                new InstantCommand(outtake::armInit),
                new InstantCommand(() -> outtake.setGripperState(outtake.NOT_GRIPPING))
//                new Command(() -> {
//                    if (gamepad1.dpad_up)
//                    {
//                        LastAction = true;
//                    }
//                    if (gamepad2.dpad_down)
//                    {
//                        LastAction = false;
//                    }
//
//                    if (LastAction)
//                    {
//                        telemetry.addLine(">BLUE ALLIANCE");
//                        telemetry.addLine("RED ALLIANCE");
//                    }
//                    else {
//                        telemetry.addLine("BLUE ALLIANCE");
//                        telemetry.addLine(">RED ALLIANCE");
//                    }
//
//                        telemetry.update();
//                    })
        );


        // Brakes
//        driver1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
//                .whileHeld(() -> chassis.setPowerLimit(0.5))
//                .whenReleased(() -> chassis.setPowerLimit(1.0));
//        driver1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
//                .whileHeld(() -> chassis.setPowerLimit(0.33))
//                .whenReleased(() -> chassis.setPowerLimit(1.0));

        driver2.getGamepadButton(GamepadKeys.Button.A)
                .and(new Trigger(() -> !driver2.isDown(GamepadKeys.Button.RIGHT_BUMPER)))
                .toggleWhenActive(
                        new InstantCommand(() -> {
                            sliderOffset=intake.SLIDER_EXTENDED;
                            intake.readyPosition();
                        }),
                        new SequentialCommandGroup (
                                new InstantCommand(() -> {
                                    intake.ArmAngle(intake.ARM_TRANSFER_POS);
                                    intake.setPitch(intake.PITCH_TRANSFER);
                                }),
                                new WaitCommand(100),
                                new InstantCommand(() -> {
                                    sliderOffset=intake.SLIDER_SHELLED;
                                    intake.transferPosition();
                                })
                        )
                );

        driver2.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(() -> imu.resetYaw());

        driver2.getGamepadButton(GamepadKeys.Button.A)
                .and(new Trigger(() -> intake.stage == IntakeSubsystem.INTAKE_STAGE.READY && driver2.isDown(GamepadKeys.Button.RIGHT_BUMPER)))
                .whenActive(() -> intake.setYaw(intake.YAW_90));

        driver2.getGamepadButton(GamepadKeys.Button.B)
                .and(new Trigger(() -> intake.stage == IntakeSubsystem.INTAKE_STAGE.READY && driver2.isDown(GamepadKeys.Button.RIGHT_BUMPER)))
                .whenActive(() -> intake.setYaw(intake.YAW_135));

        driver2.getGamepadButton(GamepadKeys.Button.X)
                .and(new Trigger(() -> intake.stage == IntakeSubsystem.INTAKE_STAGE.READY && driver2.isDown(GamepadKeys.Button.RIGHT_BUMPER)))
                .whenActive(() -> intake.setYaw(intake.YAW_45));

        driver2.getGamepadButton(GamepadKeys.Button.Y)
                .and(new Trigger(() -> intake.stage == IntakeSubsystem.INTAKE_STAGE.READY && driver2.isDown(GamepadKeys.Button.RIGHT_BUMPER)))
                .whenActive(() -> intake.setYaw(intake.YAW_180));


        driver2.getGamepadButton(GamepadKeys.Button.B)
                .and(new Trigger(() -> intake.stage == IntakeSubsystem.INTAKE_STAGE.READY && !driver2.isDown(GamepadKeys.Button.RIGHT_BUMPER)))
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(() -> intake.setPitch(intake.PITCH_GRIP + 0.05)),
                        new InstantCommand(() -> intake.gripState(intake.NOT_GRIPPING)),
                        new WaitCommand(100),
                        new InstantCommand(intake::armToElement),
                        new WaitCommand(150),
                        new InstantCommand(() -> intake.gripState(intake.GRIPPING)),
                        new WaitCommand(250),
                        new InstantCommand(intake::armToReady)
                ));

        driver2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .and(new Trigger(() -> intake.stage == IntakeSubsystem.INTAKE_STAGE.READY))
                        .whenActive(() -> AI.Render = true)
                        .whileActiveContinuous(() -> {
                            try {
                                if (LastAction)
                                {
                                    telemetry.addData("Angle", AI.getDetectedStonesBlue().get(0).angle);
                                    intake.setYaw(AI.getDetectedStonesBlue().get(0).angle * 0.0055555555555556);
                                }
                                else {
                                    telemetry.addData("Angle", AI.getDetectedStonesRed().get(0).angle);
                                    intake.setYaw(AI.getDetectedStonesRed().get(0).angle * 0.0055555555555556);
                                }

                            }
                            catch(Exception ex) {
                                telemetry.addData("Angle", "nothing found");
                            }

                        })
                        .whenInactive(() -> AI.Render = false);


        driver1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                        .whenPressed(() -> power2 = 0.1)
                        .whenReleased(() -> power2 = 1);

        driver1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> power2 = 0.2)
                .whenReleased(() -> power2 = 1);


        driver2.getGamepadButton(GamepadKeys.Button.X)
                .and(new Trigger(() -> intake.stage == IntakeSubsystem.INTAKE_STAGE.SHELLED))
                        .toggleWhenActive(
                                new InstantCommand(() -> {
                                    outtake.setArticulationAngle(outtake.ARTICULATION_WALL);
                                    outtake.setArmAngle(outtake.ARM_WALL);
                                    outtake.setGripperState(outtake.NOT_GRIPPING);
                                }),
                                new SequentialCommandGroup(
                                    new InstantCommand(() -> {
                                        outtake.setGripperState(outtake.GRIPPING);
                                    }),
                                    new WaitCommand(250),
                                    new InstantCommand(() -> {
                                        outtake.setArmAngle(outtake.ARM_PLACE_ELEMENT + 0.07);
                                        outtake.setArticulationAngle(outtake.ARTICULATION_ELEMENT+0.4);
                                    })
                                )
                        );

        driver2.getGamepadButton(GamepadKeys.Button.Y)
                .and(new Trigger(() -> intake.stage == IntakeSubsystem.INTAKE_STAGE.SHELLED))
                .toggleWhenActive(
                      new SequentialCommandGroup(
                              new InstantCommand(() -> intake.gripState(intake.NOT_GRIPPING)),
                              new InstantCommand(() -> outtake.setGripperState(outtake.GRIPPING)),
                              new WaitCommand(150),
                              new InstantCommand(() -> { sliderOffset=intake.SLIDER_SHELLED + 0.07; intake.setSliderPosition(intake.SLIDER_SHELLED + 0.07);}),
                              new InstantCommand(outtake::readyPosition)
                      ),
                      new SequentialCommandGroup(
                              new InstantCommand(() -> outtake.setGripperState(outtake.NOT_GRIPPING)),
                              new WaitCommand(350),
                              new InstantCommand(outtake::transferPosition)
                      )
                );


        driver2.getGamepadButton(GamepadKeys.Button.Y)
                .and(new Trigger(() -> intake.stage == IntakeSubsystem.INTAKE_STAGE.SHELLED))
                .toggleWhenActive(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> intake.gripState(intake.NOT_GRIPPING)),
                                new InstantCommand(() -> outtake.setGripperState(outtake.GRIPPING)),
                                new WaitCommand(150),
                                new InstantCommand(() -> { sliderOffset=intake.SLIDER_SHELLED + 0.07; intake.setSliderPosition(intake.SLIDER_SHELLED + 0.07);}),
                                new InstantCommand(outtake::readyPosition)
                        ),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> outtake.setGripperState(outtake.NOT_GRIPPING)),
                                new WaitCommand(150),
                                new InstantCommand(outtake::transferPosition)
                        )
                );


        while (opModeInInit()){
                if (gamepad1.dpad_up)
                {
                    LastAction = true;
                }
                if (gamepad1.dpad_down)
                {
                    LastAction = false;
                }

                if (LastAction)
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


//        schedule(
//                new RunCommand(outtake::updatePID)
//        );


//        OuttakeTrigger.whenActive(() -> outtake.setPower(driver2.getLeftY()));
//        IntakeTrigger.whenActive(() -> {
//            if (intake.getSliderPosition()+driver2.getLeftY() <= 0.95 && intake.getSliderPosition()+driver2.getLeftY() >= 0.5)
//            {
//                intake.setSliderPosition(intake.getSliderPosition()+driver2.getLeftY());
//            }
//        });


    }



    /**
     * Loop that runs for every iteration of the OpMode after start is pressed.
     */
    @Override
    public void run() {
        hubs.forEach(LynxModule::clearBulkCache);
        super.run();

        if (intake.stage == IntakeSubsystem.INTAKE_STAGE.SHELLED)
        {
            outtake.setPower(-driver2.getLeftY() + (outtake.getPosition() <= -100 ? -0.125 : 0));
        }
        else {
            finalValue = driver2.getLeftX() * 0.01;

            if (edgeDetector.milliseconds() >= 20 && sliderOffset+finalValue <= 0.95 && sliderOffset+finalValue >= 0.5)
            {
                intake.setSliderPosition(sliderOffset+finalValue);
                sliderOffset = sliderOffset+finalValue;
                edgeDetector.reset();
            }
        }


        if (gamepad1.options)
        {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        FL.setPower(frontLeftPower * power2);
        BL.setPower(backLeftPower * power2);
        FR.setPower(frontRightPower * power2);
        BR.setPower(backRightPower * power2);


        telemetry.addData("Loop times", elapsedTime.milliseconds());
        telemetry.update();
        elapsedTime.reset();
    }
}