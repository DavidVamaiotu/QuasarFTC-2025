package org.firstinspires.ftc.teamcode.Subsystems.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.OuttakeSubsystem;

public class GrippingBothSides extends CommandBase {

    // The subsystem the command runs on
    private final IntakeSubsystem intake;
    private final OuttakeSubsystem outtake;

    private ElapsedTime elps;

    boolean hasFinished = false;

    double target = 0;

    public GrippingBothSides(IntakeSubsystem subsystem, OuttakeSubsystem subsystem2, double trgt) {
        intake = subsystem;
        outtake = subsystem2;
        target = trgt;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        elps.reset();

    }

    @Override
    public void execute() {
        if (outtake.getPosition() <= 10 && elps.milliseconds() >= 500)
        {
            hasFinished = true;
        }


    }

    public void setTarget(double target1)
    {
        target = target1;
    }


    @Override
    public boolean isFinished() {
        return hasFinished;
    }

}