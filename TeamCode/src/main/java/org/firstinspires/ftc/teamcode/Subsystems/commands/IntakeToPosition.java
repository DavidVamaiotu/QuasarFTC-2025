//package org.firstinspires.ftc.teamcode.Subsystems.commands;
//
//import com.arcrobotics.ftclib.command.CommandBase;
//
//import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
//
//public class IntakeToPosition extends CommandBase {
//
//    // The subsystem the command runs on
//    private final IntakeSubsystem intake;
//
//    double target = 0;
//
//    public IntakeToPosition(IntakeSubsystem subsystem, double trgt) {
//        intake = subsystem;
//        target = trgt;
//        addRequirements(intake);
//    }
//
//    @Override
//    public void execute() {
//        double output = intake.calculatedPower(target);
//
//        intake.setPower(output);
//
//    }
//
//    public void setTarget(double target1)
//    {
//        target = target1;
//    }
//
//
//    @Override
//    public boolean isFinished() {
//        return intake.atSetPoint() || intake.getPosition() == target;
//    }
//
//}