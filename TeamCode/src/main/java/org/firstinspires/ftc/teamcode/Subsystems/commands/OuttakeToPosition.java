//package org.firstinspires.ftc.teamcode.Subsystems.commands;
//
//import com.arcrobotics.ftclib.command.CommandBase;
//
//import org.firstinspires.ftc.teamcode.Subsystems.OuttakeSubsystem;
//
//public class OuttakeToPosition extends CommandBase {
//
//    // The subsystem the command runs on
//    private final OuttakeSubsystem outtake;
//
//    double target = 0;
//
//    public OuttakeToPosition(OuttakeSubsystem subsystem, double targets) {
//        outtake = subsystem;
//        target = targets;
//        addRequirements(outtake);
//    }
//
//    @Override
//    public void initialize() {
//        outtake.setTarget(target);
//    }
//
//    public void setTarget(double target1)
//    {
//        target = target1;
//    }
//
//    @Override
//    public boolean isFinished() {
//        return true;
//    }
//
//}