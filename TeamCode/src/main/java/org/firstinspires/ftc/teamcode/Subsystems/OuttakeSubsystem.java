//package org.firstinspires.ftc.teamcode.Subsystems;
//
//import com.arcrobotics.ftclib.command.SubsystemBase;
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.arcrobotics.ftclib.controller.PIDFController;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
//import dev.frozenmilk.dairy.cachinghardware.CachingServo;
//
//public class OuttakeSubsystem extends SubsystemBase {
//    private final CachingDcMotorEx slider1;
//
//    private final CachingServo arm2;
//
//    private final CachingServo Diffy1;
//    private final CachingServo Diffy2;
//
//    private final CachingServo articulation;
//
////    private final CachingServo sldServo;
//
//    private double kP = 0.02;
//    private double kI = 0;
//    private double kD = 0.0002;
//
//
//    private double currentDiffyYaw = 90;
//    private double currentDiffyPitch = 90;
//    private double currentDiffy1Position = 0.5;
//    private double currentDiffy2Position = 0.5;
//    private final double TICK_PER_DEGREE = 0.0028169014084507;
//
//
//    public double SLIDER_EXTENDED = -1950;
//
//
//    public enum OUTTAKE_STAGE {
//        READY,
//        SHELLED
//    }
//
//    public OuttakeSubsystem.OUTTAKE_STAGE stageo = OUTTAKE_STAGE.SHELLED;
//
//
//
//    public double GRIPPING = 0.3;
//    public double NOT_GRIPPING = 0;
//
//    public double ARTICULATION_WALL = 0.39;
//
//    public double ARM_WALL = 0.95;
//
//    public double ARTICULATION_TRANSFER = 0.46;
//
//    public double ARTICULATION_ELEMENT = 0.57;
//    public double ARM_TRANSFER = 0.26;
//    public double ARM_PLACE_ELEMENT = 0.65;
//    public double ARM_INIT = 0.53;
//    public double SLIDER_SHELLED = 0;
//
//    PIDController pid;
//
//    double target = 0;
//
//    public OuttakeSubsystem(final HardwareMap hMap) {
//        pid = new PIDController(kP, kI, kD);
//        slider1 = new CachingDcMotorEx(hMap.get(DcMotorEx.class, "sld1"));
//        arm2 = new CachingServo(hMap.get(Servo.class, "arm2"));
//        articulation = new CachingServo(hMap.get(Servo.class, "artic"));
//
//        Diffy1 = new CachingServo(hMap.get(Servo.class, "diffy1"));
//        Diffy2 = new CachingServo(hMap.get(Servo.class, "diffy2"));
//
//        Diffy2.setDirection(Servo.Direction.REVERSE);
//
//        slider1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        slider1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        arm2.setDirection(Servo.Direction.FORWARD);
////        slider1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        slider1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        sldServo = new CachingServo(hMap.get(Servo.class, "srv2"));
//    }
//
//    public void setPower(double power) {
//         slider1.setPower(power);
//    }
//
//    public void extendSlider() {
//        setTarget(SLIDER_EXTENDED);
//    }
//
//
//    public void readyPosition()
//    {
//        extendSlider();
//        arm2.setPosition(ARM_PLACE_ELEMENT);
//        articulation.setPosition(ARTICULATION_ELEMENT);
//        stageo = OUTTAKE_STAGE.READY;
//    }
//
//
//    public void initDiffy()
//    {
//        currentDiffyYaw = 90;
//        currentDiffyPitch = 90;
//        currentDiffy1Position = 0.5;
//        currentDiffy2Position = 0.5;
//        Diffy1.setPosition(0.5);
//        Diffy2.setPosition(0.5);
//    }
//    public void setDiffyYaw(double deg)
//    {
//        double yawDifference = deg - currentDiffyYaw;
//        double addedPosition = yawDifference * TICK_PER_DEGREE;
//
//        Diffy1.setPosition(currentDiffy1Position + addedPosition);
//        Diffy2.setPosition(currentDiffy2Position - addedPosition);
//
//        currentDiffy1Position = currentDiffy1Position + addedPosition;
//        currentDiffy2Position = currentDiffy2Position - addedPosition;
//        currentDiffyYaw = deg;
//
//    }
//
//    public double getTarget1()
//    {
//       return Diffy1.getPosition();
//    }
//
//    public double getTarget2()
//    {
//        return Diffy2.getPosition();
//    }
//
//    public void setDiffyPitch(double deg)
//    {
//        double pitchDifference = deg - currentDiffyPitch;
//        double addedPosition = pitchDifference * TICK_PER_DEGREE;
//
//        Diffy1.setPosition(currentDiffy1Position + addedPosition);
//        Diffy2.setPosition(currentDiffy2Position + addedPosition);
//
//        currentDiffy1Position = currentDiffy1Position + addedPosition;
//        currentDiffy2Position = currentDiffy2Position + addedPosition;
//        currentDiffyPitch = deg;
//    }
//
//
//
//    public void readySetPosition(double offset)
//    {
//        arm2.setPosition(ARM_PLACE_ELEMENT - offset);
//        articulation.setPosition(ARTICULATION_ELEMENT);
//        stageo = OUTTAKE_STAGE.READY;
//    }
//
//
//    public void transferPosition()
//    {
//        arm2.setPosition(ARM_TRANSFER);
//        articulation.setPosition(ARTICULATION_TRANSFER);
//        shellSlider();
//        stageo = OUTTAKE_STAGE.SHELLED;
//    }
//
//
//    public void toPosition()
//    {
//       switch(stageo)
//       {
//           case SHELLED:
//               readyPosition();
//               break;
//           case READY:
//               transferPosition();
//               break;
//           default:
//               throw new IllegalStateException("Unexpected value: " + stageo);
//       }
//    }
//
//    public void setArmAngle(double pos) {
//        arm2.setPosition(pos);
//    }
//
//    public void armInit() {
//        setArmAngle(ARM_TRANSFER);
//        setArticulationAngle(ARTICULATION_TRANSFER);
//        stageo = OUTTAKE_STAGE.SHELLED;
//    }
//
//
//    public void setArticulationAngle(double pos) {
//        articulation.setPosition(pos);
//    }
//
//    public void shellSlider() {
//        setTarget(SLIDER_SHELLED);
//    }
//
//    public void updatePID() {
//        double output = pid.calculate(
//                slider1.getCurrentPosition(), target
//        );
//
//        slider1.setPower(output);
//    }
//
//    public void setTarget(double targetsa) {
//        target = targetsa;
//    }
//
////    public void setServoPosition(double position) {
////        sldServo.setPosition(position);
////    }
//
//    public double getPosition()
//    {
//        return slider1.getCurrentPosition();
//    }
//
//}