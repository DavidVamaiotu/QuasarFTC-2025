//package org.firstinspires.ftc.teamcode.Subsystems;
//
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.SubsystemBase;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.arcrobotics.ftclib.controller.PIDFController;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
//import dev.frozenmilk.dairy.cachinghardware.CachingServo;
//
//public class IntakeSubsystem extends SubsystemBase {
//
//    private final CachingServo servo1;
//    private final CachingServo servo2;
//
//    public enum INTAKE_STAGE {
//        READY,
//        SHELLED
//    }
//
//    public enum GRIPPER_STAGE {
//        IN_POSITION,
//        GRIP_ELEMENT
//    }
//
//    public INTAKE_STAGE stage = INTAKE_STAGE.SHELLED;
//
//    public GRIPPER_STAGE gripstage = GRIPPER_STAGE.IN_POSITION;
//
//    public double ARM_INIT_POS = 0.76;
//    public double SLIDER_SHELLED = 0.4;
//    public double SLIDER_EXTENDED = 0.7;
//
//    public double YAW_0 = 0.85;
//    public double YAW_45 = 0.75;
//    public double YAW_90 = 0.5;
//    public double YAW_135 = 0.25;
//    public double YAW_180 = 0;
//
//    public double GRIPPING = 0.62;
//
//    public double NOT_GRIPPING = 0.33;
//
//    public double PITCH_GRIP = 0.33;
//
//    public double PITCH_IDLE = 0.75;
//
//    public double PITCH_TRANSFER = 0.87;
//
//    public double ARM_TRANSFER_POS = 0.82;
//
//    public double ARM_READY_POS = 0.3;
//
//    public double ARM_GRIP_POS = 0.23;
//
//    private final CachingServo arm;
//
//    private final CachingServo gripyaw;
//
//    private final CachingServo grip;
//
//    private final CachingServo grippitch;
//
////    private final CachingServo sldServo;
//
//    public IntakeSubsystem(final HardwareMap hMap) {
//        servo1 = new CachingServo(hMap.get(Servo.class, "srv1"));
//        servo2 = new CachingServo(hMap.get(Servo.class, "srv2"));
//        arm = new CachingServo(hMap.get(Servo.class, "arm"));
//        gripyaw = new CachingServo(hMap.get(Servo.class, "yaw"));
//        grip = new CachingServo(hMap.get(Servo.class, "grip"));
//        grippitch = new CachingServo(hMap.get(Servo.class, "pitch"));
//
//
//        servo1.setDirection(Servo.Direction.REVERSE);
//
//        grippitch.setDirection(Servo.Direction.REVERSE);
//    }
//
//    public void armInit() {
//        arm.setPosition(ARM_INIT_POS);
//        setYaw(YAW_90);
//        setPitch(PITCH_IDLE);
//        sliderShell();
//    }
//
//    public void transferPosition() {
//        gripyaw.setPosition(YAW_90);
//        sliderShell();
//        stage = INTAKE_STAGE.SHELLED;
//    }
//
//    public void readyPosition() {
//        sliderExtended();
//        arm.setPosition(ARM_READY_POS);
//        grippitch.setPosition(PITCH_GRIP);
//        grip.setPosition(NOT_GRIPPING);
//        gripyaw.setPosition(YAW_90);
//        stage = INTAKE_STAGE.READY;
//    }
//
//
//    public void readySetPosition(double pos) {
//        setSliderPosition(pos);
//        grippitch.setPosition(PITCH_GRIP);
//        grip.setPosition(NOT_GRIPPING);
//        gripyaw.setPosition(YAW_90);
//        stage = INTAKE_STAGE.READY;
//    }
//
//
//    public void armToElement() {
//        arm.setPosition(ARM_READY_POS-0.067);
//    }
//
//    public void armToReady() {
//        arm.setPosition(ARM_READY_POS);
//        grippitch.setPosition(PITCH_GRIP);
//    }
//
//    public void goToPosition() {
//        switch(stage) {
//            case SHELLED:
//                readyPosition();
//                break;
//            case READY:
//                transferPosition();
//                break;
//            default:
//                throw new IllegalStateException("Unexpected value: " + stage);
//        }
//    }
//
//    public void setYaw(double yaw) {
//       gripyaw.setPosition(yaw);
//    }
//
//    public void setPitch(double pitch) {
//        grippitch.setPosition(pitch);
//    }
//
//    public void gripState(double gripstate) {
//        grip.setPosition(gripstate);
//    }
//
//
//    public void sliderExtended() {
//        servo1.setPosition(SLIDER_EXTENDED);
//        servo2.setPosition(SLIDER_EXTENDED);
//    }
//
//
//    public void setServoPos1(double pos) {
//        servo1.setPosition(pos);
//    }
//
//    public void setServoPos2(double pos) {
//        servo2.setPosition(pos);
//    }
//
//
//    public void sliderShell() {
//        servo1.setPosition(SLIDER_SHELLED);
//        servo2.setPosition(SLIDER_SHELLED);
//    }
//
//    public void setSliderPosition(double pos) {
//        servo1.setPosition(pos);
//        servo2.setPosition(pos);
//    }
//
//
//    public double getSliderPosition() {
//        return servo1.getPosition();
//    }
//
//    public void ArmAngle(double pos) {
//        arm.setPosition(pos);
//    }
//}