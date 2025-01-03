package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.teamcode.Vision.SolvePerspective;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

public class CameraSubsystem extends SubsystemBase {

    SolvePerspective AI;

    private OpenCvCamera camera;

    OpenCvWebcam phoneCam;


    public CameraSubsystem(final HardwareMap hardwareMap) {
        this(hardwareMap, "Webcam 1");

    }

    private CameraSubsystem(HardwareMap hardwareMap, String camName) {


        WebcamName webcamName = hardwareMap.get(WebcamName.class, camName);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        AI = new SolvePerspective();
        phoneCam.setPipeline(AI);


    }


    public void startCamera()
    {
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.getWhiteBalanceControl().setMode(WhiteBalanceControl.Mode.MANUAL);
                phoneCam.getWhiteBalanceControl().setWhiteBalanceTemperature(200);
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
    }

    public void EnableRendering()
    {
        AI.Render = true;
    }


    public double GetDegreesBlue()
    {
        return AI.getDetectedStonesBlue().get(0).angle;
    }
    public double GetDegreesRed()
    {
        return AI.getDetectedStonesRed().get(0).angle;
    }

    public void DisableRendering()
    {
        AI.Render = false;
    }

}