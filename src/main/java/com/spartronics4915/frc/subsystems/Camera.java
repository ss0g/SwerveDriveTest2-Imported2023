package com.spartronics4915.frc.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.spartronics4915.frc.Constants.Camera.*;

public class Camera extends SubsystemBase {
    private final PhotonCamera mCamera;

    public Camera() {
        mCamera = new PhotonCamera(kCameraName);
    }

    public PhotonPipelineResult getLatestResult() {
        return mCamera.getLatestResult();
    }
}
