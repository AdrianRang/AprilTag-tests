// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import static frc.robot.Constants.Vision.*;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  PhotonCamera camera;
  PhotonPoseEstimator photonEstimator;
  private double lastEstTimestamp = 0;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    this.camera = new PhotonCamera(kCameraName); // instantiate the camera

    // instantiate the pose estimator on multi-tag mode
    this.photonEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, kRobotToCam);
    this.photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY); // set the fallback strategy to lowest ambiguity
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    var visionEst = photonEstimator.update();
    double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
    boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
    if (newResult) lastEstTimestamp = latestTimestamp;
    return visionEst;
  }
  
  @Override
  public void periodic() {
    Optional<EstimatedRobotPose> pose = getEstimatedGlobalPose();
    if (pose.isPresent()) {
      Logger.recordOutput("VisionRobotPose", pose.get().estimatedPose);
    }
  }
}
