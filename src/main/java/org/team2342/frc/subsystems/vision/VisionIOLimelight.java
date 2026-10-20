// Copyright (c) 2025 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc.subsystems.vision;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonPoseEstimator;
import org.team2342.lib.util.AllianceUtils;
import org.team2342.lib.util.CameraParameters;
import org.team2342.lib.util.GeometryUtils;
import org.team2342.lib.util.Timestamped;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform3d;

import com.limelightvision.FiducialTarget;
import com.limelightvision.Limelight;
import com.limelightvision.PoseEstimateType;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOLimelight implements VisionIO {
  protected final Limelight camera;
  protected final CameraParameters parameters;
  protected final Transform3d robotToCamera;
  private final PhotonPoseEstimator poseEstimator;

  /**
   * Creates a new VisionIOPhotonVision.
   *
   * @param name The configured name of the camera.
   * @param robotToCamera The 3D position of the camera relative to the robot.
   */
  public VisionIOLimelight(CameraParameters parameters) {
    camera = new Limelight(parameters.getCameraName(), GeometryUtils.toPose3d(parameters.getTransform()));
    this.robotToCamera = parameters.getTransform();
    this.parameters = parameters;
    poseEstimator = new PhotonPoseEstimator(AllianceUtils.getFieldLayout(), robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs, Timestamped<Rotation2d> heading) {
    inputs.connected = camera.isConnected();

    poseEstimator.addHeadingData(heading.getTimestamp(), heading.get());

    // Read new camera observations
    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    for (var result : camera.readAcceptedPoseEstimates(PoseEstimateType.MT1_WPIBLUE)) {
      double distance = 0;
      double ambiguity = 0;
      int tagCount = result.reportedTagCount;

      for (FiducialTarget target : result.rawFiducials) {
        distance += target.getTargetPose_CameraSpace().getTranslation().getNorm();
        ambiguity += target.ambiguity;
        tagIds.add((short) target.fiducialId);
      }

      distance /= tagCount;
      ambiguity /= tagCount;

      PoseObservationType type = PoseObservationType.MEGATAG_1;

      // Add observation
      poseObservations.add(
          new PoseObservation(
              result.timestampSeconds, // Timestamp
              new Pose3d(result.pose), // 3D pose estimate
              ambiguity, // Ambiguity
              tagCount, // Tag count
              distance, // Average tag distance
              type)); // Observation type
    }

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }
}
