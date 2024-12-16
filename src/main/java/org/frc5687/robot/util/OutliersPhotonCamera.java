package org.frc5687.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;;

public class OutliersPhotonCamera {

    private PhotonCamera _cam;
    private Transform3d _robotToCamera;
    private PhotonPoseEstimator _poseEstimator;

    public OutliersPhotonCamera(String cameraName, Pipeline initialPipeline, Transform3d robotToCamera) {
        _cam = new PhotonCamera(cameraName);
        setPipeline(initialPipeline);
        _poseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
        _poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        _robotToCamera = robotToCamera;
    }

    private void setPipeline(Pipeline pipeline) {
        _cam.setPipelineIndex(pipeline.getValue());
    }

    private Pipeline getCurrentPipeline() {
        return Pipeline.fromInt(_cam.getPipelineIndex());
    }

    public Optional<EstimatedRobotPose> getRobotPose() {
        if (getCurrentPipeline() != Pipeline.AprilTag) {
            System.err.println("Current Pipeline of camera " + _cam.getName() + " is " + getCurrentPipeline() + " but expected pipeline " + Pipeline.AprilTag);
        }
        List<PhotonPipelineResult> results = _cam.getAllUnreadResults();
        if (results.size() < 1) {
            // System.err.println("Could not get pose because there were no pipeline results on camera "+_cam.getName());
            return Optional.empty();
        }
        return _poseEstimator.update(results.get(results.size() - 1));
    }

    public List<Pose2d> getDetectedObjectRobotRelativePoses() {
        if (getCurrentPipeline() != Pipeline.ObjectDetection) {
            System.err.println("Current Pipeline of camera " + _cam.getName() + " is " + getCurrentPipeline() + " but expected pipeline " + Pipeline.ObjectDetection);
        }
        List<PhotonPipelineResult> results = _cam.getAllUnreadResults();
        if (results.size() < 1) {
            System.err.println("Could not get detected objects because there were no pipeline results on camera "+_cam.getName());
            return List.of();
        }
        PhotonPipelineResult mostRecent = results.get(results.size() - 1);
        List<PhotonTrackedTarget> targets = mostRecent.targets;
        for (PhotonTrackedTarget target : targets) {
            // TODO find the intersection between the ray and the ground plane.
        }
        return List.of(); // FIXME finish implementing this function
    }

    public enum Pipeline {
        AprilTag,
        ObjectDetection;
        private static final Pipeline[] values = Pipeline.values();
        public int getValue() {
            for (int i = 0; i < Pipeline.values.length; i++) {
                if (Pipeline.fromInt(i) == this) {
                    return i;
                }
            }
            return -1;
        }

        public static Pipeline fromInt(int i) {
            return Pipeline.values[i];
        }
    }

}

