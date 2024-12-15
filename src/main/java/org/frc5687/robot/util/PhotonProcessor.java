package org.frc5687.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;

import java.util.Optional;
import java.util.concurrent.CompletableFuture;
//import java.util.concurrent.ExecutorService;
//import java.util.concurrent.Executors;

import org.frc5687.robot.Constants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonProcessor {

//     //private static final int NUM_CAMERAS = 4;
//     private final PhotonCamera _eastCamera;
//     private final PhotonCamera _southCamera;
//     private final PhotonCamera _westCamera;
//     private final PhotonCamera _northCamera;
//     private final PhotonPoseEstimator _eastCameraEstimator;
//     private final PhotonPoseEstimator _southCameraEstimator;
//     private final PhotonPoseEstimator _westCameraEstimator;
//     private final PhotonPoseEstimator _northCameraEstimator;

//     //private final ExecutorService _executorService;
//     public PhotonProcessor(AprilTagFieldLayout layout) {
//         _eastCamera = new PhotonCamera("East_Camera");
//         _southCamera = new PhotonCamera("South_Camera"); 
//         _westCamera = new PhotonCamera("West_Camera");
//         _northCamera = new PhotonCamera("North_Camera");
//         // _executorService = Executors.newFixedThreadPool(NUM_CAMERAS);

//         // setPipeline(Pipeline.FAR);
//         // z taken from floor

//         // FIXME: look at the order the rotation transformations are applied -xavier bradford
//         Transform3d robotToEastCam =
//              new Transform3d(
//                 new Translation3d(Units.inchesToMeters(3.0), Units.inchesToMeters(14), Units.inchesToMeters(2.0)),
//                 new Rotation3d(0.0, Units.degreesToRadians(0.0), Units.degreesToRadians(15.0))
//             );

//         Transform3d robotToSouthCam =
//              new Transform3d(
//                 new Translation3d(Units.inchesToMeters(3.0), Units.inchesToMeters(14), Units.inchesToMeters(2.0)),
//                 new Rotation3d(0.0, Units.degreesToRadians(0.0), Units.degreesToRadians(15.0))
//             );

//         Transform3d robotToWestCam =
//             new Transform3d(
//                 new Translation3d(Units.inchesToMeters(3.0), Units.inchesToMeters(14), Units.inchesToMeters(2.0)),
//                 new Rotation3d(0.0, Units.degreesToRadians(0.0), Units.degreesToRadians(15.0))
//             );

//         Transform3d robotToNorthCam =
//              new Transform3d(
//                 new Translation3d(Units.inchesToMeters(3.0), Units.inchesToMeters(14), Units.inchesToMeters(2.0)),
//                 new Rotation3d(0.0, Units.degreesToRadians(0.0), Units.degreesToRadians(15.0))
//             );

//         _eastCameraEstimator =
//             new PhotonPoseEstimator(
//                 layout,
//                 PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
//                 _eastCamera,
//                 robotToEastCam);
            
//         _southCameraEstimator =
//             new PhotonPoseEstimator(
//                 layout,
//                 PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
//                 _southCamera,
//                 robotToSouthCam);

//         _westCameraEstimator =
//             new PhotonPoseEstimator(
//                 layout,
//                 PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
//                 _westCamera,
//                 robotToWestCam);

//         _northCameraEstimator =
//             new PhotonPoseEstimator(
//                 layout,
//                 PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
//                 _northCamera,
//                 robotToNorthCam);

//         _eastCameraEstimator.setMultiTagFallbackStrategy(
//                 PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
//         _southCameraEstimator.setMultiTagFallbackStrategy(
//                 PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
//         _westCameraEstimator.setMultiTagFallbackStrategy(
//                 PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
//         _northCameraEstimator.setMultiTagFallbackStrategy(
//                 PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
//     }

//     public void setPipeline(Pipeline pipeline) {
//         _eastCamera.setPipelineIndex(pipeline.getValue());
//         _southCamera.setPipelineIndex(pipeline.getValue());
//         _westCamera.setPipelineIndex(pipeline.getValue());
//         _northCamera.setPipelineIndex(pipeline.getValue());
//     }
    
//     public double getEastCameraLatency() {
//         return _eastCamera.getLatestResult().getLatencyMillis();
//     }

//     public double getSouthCameraLatency() {
//         return _southCamera.getLatestResult().getLatencyMillis();
//     }
        
//     public double getWestCameraLatency() {
//         return _westCamera.getLatestResult().getLatencyMillis();
//     }

//     public double getNorthCameraLatency() {
//         return _northCamera.getLatestResult().getLatencyMillis();
//     }

//     public boolean hasEastCameraTargets() {
//         return _eastCamera.getLatestResult().hasTargets();
//     }

//     public boolean hasSouthCameraTargets() {
//         return _southCamera.getLatestResult().hasTargets();
//     }

//     public boolean hasWestCameraTargets() {
//         return _westCamera.getLatestResult().hasTargets();
//     }

//     public boolean hasNorthCameraTargets() {
//         return _northCamera.getLatestResult().hasTargets();
//     }

//     public boolean isEastTargetsWithinAmbiguity(double ambiguityTolerance) {
//         var tags = _eastCamera.getLatestResult().targets;
//         for (var tag : tags) {
//             if (tag.getPoseAmbiguity() < ambiguityTolerance) {
//                 return false;
//             }
//         } 
//         return true;
//     }
//     public boolean isSouthTargetsWithinAmbiguity(double ambiguityTolerance) {
//         var tags = _southCamera.getLatestResult().targets;
//         for (var tag : tags) {
//             if (tag.getPoseAmbiguity() < ambiguityTolerance) {
//                 return false;
//             }
//         } 
//         return true;
//     }
//     public boolean isWestTargetsWithinAmbiguity(double ambiguityTolerance) {
//         var tags = _westCamera.getLatestResult().targets;
//         for (var tag : tags) {
//             if (tag.getPoseAmbiguity() < ambiguityTolerance) {
//                 return false;
//             }
//         } 
//         return true;
//     }

//     public boolean isNorthTargetsWithinAmbiguity(double ambiguityTolerance) {
//         var tags = _northCamera.getLatestResult().targets;
//         for (var tag : tags) {
//             if (tag.getPoseAmbiguity() < ambiguityTolerance) {
//                 return false;
//             }
//         } 
//         return true;
//     }

//     public void setLowestAmbiguity() {
//         _eastCameraEstimator.setPrimaryStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
//         _southCameraEstimator.setPrimaryStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
//         _westCameraEstimator.setPrimaryStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
//         _northCameraEstimator.setPrimaryStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
//     }

//     public Optional<EstimatedRobotPose> getSouthCameraEstimatedGlobalPose(
//             Pose2d prevEstimatedPose) {
//         _southCameraEstimator.setReferencePose(prevEstimatedPose);
//         PhotonPipelineResult results = _southCamera.getLatestResult();
//         if (results.hasTargets()) {
//             results.targets.removeIf(tag -> tag.getPoseAmbiguity() > Constants.Vision.AMBIGUITY_TOLERANCE);
//         }
//         return _southCameraEstimator.update(results);
//     }

//     public Optional<EstimatedRobotPose> getWestCameraEstimatedGlobalPose(
//             Pose2d prevEstimatedPose) {
//         _westCameraEstimator.setReferencePose(prevEstimatedPose);
//         PhotonPipelineResult results = _westCamera.getLatestResult();
//         if (results.hasTargets()) {
//             results.targets.removeIf(tag -> tag.getPoseAmbiguity() > Constants.Vision.AMBIGUITY_TOLERANCE);
//         }
//         return _westCameraEstimator.update(results);
//     }

//     public Optional<EstimatedRobotPose> getEastCameraEstimatedGlobalPose(
//         Pose2d prevEstimatedPose) {
//     _eastCameraEstimator.setReferencePose(prevEstimatedPose);
//     PhotonPipelineResult results = _eastCamera.getLatestResult();
//     if (results.hasTargets()) {
//         results.targets.removeIf(tag -> tag.getPoseAmbiguity() > Constants.Vision.AMBIGUITY_TOLERANCE);
//     }
//     return _eastCameraEstimator.update(results);
// }

// public Optional<EstimatedRobotPose> getNorthCameraEstimatedGlobalPose(
//         Pose2d prevEstimatedPose) {
//     _northCameraEstimator.setReferencePose(prevEstimatedPose);
//     PhotonPipelineResult results = _northCamera.getLatestResult();
//     if (results.hasTargets()) {
//         results.targets.removeIf(tag -> tag.getPoseAmbiguity() > Constants.Vision.AMBIGUITY_TOLERANCE);
//     }
//     return _northCameraEstimator.update(results);
// }

// public Pair<EstimatedRobotPose, String> getEastCameraEstimatedGlobalPoseWithName(
//         Pose2d prevEstimatedPose) {
//     _eastCameraEstimator.setReferencePose(prevEstimatedPose);
//     PhotonPipelineResult results = _eastCamera.getLatestResult();
//     if (results.hasTargets()) {
//         results.targets.removeIf(tag -> tag.getPoseAmbiguity() > Constants.Vision.AMBIGUITY_TOLERANCE);
//     }
//     Optional<EstimatedRobotPose> pose = _eastCameraEstimator.update(results);
//     return new Pair<>(pose.orElse(null), "East");
// }


// public Pair<EstimatedRobotPose, String> getSouthCameraEstimatedGlobalPoseWithName(
//         Pose2d prevEstimatedPose) {
//     _southCameraEstimator.setReferencePose(prevEstimatedPose);
//     PhotonPipelineResult results = _southCamera.getLatestResult();
//     if (results.hasTargets()) {
//         results.targets.removeIf(tag -> tag.getPoseAmbiguity() > Constants.Vision.AMBIGUITY_TOLERANCE);
//     }
//     Optional<EstimatedRobotPose> pose = _southCameraEstimator.update(results);
//     return new Pair<>(pose.orElse(null), "North");
// }

// public Pair<EstimatedRobotPose, String> getWestCameraEstimatedGlobalPoseWithName(
//         Pose2d prevEstimatedPose) {
//     _westCameraEstimator.setReferencePose(prevEstimatedPose);
//     PhotonPipelineResult results = _westCamera.getLatestResult();
//     if (results.hasTargets()) {
//         results.targets.removeIf(tag -> tag.getPoseAmbiguity() > Constants.Vision.AMBIGUITY_TOLERANCE);
//     }
//     Optional<EstimatedRobotPose> pose = _westCameraEstimator.update(results);
//     return new Pair<>(pose.orElse(null), "North");
// }

// public Pair<EstimatedRobotPose, String> getNorthCameraEstimatedGlobalPoseWithName(
//         Pose2d prevEstimatedPose) {
//     _northCameraEstimator.setReferencePose(prevEstimatedPose);
//     PhotonPipelineResult results = _northCamera.getLatestResult();
//     if (results.hasTargets()) {
//         results.targets.removeIf(tag -> tag.getPoseAmbiguity() > Constants.Vision.AMBIGUITY_TOLERANCE);
//     }
//     Optional<EstimatedRobotPose> pose = _northCameraEstimator.update(results);
//     return new Pair<>(pose.orElse(null), "North");
// }

//     public CompletableFuture<Optional<EstimatedRobotPose>> getEastCameraEstimatedGlobalPoseAsync(
//             Pose2d prevEstimatedPose) {
//         return CompletableFuture.supplyAsync(
//                 () -> getEastCameraEstimatedGlobalPose(prevEstimatedPose));
//     }

//     public CompletableFuture<Optional<EstimatedRobotPose>> getSouthCameraEstimatedGlobalPoseAsync(
//             Pose2d prevEstimatedPose) {
//         return CompletableFuture.supplyAsync(
//                 () -> getSouthCameraEstimatedGlobalPose(prevEstimatedPose));
//     }

//     public CompletableFuture<Optional<EstimatedRobotPose>> getWestCameraEstimatedGlobalPoseAsync(
//                 Pose2d prevEstimatedPose) {
//         return CompletableFuture.supplyAsync(
//                 () -> getWestCameraEstimatedGlobalPose(prevEstimatedPose));
//     }

//     public CompletableFuture<Optional<EstimatedRobotPose>> getNorthCameraEstimatedGlobalPoseAsync(
//             Pose2d prevEstimatedPose) {
//         return CompletableFuture.supplyAsync(
//                 () -> getNorthCameraEstimatedGlobalPose(prevEstimatedPose));
//     }


//     public enum Pipeline {
//         FAR(0),
//         CLOSE(1);

//         private final int _value;

//         Pipeline(int value) {
//             _value = value;
//         }

//         public int getValue() {
//             return _value;
//         }
//     }
}

