package org.frc5687.robot;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import org.frc5687.robot.subsystems.DriveTrain;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotState {
//     private static AprilTagFieldLayout _layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    private Thread _periodicThread;
    private volatile boolean _running = false;

    private final ReadWriteLock stateLock = new ReentrantReadWriteLock();
    private final Lock readLock = stateLock.readLock();
    private final Lock writeLock = stateLock.writeLock();

    private DriveTrain _driveTrain;
    // private PhotonProcessor _photonProcessor;
    private SwerveDrivePoseEstimator _poseEstimator;

    private volatile SwerveDriveState _cachedState = new SwerveDriveState();
    private volatile Pose2d _estimatedPose = new Pose2d();
    private volatile Twist2d _velocity = new Twist2d();
    private volatile Pose2d _lastPose = new Pose2d();

    private static RobotState _instance;
    private double _lastTimestamp;
    private final double _period = 1.0 / 200.0; // Run at 200Hz

    private volatile boolean _useVisionUpdates = true;

    public RobotState() {}

    public static RobotState getInstance() {
        if (_instance == null) {
            _instance = new RobotState();
        }
        return _instance;
    }

    public void initializeRobotState(DriveTrain driveTrain) {
        _driveTrain = driveTrain;
        // _photonProcessor = photonProcessor;
        _lastTimestamp = Timer.getFPGATimestamp();
        initPoseEstimator();
        _periodicThread = new Thread(this::run);
        _periodicThread.setDaemon(true);
        _lastTimestamp = Timer.getFPGATimestamp();
    }

    private void initPoseEstimator() {
        _poseEstimator = new SwerveDrivePoseEstimator(
            _driveTrain.getKinematics(),
            _driveTrain.getHeading(),
            _driveTrain.getSwerveModuleMeasuredPositions(),
            new Pose2d(0, 0, _driveTrain.getHeading()),
            createStateStandardDeviations(
                Constants.VisionConfig.STATE_STD_DEV_X,
                Constants.VisionConfig.STATE_STD_DEV_Y,
                Constants.VisionConfig.STATE_STD_DEV_ANGLE),
            createVisionStandardDeviations(
                        Constants.VisionConfig.Auto.VISION_STD_DEV_X,
                        Constants.VisionConfig.Auto.VISION_STD_DEV_Y,
                        Constants.VisionConfig.Auto.VISION_STD_DEV_ANGLE)
            );
    }

    private void run() {
        _running = true;
        while (_running) {
            double startTime = Timer.getFPGATimestamp();
            
            periodic();
            
            double endTime = Timer.getFPGATimestamp();
            double duration = endTime - startTime;
            double sleepTime = _period - duration;
            
            if (sleepTime > 0) {
                Timer.delay(sleepTime);
            }
        }
    }

    public void start() {
        if (!_periodicThread.isAlive()) {
            _periodicThread.start();
        }
    }

    public void stop() {
        _running = false;
        try {
            _periodicThread.join();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    private void updateOdometry() {
        _driveTrain.readSignals();

        SwerveModulePosition[] positions = _driveTrain.getSwerveModuleMeasuredPositions();
        Rotation2d heading = _driveTrain.getHeading();
        Pose2d currentPose = _poseEstimator.getEstimatedPosition();
        double currentTime = Timer.getFPGATimestamp();
        double deltaTime = currentTime - _lastTimestamp;
        ChassisSpeeds measured = _driveTrain.getMeasuredChassisSpeeds();
        _velocity = new Twist2d(measured.vxMetersPerSecond, measured.vyMetersPerSecond, measured.omegaRadiansPerSecond);

        if (deltaTime > 0) {
            _lastPose = new Pose2d(currentPose.getTranslation(), heading);
        }

        _poseEstimator.update(heading, positions);
        _lastTimestamp = currentTime;
        // _estimatedPose = _poseEstimator.getEstimatedPosition();
    }

   private void updateWithVision() {
    //     Pose2d prevEstimatedPose = _estimatedPose; FIXME add this back
    //     List<Pair<EstimatedRobotPose, String>> cameraPoses = Stream.of(
    //             _photonProcessor.getEastCameraEstimatedGlobalPoseWithName(prevEstimatedPose),
    //             _photonProcessor.getSouthCameraEstimatedGlobalPoseWithName(prevEstimatedPose),
    //             _photonProcessor.getWestCameraEstimatedGlobalPoseWithName(prevEstimatedPose),
    //             _photonProcessor.getNorthCameraEstimatedGlobalPoseWithName(prevEstimatedPose)
    //             )
    //             .filter(pair -> pair.getFirst() != null)
    //             .filter(pair -> isValidMeasurementTest(pair))
    //             .collect(Collectors.toList());
            
    //     // for (int i = 0; i < cameraPoses.size() && i < 4; i++) {
    //     //     _latestCameraPoses[i] = cameraPoses.get(i);
    //     // }

    //    cameraPoses.forEach(this::processVisionMeasurement);
    }

    public void periodic() {
        updateOdometry();
        
        // if (_useVisionUpdates) {
        //     updateWithVision();
        // }

        // _visionAngle = getAngleToTagFromVision(getSpeakerTargetTagId());
        // _visionDistance = getDistanceToTagFromVision(getSpeakerTargetTagId());

        // if (_visionAngle.isPresent()) {
        //     SmartDashboard.putNumber("Vision Angle", _visionAngle.get().getRadians());
        // }
        // if (_visionDistance.isPresent()) {
        //     SmartDashboard.putNumber("Vision Distance", _visionDistance.get());
        // }
        _estimatedPose = _poseEstimator.getEstimatedPosition();
    }

    public synchronized Pose2d getEstimatedPose() {
        return _estimatedPose;
    }

    public synchronized void setEstimatedPose(Pose2d pose) {
        _poseEstimator.resetPosition(_driveTrain.getHeading(), _driveTrain.getSwerveModuleMeasuredPositions(), pose);
        _estimatedPose = pose;
        _lastPose = pose;
    }

    public void enableVisionUpdates() {
        _useVisionUpdates = true;
    }

    public void disableVisionUpdates() {
        _useVisionUpdates = false;
    }

    private boolean isValidMeasurementTest(Pair<EstimatedRobotPose, String> estimatedRobotPose) {
        Pose3d measurement = estimatedRobotPose.getFirst().estimatedPose;
        // PhotonTrackedTarget[] tagsUsed = estimatedRobotPose.targetsUsed.;

        // String cameraName = estimatedRobotPose.getSecond();
        if (measurement.getX() > Constants.FieldConstants.FIELD_LENGTH) {
            // DriverStation.reportError("According to " + cameraName +", Robot is off the
            // field in +x direction", false);
            return false;
        } else if (measurement.getX() < 0) {
            // DriverStation.reportError("According to " + cameraName +", Robot is off the
            // field in -x direction", false);
            return false;
        } else if (measurement.getY() > Constants.FieldConstants.FIELD_WIDTH) {
            // DriverStation.reportError("According to " + cameraName +", Robot is off the
            // field in +y direction", false);
            return false;
        } else if (measurement.getY() < 0) {
            // DriverStation.reportError("According to " + cameraName +", Robot is off the
            // field in the -y direction", false);
            return false;
        } else if (measurement.getZ() < -0.15) {
            // DriverStation.reportError("According to " + cameraName +", Robot is inside
            // the floor :(((", false);
            return false;
        } else if (measurement.getZ() > 0.15) {
            // DriverStation.reportError("According to " + cameraName +", Robot is floating
            // above the floor :(((", false);
            return false;
        }
        return true;
    }

    private void processVisionMeasurement(Pair<EstimatedRobotPose, String> cameraPose) {
        EstimatedRobotPose estimatedPose = cameraPose.getFirst();

        double dist = estimatedPose.estimatedPose.toPose2d().getTranslation().getDistance(_estimatedPose.getTranslation());
        
        double positionDev, angleDev;

        // DriverStation.reportError(estimatedPose.strategy.name(), false);
        if (estimatedPose.strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR) {
            // multi-tag estimate, trust it more
            positionDev = 0.1;
            angleDev = Units.degreesToRadians(10); // Try this but IMU is still probably way better
        } else {
            // single-tag estimates, adjust deviations based on distance
            if (dist < 1.5) {
                positionDev = 0.30;
                angleDev = Units.degreesToRadians(500);
            } else if (dist < 4.0) {
                positionDev = 0.45;
                angleDev = Units.degreesToRadians(500);
            } else {
                positionDev = 0.5;
                angleDev = Units.degreesToRadians(500);
            }
        }
        
        _poseEstimator.setVisionMeasurementStdDevs(
            createVisionStandardDeviations(positionDev, positionDev, angleDev)
        );
        
        _poseEstimator.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(),
                estimatedPose.timestampSeconds + Constants.RobotState.VISION_TIMESTAMP_FUDGE);
    }
    
    public void useAutoStandardDeviations() {
        _poseEstimator.setVisionMeasurementStdDevs(createVisionStandardDeviations(
            Constants.VisionConfig.Auto.VISION_STD_DEV_X,
            Constants.VisionConfig.Auto.VISION_STD_DEV_Y,
            Constants.VisionConfig.Auto.VISION_STD_DEV_ANGLE));
    }

    public void useTeleopStandardDeviations() {
        _poseEstimator.setVisionMeasurementStdDevs(createVisionStandardDeviations(
            Constants.VisionConfig.Teleop.VISION_STD_DEV_X,
            Constants.VisionConfig.Teleop.VISION_STD_DEV_Y,
            Constants.VisionConfig.Teleop.VISION_STD_DEV_ANGLE));
    }

    protected Vector<N3> createStandardDeviations(double x, double y, double z) {
        return VecBuilder.fill(x, y, z);
    }

//     /**
//      * @param x     in meters of how much we trust x component
//      * @param y     in meters of how much we trust y component
//      * @param angle in radians of how much we trust the IMU;
//      * @return Standard Deivation of the pose;
//      */
    protected Vector<N3> createStateStandardDeviations(double x, double y, double angle) {
        return createStandardDeviations(x, y, angle);
    }

    protected Vector<N3> createVisionStandardDeviations(double x, double y, double angle) {
        return createStandardDeviations(x, y, angle);
    }

    public Lock getWriteLock() {
        return writeLock;
    }

    public Lock getReadLock() {
        return readLock;
    }
}