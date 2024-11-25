package org.frc5687.robot;

import java.util.List;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import org.frc5687.robot.subsystems.DriveTrain;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
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
    private SwerveDrivePoseEstimator _poseEstimator;

    private static RobotState _instance;
    private double _lastTimestamp;
    private final double _period = 1.0 / 200.0; // Run at 200Hz

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
        writeLock.lock();
        try {
            _driveTrain.readSignals();
            SwerveModulePosition[] positions = _driveTrain.getSwerveModuleMeasuredPositions();
            Rotation2d heading = _driveTrain.getHeading();
            Pose2d currentPose = _poseEstimator.getEstimatedPosition();
            double currentTime = Timer.getFPGATimestamp();
            double deltaTime = currentTime - _lastTimestamp;
            ChassisSpeeds measured = _driveTrain.getMeasuredChassisSpeeds();
            // _velocity = new Twist2d(measured.vxMetersPerSecond, measured.vyMetersPerSecond, measured.omegaRadiansPerSecond);

            if (deltaTime > 0) {
                // Translation2d deltaPose = _lastPose.getTranslation().minus(currentPose.getTranslation());
                // Translation2d linearVelocity = deltaPose.div(deltaTime);

                // double deltaHeading = _lastPose.getRotation().minus(heading).getRadians();
                // double angularVelocity = deltaHeading / deltaTime;

                // _velocity = new Twist2d(linearVelocity.getX(), linearVelocity.getY(), angularVelocity);
                // _velocity = _velocityPredictor.getEstimatedVelocity();

                // _lastPose = new Pose2d(currentPose.getTranslation(), heading);
            }

            _poseEstimator.update(heading, positions);
            _lastTimestamp = currentTime;
        } finally {
            writeLock.unlock();
        }
    }

    public void periodic() {
        updateOdometry();
        // updateWithVision();

        readLock.lock();
        try {
            SmartDashboard.putNumber("Estimated X", getEstimatedPose().getX());
            SmartDashboard.putNumber("Estimated Y", getEstimatedPose().getY());
        } finally {
            readLock.unlock();
        }
    }

    private Pose2d getEstimatedPoseThreadSafe() {
        readLock.lock();
        try {
            Pose2d pose = _poseEstimator.getEstimatedPosition();
            return pose;
        } finally {
            readLock.unlock();
        }
    }
    
    public Pose2d getEstimatedPose() {
        return getEstimatedPoseThreadSafe(); //  thread-safe (probably)
    }

    public void setEstimatedPose(Pose2d pose) {
        // FIXME: these values might not be right
        _poseEstimator.resetPosition(_driveTrain.getHeading(), _driveTrain.getSwerveModuleMeasuredPositions(), pose);
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