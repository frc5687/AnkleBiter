package org.frc5687.robot.subsystems;

import static org.frc5687.robot.Constants.DriveTrain.HEADING_kD;
import static org.frc5687.robot.Constants.DriveTrain.HEADING_kI;
import static org.frc5687.robot.Constants.DriveTrain.HEADING_kP;
import static org.frc5687.robot.Constants.DriveTrain.NUM_MODULES;

import java.util.Optional;

import org.frc5687.lib.control.SwerveHeadingController;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;
import org.frc5687.robot.RobotState;
import org.frc5687.robot.util.OculusProcessor;
import org.frc5687.robot.util.OutliersContainer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import org.frc5687.lib.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class DriveTrain extends OutliersSubsystem {
    public enum ControlState {
        /* Neutral state */
        NEUTRAL(0),
        /* Driving With Joystick */
        MANUAL(1),

        /* Use pose controller to drive */
        POSITION(2),

        /* Rotation only control */
        ROTATION(3),
        /* Trajectory driving control */
        TRAJECTORY(4);

        private final int _value;

        ControlState(int value) {
            _value = value;
        }

        public int getValue() {
            return _value;
        }
    }

    /**
     * SystemIO is input, output of our drivetrain that we want to cache.
     */
    public static class SystemIO {
        ChassisSpeeds desiredChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        SwerveModuleState[] measuredStates = {
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
        };
        SwerveModulePosition[] measuredPositions = {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
        };
        Rotation2d heading = new Rotation2d(0.0);
        Pose2d odometryPose = new Pose2d();

        SwerveSetpoint setpoint = new SwerveSetpoint(new ChassisSpeeds(), new SwerveModuleState[4], DriveFeedforwards.zeros(4));
    }

    // Order we define swerve modules in kinematics
    private final SwerveModule[] _modules;
    private static final int NORTH_WEST_IDX = 0;
    private static final int SOUTH_WEST_IDX = 1;
    private static final int SOUTH_EAST_IDX = 2;
    private static final int NORTH_EAST_IDX = 3;

    private final SwerveSetpointGenerator _swerveSetpointGenerator;

    private final BaseStatusSignal[] _signals;

    private final SwerveDriveKinematics _kinematics;

    private final SwerveHeadingController _headingController;

    // controllers [Heading, Pose, Trajectory]
    private ControlState _controlState;

    // IMU (Pigeon)
    private final Pigeon2 _imu;
    private double _yawOffset;

    private final SystemIO _systemIO;
    private final HolonomicDriveController _poseController;
    public final OculusProcessor _oculusProcessor;

    private RobotState _robotState = RobotState.getInstance();

    private boolean _fieldCentric = true;

    public DriveTrain(
            OutliersContainer container,
            Pigeon2 imu) {
        super(container);
        SignalLogger.setPath("/home/lvuser/logs");
        SignalLogger.start();

        // configure our system IO and pigeon;
        _imu = imu;
        _systemIO = new SystemIO();

        _oculusProcessor = new OculusProcessor();

        _controlState = ControlState.NEUTRAL;

        // set up the modules
        _modules = new SwerveModule[4];

        _modules[NORTH_WEST_IDX] = new SwerveModule(
                Constants.DriveTrain.NORTH_WEST_CONFIG,
                RobotMap.CAN.TALONFX.NORTH_WEST_ROTATION,
                RobotMap.CAN.TALONFX.NORTH_WEST_TRANSLATION,
                RobotMap.CAN.CANCODER.ENCODER_NW);
        _modules[SOUTH_WEST_IDX] = new SwerveModule(
                Constants.DriveTrain.SOUTH_WEST_CONFIG,
                RobotMap.CAN.TALONFX.SOUTH_WEST_ROTATION,
                RobotMap.CAN.TALONFX.SOUTH_WEST_TRANSLATION,
                RobotMap.CAN.CANCODER.ENCODER_SW);
        _modules[SOUTH_EAST_IDX] = new SwerveModule(
                Constants.DriveTrain.SOUTH_EAST_CONFIG,
                RobotMap.CAN.TALONFX.SOUTH_EAST_ROTATION,
                RobotMap.CAN.TALONFX.SOUTH_EAST_TRANSLATION,
                RobotMap.CAN.CANCODER.ENCODER_SE);
        _modules[NORTH_EAST_IDX] = new SwerveModule(
                Constants.DriveTrain.NORTH_EAST_CONFIG,
                RobotMap.CAN.TALONFX.NORTH_EAST_ROTATION,
                RobotMap.CAN.TALONFX.NORTH_EAST_TRANSLATION,
                RobotMap.CAN.CANCODER.ENCODER_NE);

        // module CAN bus sensor outputs (position, velocity of each motor) all of them
        // are called once per loop at the start.
        _signals = new BaseStatusSignal[NUM_MODULES * 4 + 3];
        for (int i = 0; i < NUM_MODULES; ++i) {
            var signals = _modules[i].getSignals();
            _signals[(i * 4)] = signals[0];
            _signals[(i * 4) + 1] = signals[1];
            _signals[(i * 4) + 2] = signals[2];
            _signals[(i * 4) + 3] = signals[3];
        }
        _signals[NUM_MODULES * 4] = _imu.getYaw();
        _signals[NUM_MODULES * 4 + 1] = _imu.getPitch();
        _signals[NUM_MODULES * 4 + 2] = _imu.getRoll();

        // frequency in Hz
        configureSignalFrequency(250);

        _kinematics = new SwerveDriveKinematics(
                _modules[NORTH_WEST_IDX].getModuleLocation(),
                _modules[SOUTH_WEST_IDX].getModuleLocation(),
                _modules[SOUTH_EAST_IDX].getModuleLocation(),
                _modules[NORTH_EAST_IDX].getModuleLocation());

        // FIXME get real COF - xavier
        var _config = new RobotConfig(
            Units.Pounds.of(50.0),
            Units.KilogramSquareMeters.of(9),
            new ModuleConfig(
                Constants.SwerveModule.WHEEL_RADIUS,
                Constants.DriveTrain.MAX_MPS,
                1.0,
                DCMotor.getKrakenX60Foc(1),
                Constants.SwerveModule.DRIVE_CONFIG.MAX_CURRENT, 1
            ),
            _modules[NORTH_WEST_IDX].getModuleLocation(),
            _modules[SOUTH_WEST_IDX].getModuleLocation(),
            _modules[SOUTH_EAST_IDX].getModuleLocation(),
            _modules[NORTH_EAST_IDX].getModuleLocation());
        
        _swerveSetpointGenerator = new SwerveSetpointGenerator(
                _config,
                DCMotor.getKrakenX60(1).freeSpeedRadPerSec * Constants.DriveTrain.MOTOR_LOAD_OUTPUT_PERCENTAGE / Constants.SwerveModule.GEAR_RATIO_STEER
        );
        _headingController = new SwerveHeadingController(Constants.UPDATE_PERIOD);

        _poseController = new HolonomicDriveController(
            new PIDController(
                    Constants.DriveTrain.kP, Constants.DriveTrain.kI, Constants.DriveTrain.kD),
            new PIDController(
                    Constants.DriveTrain.kP, Constants.DriveTrain.kI, Constants.DriveTrain.kD),
            new ProfiledPIDController(
                    HEADING_kP,
                    HEADING_kI,
                    HEADING_kD,
                    new TrapezoidProfile.Constraints(
                            Constants.DriveTrain.MAX_ANG_VEL,
                            Constants.DriveTrain.MAX_ANG_ACC)));

        _controlState = ControlState.MANUAL;

        zeroGyroscope();

        readModules();
        setSetpointFromMeasuredModules();
        AutoBuilder.configure(
            _oculusProcessor::getRobotPose, // Robot pose supplier
            _oculusProcessor::setRobotPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getMeasuredChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> setVelocity(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            _config, // The robot configuration
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this // Reference to this subsystem to set requirements
        );

    }

    protected void configureSignalFrequency(double frequency) {
        for (var signal : _signals) {
            signal.setUpdateFrequency(frequency);
        }
    }

    protected void configureModuleControlFrequency(double updateFreqHz) {
        for (var module : _modules) {
            module.setControlRequestUpdateFrequency(updateFreqHz);
        }
    }

    public void readSignals() {
        BaseStatusSignal.waitForAll(0.1, _signals);
        readIMU();
        readModules();
    }

    public double getRotationCorrection() {
        readIMU();
        return _headingController.getRotationCorrection(getHeading());
    }

    public void temporaryDisableHeadingController() {
        _headingController.temporaryDisable();
    }

    public void disableHeadingController() {
        _headingController.disable();
    }

    public void initializeHeadingController() {
        _headingController.goToHeading(getHeading());
    }

    public void goToHeading(Rotation2d heading) {
        _headingController.goToHeading(heading);
    }

    public void setVelocityPose(Pose2d pose) {
        readIMU();
        ChassisSpeeds speeds = _poseController.calculate(
                _oculusProcessor.getRobotPose(), pose, 0.0, _systemIO.heading);
        _headingController.goToHeading(pose.getRotation());
        speeds.omegaRadiansPerSecond = _headingController.getRotationCorrection(getHeading());
        _systemIO.desiredChassisSpeeds = speeds;
    }

    @Override
    public void periodic() {
        _robotState.getWriteLock().lock();
        try {
            updateDesiredStates();
            setModuleStates(_systemIO.setpoint.moduleStates());
        } finally {
            _robotState.getWriteLock().unlock();
        }
    }

    public void setControlState(ControlState state) {
        _controlState = state;
    }

    public ControlState getControlState() {
        return _controlState;
    }

    public void setVelocity(ChassisSpeeds chassisSpeeds) {
        _systemIO.desiredChassisSpeeds = chassisSpeeds;
    }

    public SwerveSetpoint getSetpoint() {
        return _systemIO.setpoint;
    }

    /**
     * yolos the setpoint to 0 velocity. ignores acceleration limits and swerve stuff. just straight vibes
     */
    public void resetVelocityUnprotected() {
        _systemIO.setpoint = new SwerveSetpoint(new ChassisSpeeds(), _systemIO.measuredStates, DriveFeedforwards.zeros(4));
    }

    public boolean isFieldCentric() {
        return _fieldCentric;
    }

    public void setFieldCentric() {
        _fieldCentric = true;
    }

    public void setRobotCentric() {
        _fieldCentric = false;
    }

    public void updateDesiredStates() {
        // This is to avoid skew when driving and rotating.
        Pose2d robotPoseVel = new Pose2d(
                _systemIO.desiredChassisSpeeds.vxMetersPerSecond * Constants.UPDATE_PERIOD,
                _systemIO.desiredChassisSpeeds.vyMetersPerSecond * Constants.UPDATE_PERIOD,
                Rotation2d.fromRadians(
                        _systemIO.desiredChassisSpeeds.omegaRadiansPerSecond * Constants.UPDATE_PERIOD));

        Twist2d twistVel = new Pose2d().log(robotPoseVel);

        ChassisSpeeds updatedChassisSpeeds = new ChassisSpeeds(
                twistVel.dx / Constants.UPDATE_PERIOD,
                twistVel.dy / Constants.UPDATE_PERIOD,
                twistVel.dtheta / Constants.UPDATE_PERIOD);

        // SwerveSetpoint newSetpoint = _swerveSetpointGenerator.generateSetpoint(
        //         _systemIO.setpoint,
        //         updatedChassisSpeeds,
        //         Constants.UPDATE_PERIOD); // FIXME unyolo this - xavier
        // System.out.println("newSetpoint.vx = "+newSetpoint.robotRelativeSpeeds().vxMetersPerSecond);
        // System.out.println("newSetpoint.vy = "+newSetpoint.robotRelativeSpeeds().vyMetersPerSecond);
        // System.out.println("newSetpoint.omega = "+newSetpoint.robotRelativeSpeeds().omegaRadiansPerSecond);
        // for (SwerveModuleState moduleState : newSetpoint.moduleStates()) {
        //     System.out.println("newSetpoint.moduleState = "+moduleState);
        // }

        _systemIO.setpoint = new SwerveSetpoint(updatedChassisSpeeds, _kinematics.toSwerveModuleStates(updatedChassisSpeeds), DriveFeedforwards.zeros(4));

    }

    /* Module Control Start */
    public void readModules() {
        for (int module = 0; module < _modules.length; module++) {
            _systemIO.measuredStates[module] = _modules[module].getState();
            _systemIO.measuredPositions[module] = _modules[module].getPosition();
        }

    }

    public void setModuleStates(SwerveModuleState[] states) {
        for (int module = 0; module < _modules.length; module++) {
            _modules[module].setGoalState(states[module]);
        }
    }

    public SwerveModulePosition[] getSwerveModuleMeasuredPositions() {
        return _systemIO.measuredPositions;
    }

    public void setSetpointFromMeasuredModules() {
        System.arraycopy(_systemIO.measuredStates, 0, _systemIO.setpoint.moduleStates(), 0, _modules.length);
        var newSetpoint = new SwerveSetpoint(getMeasuredChassisSpeeds(), _systemIO.measuredStates, DriveFeedforwards.zeros(4));
        _systemIO.setpoint = newSetpoint;
    }

    public void orientModules(Rotation2d moduleAngle) {
        for (int module = 0; module < _modules.length; module++) {
            _systemIO.setpoint.moduleStates()[module] = new SwerveModuleState(0.0, moduleAngle);
        }
    }
    /* Module Control End */

    public SwerveDriveKinematics getKinematics() {
        return _kinematics;
    }

    @Override
    public void updateDashboard() {
        metric("Swerve State", _controlState.name());
        metric("Current Heading", getHeading().getRadians());
        metric("Current Command", getCurrentCommand() != null ? getCurrentCommand().getName() : "no command");
        for (var module : _modules) {
            module.updateDashboard();
        }
    }

    public boolean isRedAlliance() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            metric("Alliance", alliance.get() == Alliance.Red);
            return alliance.get() == Alliance.Red;
        }
        metric("Alliance", false);
        return false;
    }

    public ChassisSpeeds getMeasuredChassisSpeeds() {
        return _kinematics.toChassisSpeeds(_systemIO.measuredStates);
    }

    public ChassisSpeeds getDesiredChassisSpeeds() {
        return _systemIO.desiredChassisSpeeds;
    }

    public double getSpeed() {
        return Math.hypot(getMeasuredChassisSpeeds().vxMetersPerSecond, getMeasuredChassisSpeeds().vyMetersPerSecond);
    }    
    
    public double getYaw() {
        return _systemIO.heading.getRadians();
    }

    public Rotation2d getHeading() {
        return _systemIO.heading;
    }

    public void zeroGyroscope() {
        _yawOffset = _imu.getYaw().getValue().in(Units.Degrees);
        readIMU();
    }

    public void readIMU() {
        double yawDegrees = BaseStatusSignal.getLatencyCompensatedValue(_imu.getYaw(), _imu.getAngularVelocityZDevice()).in(Units.Degrees);
        double yawAllianceOffsetDegrees = isRedAlliance() ? 180.0 : 0;
        _systemIO.heading = Rotation2d.fromDegrees(yawDegrees - _yawOffset + yawAllianceOffsetDegrees);
    }
}
