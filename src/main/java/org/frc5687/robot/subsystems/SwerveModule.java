/* Team 5687  */
package org.frc5687.robot.subsystems;

import static org.frc5687.robot.Constants.SwerveModule.DRIVE_CONTROLLER_CONFIG;
import static org.frc5687.robot.Constants.SwerveModule.WHEEL_RADIUS;
import org.frc5687.lib.drivers.OutliersTalon;
import org.frc5687.robot.Constants;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {

    public static class ModuleConfiguration {
        public String moduleName = "";
        public Translation2d position = new Translation2d();
        public double encoderOffset = 0.0;
        public String canBus = "rio";
    }

    private final OutliersTalon _driveMotor;
    private final OutliersTalon _steeringMotor;
    private final CANcoder _encoder;

    private Translation2d _modulePosition;

    private final BaseStatusSignal[] _signals = new BaseStatusSignal[4];
    private StatusSignal<Double> _driveVelocityRotationsPerSec;
    private StatusSignal<Double> _drivePositionRotations;
    private StatusSignal<Double> _steeringVelocityRotationsPerSec;
    private StatusSignal<Double> _steeringPositionRotations;

    private VelocityTorqueCurrentFOC _velocityTorqueCurrentFOC;
    private MotionMagicExpoTorqueCurrentFOC _angleTorqueExpo;
    private MotionMagicTorqueCurrentFOC _angleTorque;

    private SwerveModulePosition _internalState = new SwerveModulePosition();

    private final double _rotPerMet;

    private final String _moduleName;
    private SwerveModuleState _goalState;

    public SwerveModule(
            SwerveModule.ModuleConfiguration config,
            int steeringMotorID,
            int driveMotorID,
            int encoderPort) {

        // Driving Torque Velocity
        _velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0).withOverrideCoastDurNeutral(true);
        // Steering Torque Position with exponential curve
        _angleTorqueExpo = new MotionMagicExpoTorqueCurrentFOC(0);
        _angleTorque = new MotionMagicTorqueCurrentFOC(0).withOverrideCoastDurNeutral(true);
        /* Motor Setup */
        _driveMotor = new OutliersTalon(driveMotorID, config.canBus, "Drive");
        _driveMotor.configure(Constants.SwerveModule.DRIVE_CONFIG);
        _driveMotor.configureClosedLoop(Constants.SwerveModule.DRIVE_CONTROLLER_CONFIG);

        _steeringMotor = new OutliersTalon(steeringMotorID, config.canBus, "Steer");
        _steeringMotor.configure(Constants.SwerveModule.STEER_CONFIG);
        _steeringMotor.configureClosedLoop(Constants.SwerveModule.STEER_CONTROLLER_CONFIG);

        _encoder = new CANcoder(encoderPort, config.canBus);
        CANcoderConfiguration CANfig = new CANcoderConfiguration();
        // set units of the CANCoder to radians, with velocity being radians per second
        CANfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        CANfig.MagnetSensor.MagnetOffset = config.encoderOffset;
        CANfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        _encoder.getConfigurator().apply(CANfig);

        FeedbackConfigs feedback = new FeedbackConfigs();
        feedback.FeedbackRemoteSensorID = encoderPort;
        feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        feedback.RotorToSensorRatio = Constants.SwerveModule.GEAR_RATIO_STEER;

        _steeringMotor.configureFeedback(feedback);

        _modulePosition = config.position;
        _rotPerMet = 1 / (2 * Math.PI * Constants.SwerveModule.WHEEL_RADIUS);

        initializeSignals();

        // _controlState = ControlState.OFF;
        _moduleName = config.moduleName;
        System.out.println(_moduleName + " Module has been constructed!!");
    }

    private void initializeSignals() {
        _drivePositionRotations = _driveMotor.getPosition();
        _driveVelocityRotationsPerSec = _driveMotor.getVelocity();
        _steeringPositionRotations = _encoder.getPosition();
        _steeringVelocityRotationsPerSec = _encoder.getVelocity();

        _driveMotor.getFault_Hardware().setUpdateFrequency(4, 0.04); // not sure if this is used?? - xavier
        _driveVelocityRotationsPerSec.setUpdateFrequency(1 / 250);
        _drivePositionRotations.setUpdateFrequency(1 / 250);

        _steeringMotor.getFault_Hardware().setUpdateFrequency(4, 0.04); // not sure if this is used?? - xavier
        _steeringVelocityRotationsPerSec.setUpdateFrequency(1 / 250);
        _steeringPositionRotations.setUpdateFrequency(1 / 250);

        _signals[0] = _driveVelocityRotationsPerSec;
        _signals[1] = _drivePositionRotations;
        _signals[2] = _steeringVelocityRotationsPerSec;
        _signals[3] = _steeringPositionRotations;
    }

    public void setControlRequestUpdateFrequency(double updateFreqHz) {
        _velocityTorqueCurrentFOC.UpdateFreqHz = updateFreqHz;
        _angleTorque.UpdateFreqHz = updateFreqHz;
        _angleTorqueExpo.UpdateFreqHz = updateFreqHz;
    }

    public void setGoalState(SwerveModuleState state) {
        // optimize goal state
        Rotation2d currentAngle = getCanCoderAngle();
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, currentAngle);
        // https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html#cosine-compensation
        optimizedState.speedMetersPerSecond *= optimizedState.angle.minus(currentAngle).getCos();
        _goalState = optimizedState;

        // send motor setpoints
        _driveMotor.configureClosedLoop(DRIVE_CONTROLLER_CONFIG);
        _driveMotor.setControl(_velocityTorqueCurrentFOC.withVelocity(optimizedState.speedMetersPerSecond * Constants.SwerveModule.GEAR_RATIO_DRIVE * _rotPerMet));
        _steeringMotor.setPositionVoltage(optimizedState.angle.getRotations());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getWheelVelocityMetersPerSecond(), getCanCoderAngle());
    }

    public SwerveModulePosition getPosition() {
        BaseStatusSignal.refreshAll(_drivePositionRotations, _driveVelocityRotationsPerSec, _steeringPositionRotations, _steeringVelocityRotationsPerSec);
        double currentEncoderRotations = BaseStatusSignal.getLatencyCompensatedValue(_drivePositionRotations, _driveVelocityRotationsPerSec);
        double distanceMeters = currentEncoderRotations * 2.0 * Math.PI / Constants.SwerveModule.GEAR_RATIO_DRIVE * Constants.SwerveModule.WHEEL_RADIUS ;
        double angle_rot = BaseStatusSignal.getLatencyCompensatedValue(_steeringPositionRotations, _steeringVelocityRotationsPerSec);

        _internalState.distanceMeters = distanceMeters;
        _internalState.angle = Rotation2d.fromRotations(angle_rot);

        return _internalState;
    }

    private Rotation2d getCanCoderAngle() {
        _steeringPositionRotations.refresh();
        return Rotation2d.fromRotations(_encoder.getAbsolutePosition().getValue());
    }

    public double getDriveRPM() {
        return OutliersTalon.rotationsPerSecToRPM(_driveVelocityRotationsPerSec.getValue(), 1.0);
    }

    public double getTurningRPM() {
        return OutliersTalon.rotationsPerSecToRPM(_steeringVelocityRotationsPerSec.getValue(), 1.0);
    }

    public double getWheelVelocityMetersPerSecond() {
        return getWheelAngularVelocity() * Constants.SwerveModule.WHEEL_RADIUS;
    }

    public double getWheelAngularVelocity() {
        return Units.rotationsPerMinuteToRadiansPerSecond(getDriveRPM() / Constants.SwerveModule.GEAR_RATIO_DRIVE);
    }

    public Translation2d getModuleLocation() {
        return _modulePosition;
    }

    public double getWheelDistanceMeters() {
        return getWheelDistanceRadians() * WHEEL_RADIUS;
    }

    public double getWheelDistanceRadians() {
        return _drivePositionRotations.getValue() * (Math.PI * 2.0) / (Constants.SwerveModule.GEAR_RATIO_DRIVE);
    }

    public BaseStatusSignal[] getSignals() {
        return _signals;
    }

    public void updateDashboard() {
        SmartDashboard.putNumber(_moduleName + "/goalSteerAngleRadians", _goalState.angle.getRadians());
        SmartDashboard.putNumber(_moduleName + "/steerAngleRadians", _internalState.angle.getRadians());
        SmartDashboard.putNumber(_moduleName + "/goalSpeedMetersPerSecond", _goalState.speedMetersPerSecond);
        SmartDashboard.putNumber(_moduleName + "/speedMetersPerSecond", getWheelVelocityMetersPerSecond());
    }
}
