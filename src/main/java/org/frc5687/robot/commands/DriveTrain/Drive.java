/* Team 5687 (C)2021-2022 */
package org.frc5687.robot.commands.DriveTrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import org.frc5687.lib.math.Vector2d;
import org.frc5687.robot.Constants;
import org.frc5687.robot.OI;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.util.Helpers;

public class Drive extends OutliersCommand {

    private final DriveTrain _driveTrain;
    private final OI _oi;

    private int segmentationArray[] = new int[360 / 5];

    public Drive(DriveTrain driveTrain, OI oi) {
        _driveTrain = driveTrain;
        _oi = oi;

        for (int i = 0; i < segmentationArray.length; i++) {
            double angle = 360 / segmentationArray.length;
            segmentationArray[i] = (int) angle * i;
        }
        addRequirements(_driveTrain);
    }

    @Override
    public void initialize() {
        _driveTrain.goToHeading(_driveTrain.getHeading());
        _driveTrain.setControlState(DriveTrain.ControlState.MANUAL);
    }

    @Override
    public void execute() {

        Vector2d vec = Helpers.axisToSegmentedUnitCircleRadians(
                _oi.getDriveY(), _oi.getDriveX(), segmentationArray);
        double vx;
        double vy;
        double rot = _oi.getRotationX();

        double max_mps = Constants.DriveTrain.MAX_MPS;

        rot = Math.signum(rot) * rot * rot;

        if (rot != 0) {
            _driveTrain.temporaryDisableHeadingController();
        }

        double controllerPower = _driveTrain.getRotationCorrection();

        vx = vec.x() * max_mps;
        vy = vec.y() * max_mps;
        rot = rot * Constants.DriveTrain.MAX_ANG_VEL;

        Rotation2d rotation = _driveTrain.isRedAlliance() ? _driveTrain.getHeading().plus(new Rotation2d(Math.PI)) : _driveTrain.getHeading();

        ChassisSpeeds commandedSpeeds = new ChassisSpeeds(vx, vy, rot + controllerPower);
        if (_driveTrain.isFieldCentric()) {
            commandedSpeeds.toFieldRelativeSpeeds(rotation);
        }
        _driveTrain.setVelocity(commandedSpeeds);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
