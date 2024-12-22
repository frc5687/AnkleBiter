package org.frc5687.robot.commands.DriveTrain;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.DriveTrain;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DriveWithSpeeds extends OutliersCommand{

    private DriveTrain _driveTrain;
    private double _vx;
    private double _vy;
    private double _omega;
    
    public DriveWithSpeeds(DriveTrain driveTrain, double vx, double vy, double omega) {
        _driveTrain = driveTrain;
        _vx = vx;
        _vy = vy;
        _omega = omega;
    }

    @Override
    public void execute() {
        super.execute();
        ChassisSpeeds commandedSpeeds = new ChassisSpeeds(_vx, _vy, _omega);
        commandedSpeeds.toRobotRelativeSpeeds(_driveTrain.getHeading());
        _driveTrain.setVelocity(commandedSpeeds);
    }

    
}
