/* Team 5687 (C)2021-2022 */
package org.frc5687.robot;

import java.io.IOException;
import java.util.Optional;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.commands.DriveTrain.Drive;
import org.frc5687.robot.subsystems.DriveTrain;
import org.frc5687.robot.subsystems.OutliersSubsystem;
import org.frc5687.robot.util.OculusProcessor;
import org.frc5687.robot.util.OutliersContainer;
import org.frc5687.robot.util.PhotonProcessor;
import org.json.simple.parser.ParseException;
import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RobotContainer extends OutliersContainer {
    private OI _oi;
    // private SendableChooser<Command> _autoChooser;
    private Pigeon2 _imu;
    private Robot _robot;
    private DriveTrain _driveTrain;
    private Field2d _field;

    private RobotState _robotState = RobotState.getInstance();

    public RobotContainer(Robot robot, IdentityMode identityMode) {
        super(identityMode);
        _robot = robot;
    }

    public void init() {
        Thread.currentThread().setPriority(Thread.MAX_PRIORITY);
        Thread.currentThread().setName("Robot Thread");
        _oi = new OI();

        _field = new Field2d();

        // configure pigeon
        _imu = new Pigeon2(RobotMap.CAN.PIGEON.PIGEON, "CANivore");
        var pigeonConfig = new Pigeon2Configuration();
        pigeonConfig.withMountPose(new MountPoseConfigs().withMountPoseRoll(Math.PI)); // I'm not actually sure if this should be pitch or roll... if this leads to issues later on blame me (xavier)
        _imu.getConfigurator().apply(pigeonConfig);

        _driveTrain = new DriveTrain(this, _imu);

        // _photonProcessor = new PhotonProcessor(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField());
        // Grab instance such that we can initalize with drivetrain and processor
        _robotState.initializeRobotState(_driveTrain);

        setDefaultCommand(_driveTrain, new Drive(_driveTrain, _oi));

        registerNamedCommands();
        // _autoChooser = AutoBuilder.buildAutoChooser("");

        SmartDashboard.putData(_field);
        // SmartDashboard.putData("Auto Chooser", _autoChooser);

        _oi.initializeButtons(_driveTrain, _robotState);

        PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);
    }

    public void periodic() {
        _robotState.periodic();
        _field.setRobotPose(_robotState.getEstimatedPose());

        var oculus = _field.getObject("oculus");

        oculus.setPose(_driveTrain._oculusProcessor.getRobotPose());
        SmartDashboard.putNumber("oculus x", _driveTrain._oculusProcessor.getRobotPose().getX());
        SmartDashboard.putNumber("oculus y", _driveTrain._oculusProcessor.getRobotPose().getY());

        // _field.getObject("futurePose").setPose(_robotState.calculateAdjustedRPMAndAngleToTargetPose());
        // Optional<Pose2d> optionalClosestNote = _robotState.getClosestNote();
        // if (optionalClosestNote.isPresent()) {
        //     Pose2d notePose = optionalClosestNote.get();
        //     _field.getObject("note").setPose(notePose);
        // } else {
        //     // TODO remove note from glass
        // }

        SmartDashboard.putData(_field);
    }

    public void disabledPeriodic() {
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void teleopInit() {
        _driveTrain.resetVelocityUnprotected(); // this is important otherwise the robot will start with velocity
        _robotState.useTeleopStandardDeviations();
    }

    @Override
    public void autonomousInit() {
        _driveTrain.resetVelocityUnprotected(); // this is important otherwise the robot will start with velocity
        _robotState.useAutoStandardDeviations();
    }

    private void setDefaultCommand(OutliersSubsystem subSystem, OutliersCommand command) {
        if (subSystem == null || command == null) {
            return;
        }
        CommandScheduler s = CommandScheduler.getInstance();
        s.setDefaultCommand(subSystem, command);
    }

    public Command getAutoCommand() {
        // Follow a path
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path;
        try {
            path = PathPlannerPath.fromPathFile("Example Path");
            return AutoBuilder.followPath(path);

        } catch (FileVersionException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (ParseException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        
        // // Create a path following command using AutoBuilder. This will also trigger
        // event markers.

        // return _autoChooser.getSelected();

        /*
         * Warning
         * This method will load all autos in the deploy directory. Since the deploy
         * process does not automatically clear the deploy directory, old auto files
         * that have since been deleted from the project could remain on the RIO,
         * therefore being added to the auto chooser.
         * To remove old options, the deploy directory will need to be cleared manually
         * via SSH, WinSCP, reimaging the RIO, etc.
         */

        return new WaitCommand(15.0);
    }

    public Optional<Rotation2d> getRotationTargetOverride() {
        // // Some condition that should decide if we want to override rotation
        // if (_shooter.getAutoShootFlag()) {
        //     // Return an optional containing the rotation override (this should be a field
        //     // relative rotation)
        //     return Optional.of(new Rotation2d(_robotState.getDistanceAndAngleToSpeaker().getSecond()));
        // } else {
        //     // return an empty optional when we don't want to override the path's rotation
            return Optional.empty();
        // }

    }

    public void registerNamedCommands() {
    }
}