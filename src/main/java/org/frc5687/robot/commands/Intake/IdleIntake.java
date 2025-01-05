package org.frc5687.robot.commands.Intake;

import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;

public class IdleIntake extends OutliersCommand{
    private Intake _intake;
    public IdleIntake(Intake intake) {
        _intake = intake;
        addRequirements(_intake);
    }

    @Override
    public void initialize(){
        _intake.SetSpeed(0);
    }

    @Override
    public void end(boolean interrupted){
        _intake.SetSpeed(0);
    }
}