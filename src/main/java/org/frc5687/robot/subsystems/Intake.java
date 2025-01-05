package org.frc5687.robot.subsystems;

import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.RobotMap;
import org.frc5687.lib.drivers.OutliersTalon;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends OutliersSubsystem {
    
    private final OutliersTalon _talon;
    
    public Intake(RobotContainer container){
        super(container);
        _talon = new OutliersTalon(RobotMap.CAN.TALONFX.INTAKE, Constants.Intake.CAN_BUS, "Intake");
        _talon.configure(Constants.Intake.CONFIG);
    }

    public void SetSpeed(double intakeSpeed){
        _talon.setPercentOutput(intakeSpeed);
    }

    @Override
    public void updateDashboard() {
        // TODO Auto-generated method stub
        
    }
}
