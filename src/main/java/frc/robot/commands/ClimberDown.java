package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberDown extends Command{
    ClimberSubsystem climber;
    public ClimberDown(ClimberSubsystem ClimberSubsystem){
        climber = ClimberSubsystem;
        addRequirements(climber);
    }
    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
            climber.climberMove(-30);
        
    }

    @Override
    public void end (boolean interrupted){
        
    }
}
