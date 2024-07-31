package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberUp extends Command {

    ClimberSubsystem climber;
    public ClimberUp(ClimberSubsystem ClimberSubsystem){
        climber = ClimberSubsystem;
        addRequirements(climber);
    }
    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
            climber.climberMove(-160);
        
    }

    @Override
    public void end (boolean interrupted){
        
    }
}
