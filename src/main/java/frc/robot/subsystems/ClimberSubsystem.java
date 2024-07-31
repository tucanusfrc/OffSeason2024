package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{
    CANSparkMax climber = new CANSparkMax(12, MotorType.kBrushless);
     private RelativeEncoder encoder = climber.getEncoder();
     public SparkPIDController pid;

     public static final double p = 0.6;
     public static final double i = 0;
     public static final double d = 0;
     public static final double iz = 0;
     public static final double ff = 0;
     public static final double max = 1;
     public static final double min = -1;

     public double position;
     public double subindo;

     public void climberMove(double reference){

        pid = climber.getPIDController();
        pid.setP(p);
        pid.setI(i);
        pid.setD(d);
        pid.setIZone(iz);
        pid.setFF(ff);
        pid.setOutputRange(min , max);

        pid.setReference(reference, ControlType.kPosition);
        

    }


    public ClimberSubsystem(){
        climber.restoreFactoryDefaults();
        climber.setIdleMode(IdleMode.kBrake);
}

    public Command Subir(){
        return this.startEnd(
            () -> climberMove(-160),
            () -> climberMove(-160)
        );
    }

    public Command Descer(){
        return this.startEnd(
            () -> climberMove(-0.25),
            () -> climberMove(-0.25)
        );
    }
    public Command Ativar(){
        return this.startEnd(
            () -> subindo = 1,
            () -> subindo = 1
        );
    }

    public Command SetZero(){
        return this.startEnd(
            () -> encoder.setPosition(0),
            () -> encoder.setPosition(0)
        );
    }
    public Command teste(){
        return this.startEnd(
            () -> climber.set(0.25),
            () -> climber.set(0.25)
        );
    }
    public Command stop(){
        return this.startEnd(
            () -> climber.set(0),
            () -> climber.set(0)
        );
    }
    public Command cima(){
        return this.startEnd(
            () -> climber.set(-1),
            () -> climber.set(-1)
        );
    }
    

    @Override
    public void periodic(){
    position = encoder.getPosition();
    SmartDashboard.putNumber("position", position);
    SmartDashboard.putNumber("subindo", subindo);
        
    


    }

}
