package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.CANSparkBase.IdleMode;
//import com.revrobotics.CANSparkLowLevel.MotorType;
//import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AmpSubsystem extends SubsystemBase{
  private static AmpSubsystem instance;
  boolean extended = false;

  int AmpFrente;
  int AmpTras;
  CANSparkMax motor = new CANSparkMax(11, MotorType.kBrushless);
  double velocity = 0.9;

  public AmpSubsystem() {

    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);
    
  }

  public static AmpSubsystem getInstance() {
    if (instance == null) {
      instance = new AmpSubsystem();
    }
    return instance;
  }
      public void Shoot(){
       motor.set(velocity); 
      }

      public void Stop(){
        motor.set(0);
      }
      public Command shootCommand(){
        return this.startEnd(
            () -> motor.set(velocity),
            () -> motor.set(velocity)
            //() -> AmpFrente = 1,
            //() -> AmpFrente = 1
        );
        }

        public Command StopShoot(){
        return this.startEnd(
            () -> motor.set(0),
            () -> motor.set(0)
            //() -> AmpFrente = 0,
            //() -> AmpFrente = 0
        );
        }
        public Command takeCommand(){
            return this.startEnd(
            () -> motor.set(-velocity),
            () -> motor.set(-velocity)
            //() -> AmpTras = 1,
            //() -> AmpTras = 1
            );
        }       

        public Command StopTake(){
            return this.startEnd(
            () -> motor.set(0),
            () -> motor.set(0)
            //() -> AmpTras = 0,
            //() -> AmpTras = 0
            );
        }       
        @Override
    public void periodic(){
    SmartDashboard.putNumber("AmpFrente", AmpFrente);
    SmartDashboard.putNumber("AmpTras", AmpTras);
    }
      }

