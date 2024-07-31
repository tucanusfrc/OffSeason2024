package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private static ShooterSubsystem instance;
    boolean extended = false;
    
    CANSparkMax motor1 = new CANSparkMax(9, MotorType.kBrushless);
    CANSparkMax motor2 = new CANSparkMax(10, MotorType.kBrushless);
    

  public ShooterSubsystem() {
    
    //Limpo qualquer configuração  inicial dos modulos
    motor1.restoreFactoryDefaults();
    motor2.restoreFactoryDefaults();

    //Configuro para  que o  motor se mantenha estatico quando em 0
    motor1.setIdleMode(IdleMode.kCoast);
    motor2.setIdleMode(IdleMode.kCoast);
    
  }

  public static ShooterSubsystem getInstance() {
    if (instance == null) {
      instance = new ShooterSubsystem();
    }
    return instance;
  }

    public void setMotorPower1() {
        motor1.set(1); //1
      
    }
    public void setMotorPower2() {
       motor2.set(1); //1
    }
      public void stopMotor1() {
      motor1.set(0);
    }
    public void stopMotor2() {
       motor2.set(0);
    }
    public void setMotorsReverse() {
        motor1.set(-0.4); //-0.9 //-0.4
        motor2.set(-0.4); // -0.9 //-0.4
    }
    public void stopMotors() {
      motor1.set(0);
      motor2.set(0);
    }
 
    public Command Shoot1(){
      return this.startEnd(
        () -> setMotorPower1(),
        () -> setMotorPower1());
    }
    
    public Command Stop1(){
      return this.startEnd(
        () -> stopMotor1(),
        () -> stopMotor1());
    }

    public Command ShootAuto1(){
      return this.run(
        () -> setMotorPower1());
    }
      public Command ShootAuto2(){
      return this.run(
        () -> setMotorPower2());
    }

    public Command Shoot2(){
      return this.startEnd(
        () -> setMotorPower2(),
        () -> setMotorPower2());
    }

     public Command Stop2(){
      return this.startEnd(
        () -> stopMotor2(),
        () -> stopMotor2());
    }

     public Command Take(){
      return this.startEnd(
        () -> setMotorsReverse(),
        () -> setMotorsReverse());
    }

    public Command StopReversed(){
      return this.startEnd(
        () -> stopMotors(),
        () -> stopMotors());
    }
    @Override
    public void periodic(){
    }
}
