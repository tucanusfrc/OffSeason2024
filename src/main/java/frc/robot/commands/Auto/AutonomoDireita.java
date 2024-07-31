
package frc.robot.commands.Auto;


import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.swervedrive.auto.MoveXYHeading;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomoDireita extends SequentialCommandGroup {
  ShooterSubsystem shooter = ShooterSubsystem.getInstance();
  
  public AutonomoDireita (SwerveSubsystem drivebase){
  {
    addRequirements(drivebase);
    addRequirements(shooter);

    addCommands(
      new InstantCommand(()->shooter.setMotorPower2(),shooter),
      new WaitCommand(3),
      new InstantCommand(()->shooter.setMotorPower1(),shooter),
      new WaitCommand(1),
      new InstantCommand(()->shooter.stopMotor1(),shooter),
      new InstantCommand(()->shooter.stopMotor2(),shooter),
      new WaitCommand(2),
      new MoveXYHeading(0, 0.5, 45, drivebase),
      new MoveXYHeading(-2, 0, 0, drivebase));


  // para andar 2metros x = 2.1
  // para andar 2metros y = 2.2
                                        
    }
  }
}

// Outros tipos de comandos para serem usados, (lembre-se das importações):

/* Permite que os comandos sejam executados até um determinado tempo limite:
    new DeadlineGroup(
        new CLASSE-USADA(driveSubsystem).withTimeout(5),
        new CLASSE-USADA(driveSubsystem).withTimeout(3)
    ),
 */

/* Executa todos os comandos paralelamente e avança quando todos terminam:

    new ParallelCommandGroup(
        new CLASSE-USADA(driveSubsystem),
        new CLASSE-USADA(driveSubsystem)
    ),

 */

 /* Um comando que executa instantaneamente quando agendado. 
    É útil para tarefas simples ou ações que não exigem uma execução contínua:
  
    new InstantCommand(() -> SmartDashboard.putString("Status", "Acabou o autônomo!")),
    new InstantCommand(() -> shooter.shoot(), shooter)
    new InstantCommand(() -> led.setColor(Color.BLUE), led)
    new InstantCommand(() -> System.out.println("Comando Instantâneo Executado!"))
    new InstantCommand(() -> armMotor.setPosition(0), armMotor)

  */