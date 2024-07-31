// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimberUp;
import frc.robot.commands.Auto.AmpAuto;
import frc.robot.commands.Auto.Autonomo;
import frc.robot.commands.Auto.AutonomoDireita;
import frc.robot.commands.Auto.AutonomoEsquerda;
import frc.robot.subsystems.AmpSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  private static final String AmpAuto = "AmpAuto";
  private static final String Autonomo = "Autonomo";
  private static final String AutonomoEsquerda = "AutonomoEsquerda";
  private static final String AutonomoDireita = "AutonomoDireita";
 
  private String AutonomoSelect;
  private final SendableChooser<String> Chooser = new SendableChooser<>();
  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandJoystick driverFlightStick = new CommandJoystick(0);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve"));
  public CommandXboxController guitarra = new CommandXboxController(1);

  private ShooterSubsystem Shooter = ShooterSubsystem.getInstance();
  private AmpSubsystem Amp = AmpSubsystem.getInstance();
  private ClimberSubsystem Climber = new ClimberSubsystem();

                                                                         
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    Chooser.setDefaultOption("AmpAuto", AmpAuto);
    Chooser.addOption("Autonomo", Autonomo);
    Chooser.addOption("AutonomoDireita", AutonomoDireita);
    Chooser.addOption("AutonomoEsquerda", AutonomoEsquerda);
    SmartDashboard.putData("AutoSelect", Chooser);
    configureBindings();
     


    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the rotational velocity 
    // buttons are quick rotation positions to different ways to face
    // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
   initializeChooser();
    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation

   Command baseDriveCommand = drivebase.driveCommand(        
      () -> MathUtil.applyDeadband(driverFlightStick.getY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driverFlightStick.getX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> MathUtil.applyDeadband(driverFlightStick.getRawAxis(4)*.85,.1));
    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive


    drivebase.setDefaultCommand(
       baseDriveCommand);
  }

  private void configureBindings()
  {
    driverFlightStick.button(1).whileTrue(new InstantCommand(drivebase::zeroGyro));
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    (guitarra.b())
    .and(guitarra.leftStick().negate())
    .onTrue(Shooter.Shoot2());
    guitarra.pov(180).onFalse(Shooter.Stop1());
    guitarra.b().onFalse(Shooter.Stop2());
    guitarra.pov(0).onFalse(Shooter.Stop1());
    guitarra.leftStick().negate();
    guitarra.pov(0).onTrue(Shooter.Shoot1());
    guitarra.pov(180).onTrue(Shooter.Shoot1());

    
    guitarra.x().and(guitarra.leftStick().negate())
    .onTrue(Amp.shootCommand());
    guitarra.leftBumper().and(guitarra.leftStick().negate())
    .onTrue(Amp.takeCommand());
    guitarra.x().onFalse(Amp.StopShoot()).and(guitarra.leftStick().negate());
    guitarra.leftBumper().onFalse(Amp.StopTake()).and(guitarra.leftStick().negate());


    guitarra.a().and(guitarra.leftStick().negate())
    .onTrue(Shooter.Shoot1());
    guitarra.y().and(guitarra.leftStick().negate())
    .onTrue(Shooter.Take());

    guitarra.y().onFalse(Shooter.StopReversed()).and(guitarra.leftStick().negate());
    guitarra.a().onFalse(Shooter.Stop1());
    guitarra.a().and(guitarra.leftStick()).onTrue(new ClimberUp(Climber));
    guitarra.b().and(guitarra.leftStick()).onTrue(Climber.Descer());
    guitarra.y().and(guitarra.leftStick()).onTrue(Climber.SetZero());
    guitarra.x().and(guitarra.leftStick()).onTrue(Climber.teste());
    guitarra.leftBumper().and(guitarra.leftStick()).onTrue(Climber.cima());
    guitarra.leftStick().onFalse(Climber.stop());
    
  }
private void initializeChooser(){



}
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomuCommand(){
    AutonomoSelect = Chooser.getSelected();

    if (AutonomoSelect == Autonomo) {
      return new Autonomo(drivebase);
    } 
     if (AutonomoSelect == AmpAuto) {
      return new AmpAuto(drivebase);
    }
    if (AutonomoSelect == AutonomoDireita) {
      return new AutonomoDireita(drivebase);
    }
    if (AutonomoSelect == AutonomoEsquerda) {
      return new AutonomoEsquerda(drivebase);
    }
    else {
      return null;
    }   
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    
    drivebase.setMotorBrake(brake);
  }

public void autonomousPeriodic(){
}
}
