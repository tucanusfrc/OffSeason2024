
package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


public class MoveXYHeading extends Command {
  
  double distanceX, distanceY, heading;
  
  SwerveSubsystem swerve;
  boolean finish = false;
  
  double lastTimestamp;
  
  double lastErrorX = 0;
  double lastErrorY = 0;
  double lastErrorH = 0;
  
  double errorSumX = 0;
  double errorSumY = 0;
  double errorSumH = 0;
  public static final double kp = 0.76;
  public static final double ki = 0.36;
  public static final double kd = 0;
  public static final double kpH = 0.008;
  public static final double kiH = 0.004;
  public static final double kdH = 0;
  public static final double MAX_SPEED = 0.2;

  
  public MoveXYHeading(double distanceX, double distanceY, double heading, SwerveSubsystem swerve) {
    this.distanceX = distanceX;
    this.distanceY = distanceY;
    this.heading = heading;
    this.swerve = swerve;
    SmartDashboard.putNumber("Distance Xi", distanceX);
    SmartDashboard.putNumber("Distance Yi", distanceY);
    SmartDashboard.putNumber("Giro", heading);
  }

  @Override
  public void initialize() {
    swerve.resetOdometry();
    swerve.zeroGyro();
    lastTimestamp = Timer.getFPGATimestamp();
    lastErrorX = 0;
    lastErrorY = 0;
    lastErrorH = 0;

  }

  @Override
  public void execute() {
    
    double speedX = 0;
    double speedY = 0;
    double speedH = 0;
    
    
    finish = true;
    if(Math.abs(swerve.getPose().getX())<Math.abs(distanceX))
    {
      finish = false;
    }
    if(Math.abs(swerve.getPose().getY())<Math.abs(distanceY))
    {
      finish = false;
    }
    if(Math.abs(swerve.getYaw().getDegrees())<Math.abs(heading))
    {
      finish = false;
    }

    // Cálculos -PID-
    double sensorX = swerve.getPose().getX();
    double errorX = distanceX - sensorX;
    speedX = kp*errorX;

    double sensorY = swerve.getPose().getY();
    double errorY = distanceY - sensorY;
    speedY = kp*errorY;

    double sensorH = swerve.getYaw().getDegrees();
    double errorH = heading - sensorH;
    speedH = kpH*errorH;

  
    
    double dt = Timer.getFPGATimestamp() - lastTimestamp;
    double errorRateX = (errorX - lastErrorX) / dt;
    double errorRateY = (errorY - lastErrorY) / dt;
    double errorRateH = (errorH - lastErrorH) / dt;

    errorSumX += errorX * dt;
    errorSumY += errorY * dt;
    errorSumH += errorH * dt;

    speedX = kp * errorX + ki * errorSumX + kd * errorRateX;
    speedY = kp * errorY + ki * errorSumY + kd * errorRateY;
    speedH = kpH * errorH + kiH * errorSumH + kdH * errorRateH;
    lastTimestamp = Timer.getFPGATimestamp();
    lastErrorX = errorX;
    lastErrorY = errorY;
    lastErrorH = errorH;

    double xVelocity   = Math.pow(speedX, 3);
    double yVelocity   = Math.pow(speedY, 3);
    double angVelocity = Math.pow(speedH, 3);
    
    swerve.drive(new Translation2d(xVelocity * MAX_SPEED, yVelocity * MAX_SPEED),
                 angVelocity,
                 true ,false);
    
  }

  @Override
  public void end(boolean interrupted) {
    swerve.lock();
  }

  @Override
  public boolean isFinished() {
    return finish;
  }
}