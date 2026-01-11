package frc.robot.commands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveWithJoystickCmd extends Command {
  private final DoubleSupplier xSupplier, ySupplier,rotationSupplier;
  private final BooleanSupplier fieldOrientedSupplier;
  private final SwerveSubsystem swerveSubsystem;
  private final SlewRateLimiter xLimiter,yLimiter,rotationLimiter;
  
  public DriveWithJoystickCmd(SwerveSubsystem requiredSubsystem, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier,BooleanSupplier fieldOriented ) {
    this.xSupplier = xSupplier;
    this.fieldOrientedSupplier = fieldOriented;
    this.ySupplier = ySupplier;
    this.rotationSupplier = thetaSupplier;
    this.swerveSubsystem = requiredSubsystem;
    xLimiter = new SlewRateLimiter(Constants.SwerveSubsystemConstants.kSlewRateXLimit);
    yLimiter = new SlewRateLimiter(Constants.SwerveSubsystemConstants.kSlewRateYLimit);
    rotationLimiter =  new SlewRateLimiter(Constants.SwerveSubsystemConstants.kSlewRateOmegaLimit);
    addRequirements(swerveSubsystem); 
  }

  
  @Override
  public void initialize() {}

  
  @Override
  public void execute() {
    double xSpeed = xSupplier.getAsDouble();
    double ySpeed = ySupplier.getAsDouble();
    double omega = rotationSupplier.getAsDouble();


    xSpeed = Math.abs(xSpeed) > 0.05 ? xSpeed:0;
    ySpeed = Math.abs(ySpeed) > 0.05 ? ySpeed:0;
    omega = Math.abs(omega) > 0.05 ? omega:0;
    
    xSpeed = xLimiter.calculate(xSpeed) * Constants.ModuleConstants.kTeleopMaxLinearSpeed;
    ySpeed = yLimiter.calculate(ySpeed) * Constants.ModuleConstants.kTeleopMaxLinearSpeed;
    omega = rotationLimiter.calculate(omega) * Constants.ModuleConstants.kTeleopMaxAngularSpeed;

    ChassisSpeeds speeds;

    if(fieldOrientedSupplier.getAsBoolean()){
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omega, swerveSubsystem.getRotation2D());
    }
    else{
      speeds = new ChassisSpeeds(xSpeed, ySpeed, omega);
    }
    SwerveModuleState[] states = Constants.SwerveSubsystemConstants.kinematics.toSwerveModuleStates(speeds);

    swerveSubsystem.setStates(states);
  }

  
  @Override
  public void end(boolean interrupted) {}

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
