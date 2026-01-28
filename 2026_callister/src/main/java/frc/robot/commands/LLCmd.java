  package frc.robot.commands;
  import java.security.cert.X509CRL;

  import edu.wpi.first.math.controller.PIDController;
  import edu.wpi.first.math.kinematics.ChassisSpeeds;
  import edu.wpi.first.math.kinematics.SwerveModuleState;
  import edu.wpi.first.wpilibj2.command.Command;
  import frc.robot.Constants;
  import frc.robot.subsystems.LLSub;
  import frc.robot.subsystems.SwerveSubsystem;


  public class LLCmd extends Command {
    LLSub LL;
    SwerveSubsystem swerveSubsystem;
    
    public LLCmd(LLSub limelight, SwerveSubsystem swerveSubsystem) {
      this.LL = limelight;
      this.swerveSubsystem = swerveSubsystem;
      addRequirements(limelight, swerveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
      double offsetX = LL.getLimelightTargetOffsetX();
      double offsetY = LL.getLimelightTargetOffsetY();
      double offsetA = LL.getLimelightTargetOffsetA();
      double target = LL.getLimelightTargetValid();
      //double sppedx = 1;
      //double speedy = offsetY * 0.5;
      //double turn = offsetX * 0.2;
      
      if(target == 1 && offsetA < 20){
        double forwardSpeed = 0;
        double strafeSpeed = 0;
        double rotateSpeed = 0;
        if (offsetX > 0) {
          forwardSpeed = 2;
          
          strafeSpeed = forwardSpeed * 0.3;
          
          rotateSpeed = offsetX * 0.1;   // Strafe sağa hareket
        }
        // Eğer hedef soldaysa, sola kaydır
        else if (offsetX < 0) {
          forwardSpeed = 2;
          
          strafeSpeed = forwardSpeed * -0.3; 
          
          rotateSpeed = offsetX * 0.1;   // Strafe sola hareket
        }
        // Eğer hedef ortadaysa, sadece ileri/geri hareket yap
        else {
          // Hedefin uzaklığına göre yaklaşma/uzaklaşma
          if (offsetA < 0.1) {  // Hedef çok yakınsa, geri git
            forwardSpeed = 0.5;  // Geri git
          } else if (offsetA > 0.2) {  // Hedef uzaksa, ileri git
            forwardSpeed = -0.5;  // İleri git
          } else {
            forwardSpeed = 0;  // Dur
          }
        }
        ChassisSpeeds speeds = new ChassisSpeeds(forwardSpeed, strafeSpeed, rotateSpeed);
        SwerveModuleState[] states = Constants.SwerveSubsystemConstants.kinematics.toSwerveModuleStates(speeds);
        swerveSubsystem.setStates(states);
      }
      else{
        ChassisSpeeds fSpeeds = new ChassisSpeeds(0, 0, 0);
        swerveSubsystem.setStates(Constants.SwerveSubsystemConstants.kinematics.toSwerveModuleStates(fSpeeds));
      }
      
    }

    
    @Override
    public void end(boolean interrupted) {
      ChassisSpeeds fSpeeds = new ChassisSpeeds(0, 0, 0);
      swerveSubsystem.setStates(Constants.SwerveSubsystemConstants.kinematics.toSwerveModuleStates(fSpeeds));
    }

    
    @Override
    public boolean isFinished() {
      return false;
    }
  }
