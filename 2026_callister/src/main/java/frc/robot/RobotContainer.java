package frc.robot;
import java.nio.file.Path;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindThenFollowPath;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveWithJoystickCmd;

import frc.robot.commands.IntakeCmd;
import frc.robot.commands.LLCmd;
import frc.robot.commands.LoaderCmd;
import frc.robot.commands.ShooterCmd;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.LLSub;
import frc.robot.subsystems.LoaderSub;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  ShooterSub Shooter = new ShooterSub();
  LoaderSub Loader = new LoaderSub();

  IntakeSub Intake = new IntakeSub();
  LLSub Limelight = new LLSub();

  SwerveSubsystem swerveSubsystem =  new SwerveSubsystem();
  CommandPS5Controller controller = new CommandPS5Controller(0);
  CommandPS5Controller controller2 = new CommandPS5Controller(1);

  //CommandXboxController controller = new CommandXboxController(0);
  //CommandXboxController controller2 = new CommandXboxController(1);


  public RobotContainer() {
    NamedCommands.registerCommand("IntakeMotor", new IntakeCmd(Intake, 0.2));

    

    swerveSubsystem.setDefaultCommand(new DriveWithJoystickCmd(swerveSubsystem, () -> -controller.getLeftY() * 1 , () -> controller.getLeftX() * 1 , ()-> controller.getRightX() * 1 , ()-> true ));
    configureBindings();
  }

  public void publishJoystickValues() {
    SmartDashboard.putNumber("Joystick X", controller.getLeftX());
    SmartDashboard.putNumber("Joystick Y", controller.getLeftY());
    SmartDashboard.putNumber("Joystick XX", controller.getRightY());
  }

  public CommandPS5Controller getJoystick(){
    return controller;
  } 

  /* public CommandXboxController getJoystick() {
    return controller;
  } */

  private void configureBindings() {
    controller2.R2().whileTrue(new ShooterCmd(Shooter, 1).alongWith(new LoaderCmd(Loader, -0.5)));
    
    controller2.L2().whileTrue(new ShooterCmd(Shooter, 1));

    controller2.R1().whileTrue(new LoaderCmd(Loader, 0.35));
    controller2.R2().whileTrue(new LoaderCmd(Loader, -0.35));

    controller.R2().whileTrue(new IntakeCmd(Intake, 0.35));
    controller.L2().whileTrue(new IntakeCmd(Intake, -0.3));

    controller.R1().whileTrue(new LLCmd(Limelight, swerveSubsystem));
    controller.cross().onTrue(new InstantCommand(() -> swerveSubsystem.resetHeading(), swerveSubsystem));
  }
  
  Command deneme = new PathPlannerAuto("deneme");

public Command getAutonomousCommand() {
  return new PathPlannerAuto("deneme");
}
}
