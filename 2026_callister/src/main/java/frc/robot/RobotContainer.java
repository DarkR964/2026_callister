package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveWithJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;


public class RobotContainer {
    SwerveSubsystem swerveSubsystem =  new SwerveSubsystem();
    CommandPS5Controller controller = new CommandPS5Controller(0);
     // CommandPS5Controller controller2 = new CommandPS5Controller(1);

     //CommandXboxController controller = new CommandXboxController(0);
    //CommandXboxController controller2 = new CommandXboxController(1);




  public RobotContainer() {
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

   /*  public CommandXboxController getJoystick(){
    return controller;
  } */

  private void configureBindings() {
  controller.cross().onTrue(new InstantCommand(() -> swerveSubsystem.resetHeading(), swerveSubsystem));
  //controller.a().whileTrue(new RunCommand(()-> swerveSubsystem.resetHeading(), swerveSubsystem ));
  }


  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
