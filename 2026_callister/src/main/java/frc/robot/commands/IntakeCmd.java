package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSub;

public class IntakeCmd extends Command {
  IntakeSub Intake;
  double set;
  
  public IntakeCmd(IntakeSub Intake, double set) {
    this.Intake = Intake;
    this.set = set;
    addRequirements(Intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Intake.IntakeSet(set);
  }

  @Override
  public void end(boolean interrupted) {
    Intake.IntakeSet(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
