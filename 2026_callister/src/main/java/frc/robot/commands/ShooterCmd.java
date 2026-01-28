package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSub;

public class ShooterCmd extends Command {

  private final ShooterSub shooter;
  private final double power;

  public ShooterCmd(ShooterSub shooter, double power) {
    this.shooter = shooter;
    this.power = power;
    addRequirements(shooter); // BURADA artık çalışır
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    shooter.shoot(power);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  @Override
  public boolean isFinished() {
    return false; // butona basılı tutuldukça çalışır
  }
}
