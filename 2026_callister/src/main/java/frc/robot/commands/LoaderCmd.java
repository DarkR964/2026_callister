package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LoaderSub;

public class LoaderCmd extends Command {

  private final LoaderSub loader;
  private final double power;

  public LoaderCmd(LoaderSub loader, double power) {
    this.loader = loader;
    this.power = power;
    addRequirements(loader);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    loader.load(power);
  }

  @Override
  public void end(boolean interrupted) {
    loader.stop();
  }

  @Override
  public boolean isFinished() {
    return false; // butona basılı tutuldukça çalışır
  }
}
