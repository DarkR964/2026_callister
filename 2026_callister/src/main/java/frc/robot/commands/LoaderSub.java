package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LoaderSub extends SubsystemBase {

  private final TalonFX loaderMotor = new TalonFX(60);

  public LoaderSub() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    loaderMotor.getConfigurator().apply(config);
  }

  /** Loader çalıştır */
  public void load(double power) {
    loaderMotor.set(power);
  }

  /** Loader durdur */
  public void stop() {
    loaderMotor.set(0);
  }
}
