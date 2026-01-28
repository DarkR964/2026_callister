package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSub extends SubsystemBase {

  private final TalonFX shooterMotor = new TalonFX(50);

  public ShooterSub() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    shooterMotor.getConfigurator().apply(config);
  }


  public void shoot(double power) {
    shooterMotor.set(power);
  }

  public void stop() {
    shooterMotor.set(0);
  }
}
