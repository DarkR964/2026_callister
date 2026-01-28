package frc.robot.subsystems;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSub extends SubsystemBase {
  SparkFlex Intake = new SparkFlex(31, MotorType.kBrushless);
  
  public IntakeSub() {
    SparkFlexConfig IntakeConfig = new SparkFlexConfig();
    IntakeConfig.idleMode(IdleMode.kBrake);
    IntakeConfig.smartCurrentLimit(40);
    Intake.configure(IntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  
  public void IntakeSet(double power){
    Intake.set(power);
  }
  @Override
  public void periodic() {
    
  }
}
