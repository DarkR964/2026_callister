package frc.robot.subsystems;
import static edu.wpi.first.units.Units.Degrees;

import java.util.Optional;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LLSub extends SubsystemBase {
  NetworkTable limelightTable;
  public LLSub() {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  }

  public double getLimelightTargetOffsetX(){
    return limelightTable.getEntry("tx").getDouble(0);
  }

  public double getLimelightTargetOffsetY(){
    return limelightTable.getEntry("ty").getDouble(0);
  }

  public double getLimelightTargetOffsetA(){
    return limelightTable.getEntry("ta").getDouble(0);
  }

  public double getLimelightTargetValid(){
    return limelightTable.getEntry("tv").getDouble(0);
  }
  

  @Override
  public void periodic() {

  }
}
