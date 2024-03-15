package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utilities.Constants.HardwareID;
import frc.lib.utilities.Constants.TuningValues;
import frc.robot.commands.Launch.LaunchDirection;

public class LauncherSubsystem extends SubsystemBase{

    TalonFX bottomSpinner, middleSpinner, topSpinner;

    private double upperTargetV = 80;
    private double lowerTargetV = -80;

    VelocityVoltage topVelocity;
    VelocityVoltage middleVelocity;
    VelocityVoltage bottomVelocity;

    public BooleanSupplier isLauncherSpinning = () -> false;

    public LauncherSubsystem() {
        bottomSpinner = new TalonFX(HardwareID.bottomSpinnerMotorCANId);
        middleSpinner = new TalonFX(HardwareID.middleSpinnerMotorCANId);
        topSpinner = new TalonFX(HardwareID.topSpinnerMotorCANId);
        

        bottomSpinner.setNeutralMode(NeutralModeValue.Coast);
        middleSpinner.setNeutralMode(NeutralModeValue.Coast);
        topSpinner.setNeutralMode(NeutralModeValue.Coast);

        topVelocity = new VelocityVoltage(0);
        middleVelocity = new VelocityVoltage(0);
        bottomVelocity = new VelocityVoltage(0);
    }

    public void spinUpperSpinners(boolean ampShot) {
        
        var config = new Slot0Configs();
        config.kV = TuningValues.launcherkV;
        config.kP = TuningValues.launcherkP;
        config.kI = TuningValues.launcherkI;
        config.kD = TuningValues.launcherkD;
        
        if (ampShot) {
            upperTargetV = SmartDashboard.getNumber("upperAmpV", 40);
        }
        else {
            upperTargetV = SmartDashboard.getNumber("upperSpeakerV", 80);
        }

        topSpinner.getConfigurator().apply(config);
        middleSpinner.getConfigurator().apply(config);

        topSpinner.setControl(topVelocity.withVelocity(upperTargetV));
        middleSpinner.setControl(middleVelocity.withVelocity(upperTargetV));
    }

    public void spinLowerSpinners() {
        
        var config = new Slot0Configs();
        config.kV = TuningValues.launcherkV;
        config.kP = TuningValues.launcherkP;
        config.kI = TuningValues.launcherkI;
        config.kD = TuningValues.launcherkD;

        lowerTargetV = SmartDashboard.getNumber("lowerSpeakerV", -80);

        middleSpinner.getConfigurator().apply(config);
        bottomSpinner.getConfigurator().apply(config);

        middleSpinner.setControl(middleVelocity.withVelocity(lowerTargetV));
        bottomSpinner.setControl(bottomVelocity.withVelocity(lowerTargetV));
    }

    public void stopSpinners() {
        bottomSpinner.set(0.0);
        middleSpinner.set(0.0);
        topSpinner.set(0.0);                     

        /*
        bottomSpinner.set(0.0);
        middleSpinner.set(0.0);
        topSpinner.set(0.0);
        */
    }

    public boolean isLauncherUpToSpeed(LaunchDirection direction) {
        double lowerMaxVelocity;
        double upperMaxVelocity;

        if (direction == LaunchDirection.AMP) {
            lowerMaxVelocity = -SmartDashboard.getNumber("upperAmpV", 40);
            upperMaxVelocity = SmartDashboard.getNumber("upperAmpV", 40);
        }
        else {
            lowerMaxVelocity = SmartDashboard.getNumber("lowerSpeakerV", -80);
            upperMaxVelocity = SmartDashboard.getNumber("upperSpeakerV", 80);
        }

        double bottomSpinnerVelocity = bottomSpinner.getVelocity().getValueAsDouble();
        double topSpinnerVelocity = topSpinner.getVelocity().getValueAsDouble();
        double middleSpinnerVelocity = middleSpinner.getVelocity().getValueAsDouble();

        double tolerance = 15;

        if ((bottomSpinnerVelocity < lowerMaxVelocity + tolerance) || (topSpinnerVelocity > upperMaxVelocity - tolerance)) {
          if ((middleSpinnerVelocity < lowerMaxVelocity + tolerance) || (middleSpinnerVelocity > upperMaxVelocity - tolerance)) {
            return true;
          }
        }
 
        return false;
 
    }

    @Override
    public void periodic() {
        telemetry();
    }

    public void telemetry() {
        SmartDashboard.putNumber("Bottom Spinner Speed", bottomSpinner.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Middle Spinner Speed", middleSpinner.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Top Spinner Speed", topSpinner.getVelocity().getValueAsDouble());
    }
}
