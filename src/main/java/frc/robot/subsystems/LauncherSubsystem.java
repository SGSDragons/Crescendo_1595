package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utilities.Constants.HardwareID;
import frc.lib.utilities.Constants.Keys;
import frc.lib.utilities.Constants.TuningValues;
import frc.robot.commands.Launch.LaunchDirection;

public class LauncherSubsystem extends SubsystemBase{

    TalonFX bottomSpinner, middleSpinner, topSpinner;

    private double upperSpinnerTargetV, lowerSpinnerTargetV;

    VelocityVoltage upperSpinnerVelocity, middleSpinnerVelocity, bottomSpinnerVelocity;

    public BooleanSupplier isLauncherSpinning = () -> false;

    public LauncherSubsystem() {
        bottomSpinner = new TalonFX(HardwareID.bottomSpinnerMotorCANId);
        middleSpinner = new TalonFX(HardwareID.middleSpinnerMotorCANId);
        topSpinner = new TalonFX(HardwareID.topSpinnerMotorCANId);
        

        bottomSpinner.setNeutralMode(NeutralModeValue.Coast);
        middleSpinner.setNeutralMode(NeutralModeValue.Coast);
        topSpinner.setNeutralMode(NeutralModeValue.Coast);

        upperSpinnerVelocity = new VelocityVoltage(0);
        middleSpinnerVelocity = new VelocityVoltage(0);
        bottomSpinnerVelocity = new VelocityVoltage(0);
    }

    public void spinUpperSpinners(boolean ampShot) {
        
        var config = new Slot0Configs();
        config.kV = TuningValues.launcherkV;
        config.kP = TuningValues.launcherkP;
        config.kI = TuningValues.launcherkI;
        config.kD = TuningValues.launcherkD;
        
        upperSpinnerTargetV = ampShot ? Preferences.getDouble(Keys.ampV, 40) : Preferences.getDouble(Keys.speakerHighAimV, 65);
        

        topSpinner.getConfigurator().apply(config);
        middleSpinner.getConfigurator().apply(config);

        topSpinner.setControl(upperSpinnerVelocity.withVelocity(upperSpinnerTargetV));
        middleSpinner.setControl(middleSpinnerVelocity.withVelocity(upperSpinnerTargetV));
    }

    public void spinLowerSpinners() {
        
        var config = new Slot0Configs();
        config.kV = TuningValues.launcherkV;
        config.kP = TuningValues.launcherkP;
        config.kI = TuningValues.launcherkI;
        config.kD = TuningValues.launcherkD;

        lowerSpinnerTargetV =  Preferences.getDouble(Keys.speakerLowAimV, -80);

        middleSpinner.getConfigurator().apply(config);
        bottomSpinner.getConfigurator().apply(config);

        middleSpinner.setControl(middleSpinnerVelocity.withVelocity(lowerSpinnerTargetV));
        bottomSpinner.setControl(bottomSpinnerVelocity.withVelocity(lowerSpinnerTargetV));
    }

    public void stopSpinners() {
        bottomSpinner.set(0.0);
        middleSpinner.set(0.0);
        topSpinner.set(0.0);                     
    }

    public boolean isLauncherUpToSpeed(LaunchDirection direction) {
        double lowerMaxVelocity;
        double upperMaxVelocity;

        if (direction == LaunchDirection.AMP) {
            lowerMaxVelocity = -Preferences.getDouble(Keys.ampV, 10);
            upperMaxVelocity = Preferences.getDouble(Keys.ampV, 10);
        }
        else {
            lowerMaxVelocity = Preferences.getDouble(Keys.speakerLowAimV, -80);
            upperMaxVelocity = Preferences.getDouble(Keys.speakerHighAimV, 80);
        }

        double bottomSpinnerVelocity = bottomSpinner.getVelocity().getValueAsDouble();
        double topSpinnerVelocity = topSpinner.getVelocity().getValueAsDouble();
        double middleSpinnerVelocity = middleSpinner.getVelocity().getValueAsDouble();

        double tolerance = Preferences.getDouble(Keys.launcherTolerance, 5);

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
