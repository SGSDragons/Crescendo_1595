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

    private double topSpinnerTargetV, bottomSpinnerTargetV, middleSpinnerTargetV;

    VelocityVoltage topSpinnerVelocity, middleSpinnerVelocity, bottomSpinnerVelocity;

    public BooleanSupplier isLauncherSpinning = () -> false;

    public LauncherSubsystem() {
        bottomSpinner = new TalonFX(HardwareID.bottomSpinnerMotorCANId);
        middleSpinner = new TalonFX(HardwareID.middleSpinnerMotorCANId);
        topSpinner = new TalonFX(HardwareID.topSpinnerMotorCANId);
        

        bottomSpinner.setNeutralMode(NeutralModeValue.Coast);
        middleSpinner.setNeutralMode(NeutralModeValue.Coast);
        topSpinner.setNeutralMode(NeutralModeValue.Coast);

        topSpinnerVelocity = new VelocityVoltage(0);
        middleSpinnerVelocity = new VelocityVoltage(0);
        bottomSpinnerVelocity = new VelocityVoltage(0);

        var config = new Slot0Configs();
        config.kV = TuningValues.launcherkV;
        config.kP = TuningValues.launcherkP;
        config.kI = TuningValues.launcherkI;
        config.kD = TuningValues.launcherkD;

        topSpinner.getConfigurator().apply(config);
        middleSpinner.getConfigurator().apply(config);
        bottomSpinner.getConfigurator().apply(config);

    }

    public void spinTopFlywheel(LaunchDirection direction) {
        topSpinnerTargetV = direction == LaunchDirection.AMP ? Preferences.getDouble(Keys.ampUpperV, 9.5) : Preferences.getDouble(Keys.speakerHighAimV, 65);
        

        topSpinner.setControl(topSpinnerVelocity.withVelocity(topSpinnerTargetV));
    }

    public void spinMiddleFlywheel(LaunchDirection direction) {
        if (direction != LaunchDirection.LOW) {
            middleSpinnerTargetV = direction == LaunchDirection.AMP ? Preferences.getDouble(Keys.ampMiddleV, 9.3) : Preferences.getDouble(Keys.speakerHighAimV, 65);
        }

        else {
            middleSpinnerTargetV = Preferences.getDouble(Keys.speakerLowAimV, -80);
        }
        middleSpinner.setControl(middleSpinnerVelocity.withVelocity(middleSpinnerTargetV));
    }


    public void spinBottomFlywheel() {

        bottomSpinnerTargetV =  Preferences.getDouble(Keys.speakerLowAimV, -80);

        bottomSpinner.setControl(bottomSpinnerVelocity.withVelocity(bottomSpinnerTargetV));
    }

    public void stopSpinners() {
        bottomSpinner.set(0.0);
        middleSpinner.set(0.0);
        topSpinner.set(0.0);                     
    }

    public boolean isLauncherUpToSpeed(LaunchDirection direction) {
        double bottomMaxVelocity;
        double topMaxVelocity;
        double middleMaxVelocity;

        switch (direction) {
            case AMP:
                middleMaxVelocity = Preferences.getDouble(Keys.ampMiddleV, 8.5);
                topMaxVelocity = Preferences.getDouble(Keys.ampUpperV, 5.0);
                break;
            
            case HIGH:
                middleMaxVelocity = Preferences.getDouble(Keys.speakerHighAimV, 65);
                topMaxVelocity = Preferences.getDouble(Keys.speakerHighAimV, 65);
                break;

            default:
                middleMaxVelocity = Preferences.getDouble(Keys.speakerLowAimV, -80);
                topMaxVelocity = Preferences.getDouble(Keys.speakerHighAimV, 65);
                break;
        }
        bottomMaxVelocity = Preferences.getDouble(Keys.speakerLowAimV, -80);

        double bottomSpinnerVelocity = bottomSpinner.getVelocity().getValueAsDouble();
        double topSpinnerVelocity = topSpinner.getVelocity().getValueAsDouble();
        double middleSpinnerVelocity = middleSpinner.getVelocity().getValueAsDouble();

        double tolerance = Preferences.getDouble(Keys.launcherTolerance, 5);

        if ((Math.abs(bottomSpinnerVelocity) > Math.abs(bottomMaxVelocity) - tolerance) || (Math.abs(topSpinnerVelocity) > Math.abs(topMaxVelocity) - tolerance)) {
          if (Math.abs(middleSpinnerVelocity) > Math.abs(middleMaxVelocity)) {
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
