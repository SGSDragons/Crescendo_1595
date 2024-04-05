package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utilities.Constants.HardwareID;
import frc.lib.utilities.Constants.Keys;
import frc.lib.utilities.Constants.TuningValues;
import frc.lib.utilities.Constants.VoltageVelocityValues;
import frc.robot.commands.LaunchTeleop.LaunchDirection;

public class LauncherSubsystem extends SubsystemBase{

    TalonFX bottomSpinner, middleSpinner, topSpinner;

    private double topSpinnerTargetV, bottomSpinnerTargetV, middleSpinnerTargetV;

    VelocityVoltage topSpinnerVelocity, middleSpinnerVelocity, bottomSpinnerVelocity;

    public BooleanSupplier isLauncherSpinning = () -> false;

    public LauncherSubsystem() {
        bottomSpinner = new TalonFX(HardwareID.bottomSpinnerMotorCANId);
        middleSpinner = new TalonFX(HardwareID.middleSpinnerMotorCANId);
        topSpinner = new TalonFX(HardwareID.topSpinnerMotorCANId);

        topSpinnerVelocity = new VelocityVoltage(0);
        middleSpinnerVelocity = new VelocityVoltage(0);
        bottomSpinnerVelocity = new VelocityVoltage(0);

        configureMotors();

    }

    public void spinTopFlywheel(LaunchDirection direction) {
        topSpinnerTargetV = direction == LaunchDirection.AMP ? VoltageVelocityValues.ampUpperV : VoltageVelocityValues.speakerHighAimV;
        

        topSpinner.setControl(topSpinnerVelocity.withVelocity(topSpinnerTargetV));
    }

    public void spinMiddleFlywheel(LaunchDirection direction) {
        if (direction != LaunchDirection.LOW) {
            middleSpinnerTargetV = direction == LaunchDirection.AMP ? VoltageVelocityValues.ampMiddleV : VoltageVelocityValues.speakerHighAimV;
        }
        else {
            middleSpinnerTargetV = VoltageVelocityValues.speakerLowAimV;
        }
        middleSpinner.setControl(middleSpinnerVelocity.withVelocity(middleSpinnerTargetV));
    }

    public void spinMiddleFlywheel(double velocity) {
        middleSpinnerTargetV = velocity;
        middleSpinner.setControl(middleSpinnerVelocity.withVelocity(middleSpinnerTargetV));
    }


    public void spinBottomFlywheel() {

        bottomSpinnerTargetV = VoltageVelocityValues.speakerLowAimV;

        bottomSpinner.setControl(bottomSpinnerVelocity.withVelocity(bottomSpinnerTargetV));
    }

    public void spinBottomFlywheel(double velocity) {

        bottomSpinnerTargetV = velocity;

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
                middleMaxVelocity = VoltageVelocityValues.ampMiddleV;
                topMaxVelocity =VoltageVelocityValues.ampUpperV;
                break;
            
            case HIGH:
                middleMaxVelocity = VoltageVelocityValues.speakerHighAimV;
                topMaxVelocity = VoltageVelocityValues.speakerHighAimV;
                break;

            default:
                middleMaxVelocity = VoltageVelocityValues.speakerLowAimV;
                topMaxVelocity = VoltageVelocityValues.speakerHighAimV;
                break;
        }
        bottomMaxVelocity = VoltageVelocityValues.speakerLowAimV;

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
        //telemetry();
    }

    private void telemetry() {
        SmartDashboard.putNumber("Bottom Spinner Speed", bottomSpinner.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Middle Spinner Speed", middleSpinner.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Top Spinner Speed", topSpinner.getVelocity().getValueAsDouble());
    }

    private void configureMotors() {
        // bottomSpinner.setNeutralMode(NeutralModeValue.Coast);
        // middleSpinner.setNeutralMode(NeutralModeValue.Coast);
        // topSpinner.setNeutralMode(NeutralModeValue.Coast);

        bottomSpinner.setNeutralMode(NeutralModeValue.Brake);
        middleSpinner.setNeutralMode(NeutralModeValue.Brake);
        topSpinner.setNeutralMode(NeutralModeValue.Brake);

        var velocityConfig = new Slot0Configs();
        velocityConfig.kV = TuningValues.launcherkV;
        velocityConfig.kP = TuningValues.launcherkP;
        velocityConfig.kI = TuningValues.launcherkI;
        velocityConfig.kD = TuningValues.launcherkD;

        var rampRateConfig = new ClosedLoopRampsConfigs();
        rampRateConfig.VoltageClosedLoopRampPeriod = 0.25;

        var topConfig = topSpinner.getConfigurator();
        var middleConfig = middleSpinner.getConfigurator();
        var bottomConfig = bottomSpinner.getConfigurator();

        topConfig.apply(velocityConfig);
        middleConfig.apply(velocityConfig);
        bottomConfig.apply(velocityConfig);
        topConfig.apply(rampRateConfig);
        middleConfig.apply(rampRateConfig);
        bottomConfig.apply(rampRateConfig);
    }
}
