package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utilities.Constants.HardwareID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LauncherSubsystem extends SubsystemBase{

    TalonFX bottomSpinner, middleSpinner, topSpinner;

    private double upperTargetV = 0;
    private double lowerTargetV = 0;

    VelocityVoltage topVelocity;
    VelocityVoltage middleVelocity;
    VelocityVoltage bottomVelocity;

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

    public void spinUpperSpinners() {
        
        //Re-apply config for tuning.  SHOULD REMOVE BEFORE COMP
        var config = new Slot0Configs();
        config.kV = SmartDashboard.getNumber("V", 0);
        config.kP = SmartDashboard.getNumber("p", 0);
        config.kI = SmartDashboard.getNumber("i", 0);
        config.kD = SmartDashboard.getNumber("d", 0);
        
        upperTargetV = SmartDashboard.getNumber("upperTargetV", 50);

        topSpinner.getConfigurator().apply(config);
        middleSpinner.getConfigurator().apply(config);

        topSpinner.setControl(topVelocity.withVelocity(upperTargetV));
        middleSpinner.setControl(middleVelocity.withVelocity(upperTargetV));
    }

    public void spinLowerSpinners() {
        
        //Re-apply config for tuning.  SHOULD REMOVE BEFORE COMP
        var config = new Slot0Configs();
        config.kV = SmartDashboard.getNumber("V", 0);
        config.kP = SmartDashboard.getNumber("p", 0);
        config.kI = SmartDashboard.getNumber("i", 0);
        config.kD = SmartDashboard.getNumber("d", 0);

        lowerTargetV = SmartDashboard.getNumber("lowerTargetV", -50);

        middleSpinner.getConfigurator().apply(config);
        bottomSpinner.getConfigurator().apply(config);

        middleSpinner.setControl(middleVelocity.withVelocity(lowerTargetV));
        bottomSpinner.setControl(bottomVelocity.withVelocity(lowerTargetV));
    }

    // public void spinLowerSpinners(double speed) {
    //     bottomSpinner.set(speed);
    //     middleSpinner.set(speed);
    // }

    // public void spinUpperSpinners() {
    //     middleSpinner.set(-0.3);
    //     topSpinner.set(-0.3);
    // }

    // public void spinUpperSpinners(double speed) {
    // middleSpinner.set(-speed);
    // topSpinner.set(-speed);
    // }

    public void stopSpinners() {
        bottomSpinner.set(0.0);
        middleSpinner.set(0.0);
        topSpinner.set(0.0);
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
