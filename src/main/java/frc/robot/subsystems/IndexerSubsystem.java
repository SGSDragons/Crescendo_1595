package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utilities.Constants.HardwareID;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.ColorSensorV3;

public class IndexerSubsystem extends SubsystemBase{

    TalonFX indexerMotor, indexerMotorTwo;
    ColorSensorV3 colorSensor;
    Color percievedColor;
    //DigitalInput beamBreakSensor = new DigitalInput(0); //True when Unimposed
    boolean noteLoaded = false;

    public IndexerSubsystem() {
        indexerMotor = new TalonFX(HardwareID.indexerMotorCANId);
        indexerMotorTwo = new TalonFX(HardwareID.indexerMotor2CANId);
        indexerMotorTwo.setControl(new Follower(HardwareID.indexerMotorCANId, true));
        
        indexerMotor.setNeutralMode(NeutralModeValue.Brake);
        indexerMotor.setInverted(true);
        
        colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
        noteLoaded = false;
    }

    public void stopIndexer() {
        indexerMotor.setVoltage(0.0);
    }

    public void indexNoteIntake() {
        if (!noteLoaded) {
            indexerMotor.setVoltage(SmartDashboard.getNumber("indexerIntakeVoltage", 3.00));
            return;
        }

        indexerMotor.setVoltage(0.0);
    }

    public void indexNoteOuttake() {
        indexerMotor.setVoltage(SmartDashboard.getNumber("indexerOuttakeVoltage", 3.00));
    }

    public void indexNoteLaunch() {
        indexerMotor.set(SmartDashboard.getNumber("indexerVoltage", 3.00));
    }

    //Used for Testing Purposes
    public void indexNoteIntakeDisregardLoading() {
        indexerMotor.setVoltage(SmartDashboard.getNumber("indexerIntakeVoltage", 3.00));
    }
    
    @Override
    public void periodic() {

        /*
        if (!beamBreakSensor.get()) {
            noteLoaded = true;
        }
        else {
            noteLoaded = false;
        }
        */
        
        int proximity = colorSensor.getProximity();
        if (proximity > 250) {
            noteLoaded = true;
        }
        else {
            noteLoaded = false;
        }
        telemetry();
    }

    public boolean isNoteLoaded() {
        return noteLoaded;
    }

    public void telemetry() {
        SmartDashboard.putNumber("Indexer Motor Velocity", indexerMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Note Proximity", colorSensor.getProximity());
        SmartDashboard.putBoolean("Note Loaded", noteLoaded);
        SmartDashboard.getNumber("indexerVoltage", 3.00);
        SmartDashboard.getNumber("indexerOuttakeVoltage", 3.00);
    }
}
