package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utilities.Constants.HardwareID;

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

        indexerMotor.setNeutralMode(NeutralModeValue.Brake);
        indexerMotorTwo.setNeutralMode(NeutralModeValue.Brake);

        indexerMotor.setInverted(true);
        indexerMotorTwo.setInverted(false);
        
        colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
        noteLoaded = false;
    }

    public void stopIndexer() {
        indexerMotor.set(0.0);
        indexerMotorTwo.set(0.0);
    }

    public void indexNoteIntake() {
        if (!noteLoaded) {
            indexerMotor.set(0.25);
            indexerMotorTwo.set(0.25);
            return;
        }

        indexerMotor.set(0.0);
        indexerMotorTwo.set(0.0);
    }

    public void indexNoteOuttake() {
        indexerMotor.set(-0.25);
        indexerMotorTwo.set(-0.25);
    }

    public void indexNoteLaunch() {
        indexerMotor.set(0.25);
        indexerMotorTwo.set(0.25);
    }

    //Used for Testing Purposes
    public void indexNoteIntakeDisregardLoading() {
        indexerMotor.set(0.25);
        indexerMotorTwo.set(0.25);
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
    }
}
