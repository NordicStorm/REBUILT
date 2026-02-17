package frc.robot.commands.paths;

import java.lang.reflect.Field;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Used to set the value of a DrivetrainConfig at runtime using reflection
 */
public class ConfigPropertySetPiece extends InstantCommand implements CommandPathPiece {
    MultiPartPath path;
    String propName;
    double value;

    public ConfigPropertySetPiece(MultiPartPath path, String propName, double value) {
        this.path = path;
        this.propName = propName;
        this.value = value;
    }

    @Override
    public void initialize() {
        Field field;

        try {
            field = path.drivetrainConfig.getClass().getDeclaredField(propName);
        } catch (NoSuchFieldException | SecurityException e) {
            throw new RuntimeException(e);
        }

        try {
            field.setDouble(path.drivetrainConfig, value);
        } catch (IllegalArgumentException | IllegalAccessException e) {
            throw new RuntimeException(e);
        }
        
        
    }

    
}