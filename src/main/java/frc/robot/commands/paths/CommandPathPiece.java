package frc.robot.commands.paths;

public interface CommandPathPiece extends PathPiece {

    /**
     * Is used only needed if interruptsTrajectory is true.
     * 
     * @return the requested speed in meters per second
     */
    public default double getRequestedStartSpeed() {
        return 0;
    }

    @Override
    public default PieceType getPieceType() {
        return PieceType.Command;
    }

}