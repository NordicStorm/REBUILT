package frc.robot.commands.paths;


public interface PathPiece{
    public static enum PieceType{Waypoint, Command, Branch}

    /**
     * @return What kind of path piece this is
     */
    public PieceType getPieceType();

}