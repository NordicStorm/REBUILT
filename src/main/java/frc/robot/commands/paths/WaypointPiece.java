package frc.robot.commands.paths;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;

public class WaypointPiece implements PathPiece{

    private Translation2d point;
    List<CommandPathPiece> parallelCommands = new ArrayList<>();
    Double forcedEndDirection = null; 
    /**
     * Follows the coordinate system of the path
     * @param x meters
     * @param y meters
     */
    public WaypointPiece(double x, double y){
        point = new Translation2d(x, y);
    }

    public void forcedEndDirection(double angle){
        this.forcedEndDirection = Double.valueOf(angle);
    }

    public void addParallelCommand(CommandPathPiece command){
        parallelCommands.add(command);
    }

    public Translation2d getPoint(){
        return point;
    }

    public void setPoint(Translation2d point){
        this.point = point;
    }
    
    @Override
    public PieceType getPieceType() {
        return PieceType.Waypoint;
    }
    
}