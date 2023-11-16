package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Auto;

/**
 * Class for selecting autonomous routines.
 */
public class AutoSelection {

    // If you want to know what auto is running anywhere else in the code
    public static String selectedAuto;

    public static String selectedPiece;

    // Auto chooser.
    // Couple notes:
    // Usage: When we are plugged into the FMS (or queuing), use Glass to select an auto.
    // There is no default auto set up yet. So if we enable without selecting an auto,
    // we will hard crash w/ stack trace. Not really ideal on the field.
    public static final SendableChooser<String> autoChooser = new SendableChooser<>();

    public static final SendableChooser<String> pieceMode = new SendableChooser<>();

    /*
     * Setup the Auto selector.
     */
    public static void setup() {
        autoChooser.setDefaultOption("No Auto", "No Auto");
        autoChooser.addOption("Balance and Score", "Balance and Score");
        autoChooser.addOption("Substation Auto", "Substation Auto");
        autoChooser.addOption("Substation J-Manuver", "Substation J-Manuver");
        autoChooser.addOption("Edge J-Manuver", "Edge J-Manuver");
        autoChooser.addOption("Edge Auto", "Edge Auto");
        autoChooser.addOption("Middle Auto", "Middle Auto");
        autoChooser.addOption("Test", "Test");
        
        pieceMode.setDefaultOption("CUBE", "CUBE");
        pieceMode.addOption("CONE", "CONE");
    }

    public static Piece getSelected() {
        switch (pieceMode.getSelected()) {
            case "CUBE":
                return Piece.CUBE;
            case "CONE":
                return Piece.CONE;
        }
        return Piece.CUBE;
    }

    /**
     * Run the auto selector. Gets the selected string from Glass,
     * and runs the auto that corresponds with the string chosen.
     */
    public static void run() {

        selectedAuto = autoChooser.getSelected();
        selectedPiece = pieceMode.getSelected();

        Auto.scoringPiece = getSelected();

        switch(autoChooser.getSelected()) {

            case "No Auto":
                Auto.doNothing();
            break;

            case "Balance and Score":
                Auto.balanceAndScore();
            break;

            case "Substation Auto":
                Auto.substationAuto();
            break;

            case "Substation J-Manuver":
                Auto.substation_J_Auto();
            break;

            case "Edge J-Manuver":
                Auto.edge_J_Manuver();
            break;

            case "Edge Auto":
                Auto.edgeAuto();
            break;

            case "Middle Auto":
                Auto.middleAuto();
            break;

            case "Test":
                Auto.testSwervePID();
            break;
        }

    }
}
