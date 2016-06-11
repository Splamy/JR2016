import jrobots.utils.*;
import jrobots.simulation.simulationObjects.JRobot2015_3;

import java.util.LinkedList;

/** A minimal functional Bot, ready for being programmed. 
 * <p>Please enter the documentation via the method <tt>actions()</tt>.
 * <p>Please rename uniquely. The individual name will be displayed in the GUI.
 * @see JRobot2015_3#actions()
 */
public class NooBot extends JRobot2015_3 {
	private static final long serialVersionUID = 142001L;

	private LinkedList<SonarTrace> sonarHist = new LinkedList<SonarTrace>();
	private int chargeTo = 0;
	private boolean chargeUp = false;
	private boolean chargeUpConfirmed = false;
	private boolean requestScan = false;

	@Override
	protected void actions() {
		// TODO implement brain

		//sonarBot();
	}

	private void sonarBot() {
		final double ScanEngy = 0.1;

		SonarTrace sr = getLastSonarTrace();
		if (sonarHist.size() == 0) {
			if (getEnergy() >= ScanEngy) {
				setSonarEnergy(ScanEngy);
			}

			if (sr != null)
				sonarHist.add(sr);
		} else {
			double chargeExtra = getEnergyConsumptionProjectile() * chargeTo + 0.01;
			if (requestScan && getEnergy() > ScanEngy + (chargeUp ? chargeExtra : 0)) {
				setSonarEnergy(ScanEngy);
				requestScan = false;
			} else
				setSonarEnergy(0);

			if (sr != null && sr.timestamp != sonarHist.peekLast().timestamp) {
				chargeUpConfirmed = true;
				sonarHist.add(sr);
			}
		}

		if (sonarHist.size() > 10)
			sonarHist.removeFirst();
	}
}
