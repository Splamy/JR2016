import jrobots.utils.*;
import jrobots.simulation.simulationObjects.JRobot2015_3;

import java.awt.*;
import java.util.LinkedList;

/**
 * A minimal functional Bot, ready for being programmed.
 * <p>Please enter the documentation via the method <tt>actions()</tt>.
 * <p>Please rename uniquely. The individual name will be displayed in the GUI.
 *
 * @see JRobot2015_3#actions()
 */
public class NooBot extends JRobot2015_3 {
    private static final long serialVersionUID = 142001L;

    private LinkedList<SonarTrace> sonarHist = new LinkedList<SonarTrace>();
    private int chargeTo = 0;
    private boolean chargeUp = false;
    private boolean chargeUpConfirmed = false;
    private boolean requestScan = true;

    @Override
    protected void actions() {
        sonarBot();
        myBot();
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
                //requestScan = false;
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

    private void myBot() {
        setTurretColor(Color.GRAY);
        if (getProjectileRadar() != null) {
            Vector projectileP = getProjectileRadar().pos;
            Vector projectileS = getProjectileRadar().speed;

            Vector ownP = getPosition();
            Vector ownS = getVelocity();

            addDebugArrow(projectileP, projectileP.add(projectileS));
            addDebugArrow(ownP, ownP.add(ownS));

            {
                double p_x = projectileP.getX();
                double p_y = projectileP.getY();
                double p_dx = projectileS.getX();
                double p_dy = projectileS.getY();

                double o_x = ownP.getX();
                double o_y = ownP.getY();
                double o_dx = ownS.getX();
                double o_dy = ownS.getY();

                double t_x = (p_x - o_x) / (o_dx - p_dx);
                double t_y = (p_y - o_y) / (o_dy - p_dy);

                setNameColor(Color.CYAN);
                addDebugCrosshair(projectileP.add(projectileS.mult(t_x)));
                addDebugCrosshair(projectileP.add(projectileS.mult(t_y)));
                addDebugCrosshair(ownP.add(ownS.mult(t_x)));
                addDebugCrosshair(ownP.add(ownS.mult(t_y)));
            }


            double angle = projectileS.getAngle().sub(getPosition().sub(projectileP).getAngle()).normalize().angle;
            if (angle > Math.PI) {
                angle -= Math.PI * 2;
            }

            if (angle > 0) {
                if (angle < Math.PI / 2) {
                    setTurretColor(Color.BLUE);
                    setAutopilot(projectileS.getAngle().sub(new Angle(90, "d")), 1);
                    setBoost();
                }
            } else {
                if (angle > -Math.PI / 2) {
                    setTurretColor(Color.YELLOW);
                    setAutopilot(projectileS.getAngle().add(new Angle(90, "d")), 1);
                    setBoost();
                }
            }
        } else if (sonarHist.size() > 0) {
            double distanceToEnemy = sonarHist.getLast().location.sub(getPosition()).getLength();
            if (distanceToEnemy > getMaxArenaDiameter() * 3 / 4) {
                setAutopilot(sonarHist.getLast().location.sub(getPosition()).getAngle(), 1);
            } else if (distanceToEnemy > getMaxArenaDiameter() / 2) {
                setAutopilot(sonarHist.getLast().location.sub(getPosition()).getAngle(), 0.5);
            } else {
                setAutopilot(getPosition().sub(sonarHist.getLast().location).getAngle(), 0.5);
            }
        }
    }
}
