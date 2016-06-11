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
    private static final double[] traceDown = new double[]{0.1, 0.1};
    private static final double traceEnergy = SumArr(traceDown);
    private final int traceSleepMax = 10;
    private LinkedList<SonarTrace> sonarHist = new LinkedList<SonarTrace>();
    private SonarStatus sonar = SonarStatus.Charge;
    private int traceStep = 0;
    private int recStep = 0;
    private int traceSleep = 0;

    private static double SumArr(double[] arr) {
        double sum = 0;
        for (double i : arr)
            sum += i;
        return sum;
    }

    @Override
    protected void actions() {
        sonarBot();
        myBot();
        setNameColor(new Color(Color.HSBtoRGB((float) ((getTime() % 5) / 5), 1, 1)));
    }

    private void myBot() {
        setBodyColor(Color.GRAY);

        boolean dodging = false;

        if (getProjectileRadar() != null) {
            Vector projectileP = getProjectileRadar().pos;
            Vector projectileS = getProjectileRadar().speed;

            Vector ownP = getPosition();
            Vector ownS = getVelocity();

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

            double difference = Math.abs((t_x - t_y) * ownS.getLength());

            if (t_x + t_y >= -getJRobotLength() * 8 / ownS.getLength() && difference <= getJRobotLength() * 2) {

                addDebugArrow(projectileP, projectileP.add(projectileS));
                addDebugArrow(ownP, ownP.add(ownS));

                addDebugCrosshair(projectileP.add(projectileS.mult(t_x)));
                addDebugCrosshair(projectileP.add(projectileS.mult(t_y)));
                addDebugCrosshair(ownP.add(ownS.mult(t_x)));
                addDebugCrosshair(ownP.add(ownS.mult(t_y)));

                double angle = projectileS.getAngle().sub(getPosition().sub(projectileP).getAngle()).normalize().angle;
                if (angle > Math.PI) {
                    angle -= Math.PI * 2;
                }

                if (angle > 0) {
                    if (angle < Math.PI / 2) {
                        dodging = true;
                        setBodyColor(Color.BLUE);
                        setAutopilot(projectileS.getAngle().sub(new Angle(90, "d")), 1);
                        if (difference <= getJRobotLength()) {
                            setBoost();
                            sonar = SonarStatus.Charge;
                        }
                    }
                } else {
                    if (angle > -Math.PI / 2) {
                        dodging = true;
                        setBodyColor(Color.YELLOW);
                        setAutopilot(projectileS.getAngle().add(new Angle(90, "d")), 1);
                        if (difference <= getJRobotLength()) {
                            setBoost();
                            sonar = SonarStatus.Charge;
                        }
                    }
                }
            }
        }
        if (!dodging && sonarHist.size() > 0) {
            double distanceToEnemy = sonarHist.getLast().location.sub(getPosition()).getLength();
            if (distanceToEnemy > getMaxArenaDiameter() * 3 / 4) {
                setAutopilot(sonarHist.getLast().location.sub(getPosition()).getAngle(), 0.25);
            } else if (distanceToEnemy > getMaxArenaDiameter() / 2) {
                setAutopilot(sonarHist.getLast().location.sub(getPosition()).getAngle(), 0.1);
            } else {
                setAutopilot(getPosition().sub(sonarHist.getLast().location).getAngle(), 0.5);
            }
        }
    }

    private void sonarBot() {
        SonarTrace sr = getLastSonarTrace();

        if (sr != null && (sonarHist.size() == 0 || sr.timestamp != sonarHist.peekLast().timestamp)) {
            sonarHist.add(sr);
            if (sonar == SonarStatus.Trace) recStep++;
        }

        switch (sonar) {
            case Charge:
                setTurretColor(Color.GREEN);
                if (getEnergy() < traceEnergy + getEnergyConsumptionProjectile())
                    break;
                traceStep = 0;
                recStep = 0;
                sonar = SonarStatus.Trace;
                //break;

            case Trace:
                setTurretColor(Color.YELLOW);
                if (recStep == traceStep && traceStep < traceDown.length) {
                    if (traceSleep < traceSleepMax) {
                        traceSleep++;
                    } else {
                        traceSleep = 0;
                        setSonarEnergy(traceDown[traceStep]);
                        traceStep++;
                    }
                }

                if (recStep == traceDown.length) {
                    sonar = SonarStatus.Fire;
                } else
                    break;

            case Fire:
                setTurretColor(Color.RED);
                attackBot();

                sonar = SonarStatus.Charge;
                break;
        }

        DrawHist();

        Draw(predictstart, predict);

        if (sonarHist.size() > 5)
            sonarHist.removeFirst();
    }

    double enemySpeeeed = 0;
    Vector dir = new Vector();

    Vector predictstart = new Vector();
    Vector predict = new Vector();

    private void attackBot() {
        //SonarTrace first = sonarHist.get(sonarHist.size() - 3);
        SonarTrace second = sonarHist.get(sonarHist.size() - 2);
        SonarTrace third = sonarHist.get(sonarHist.size() - 1);

        //Vector betw1_2 = second.location.sub(first.location);
        Vector betw2_3 = third.location.sub(second.location);

        double delta = third.timestamp - second.timestamp;
        double enemySpeed = betw2_3.getLength() / delta;
        Vector enemySpeedVector = betw2_3.getNormal().mult(enemySpeed);
        enemySpeeeed = enemySpeed;
        System.out.println(enemySpeeeed);
        dir = betw2_3;

        double Xp = third.location.getX(); // target pos ?
        double Yp = third.location.getY();
        double MovXp = enemySpeedVector.getX();
        double MovYp = enemySpeedVector.getY();
        double Xk = getPosition().getX(); // kugel
        double Yk = getPosition().getY();
        double Movk = getProjectileSpeed();

        Xk -= Xp;
        Yk -= Yp;
        double a = (MovXp * MovXp) + (MovYp * MovYp) + (0 * 0) - (Movk * Movk);
        double b = (-2 * Xk * MovXp) + (-2 * Yk * MovYp) + (-2 * 0 * 0);
        double c = (Xk * Xk) + (Yk * Yk) + (0 * 0);
        Xk += Xp;
        Yk += Yp;

        Result r = MidnightSolver(a, b, c);

        if (!r.hasResult)
            return;

        double useval;
        double sval;
        if (r.a >= 0 && !(r.b >= 0 && r.b < r.a)) {
            useval = r.a;
            sval = r.b;
        } else if (r.b >= 0) {
            useval = r.b;
            sval = r.a;
        } else
            return;

        Vector targetAim = new Vector(Xp + MovXp * useval, Yp + MovYp * useval);
        Vector hitPos = targetAim.sub(getPosition());
        predictstart = getPosition();
        predict = hitPos;

        setLaunchProjectileCommand(hitPos.getAngle());
    }

    private void Draw(Vector pos, Vector asDir) {
        Draw(pos, asDir, asDir.getLength());
    }

    private void Draw(Vector pos, Vector asDir, double len) {
        addDebugArrow(pos, pos.add(new Vector(len, asDir.getAngle())));
    }

    private void DrawHist() {
        SonarTrace srLast = null;
        for (SonarTrace srl : sonarHist) {
            if (srLast != null)
                addDebugLine(srl.location, srLast.location);
            srLast = srl;
        }
    }

    public Result MidnightSolver(double a, double b, double c) {
        double subsquare = b * b - 4 * a * c;
        if (subsquare < 0)
            return new Result(false);
        return new Result(true, (-b + Math.sqrt(subsquare)) / (2 * a), (-b - Math.sqrt(subsquare)) / (2 * a));
    }

    private enum SonarStatus {
        Sleep,
        Charge,
        Trace,
        Fire,
    }

    class Result {
        public boolean hasResult;
        public double a;
        public double b;

        public Result(boolean has) {
            hasResult = has;
            a = 0;
            b = 0;
        }

        public Result(boolean has, double _a, double _b) {
            hasResult = has;
            a = _a;
            b = _b;
        }
    }
}
