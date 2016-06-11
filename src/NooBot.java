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
    private static final double[] traceDown = new double[]{0.2, 0.2, 0.2};
    private static final double traceEnergy = SumArr(traceDown);
    private final int traceSleepMax = 5;
    private LinkedList<SonarTrace> sonarHist = new LinkedList<SonarTrace>();
    private SonarStatus sonar = SonarStatus.Charge;
    private Vector predictionMain = new Vector();
    private Vector movement = new Vector();
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
        // TODO implement brain

        sonarBot();
        //attackBot();
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

        Draw(getPosition(), dir, enemySpeeeed);

        if (sonarHist.size() > 5)
            sonarHist.removeFirst();
    }

    double enemySpeeeed = 0;
    Vector dir = new Vector();

    private void attackBot() {
        SonarTrace first = sonarHist.get(sonarHist.size() - 3);
        SonarTrace second = sonarHist.get(sonarHist.size() - 2);
        SonarTrace third = sonarHist.get(sonarHist.size() - 1);

        Vector betw1_2 = second.location.sub(first.location);
        Vector betw2_3 = third.location.sub(second.location);

        double delta = third.timestamp - second.timestamp;
        double enemySpeed = betw2_3.getLength() / delta;
        Vector enemyVector = betw2_3.getNormal().mult(enemySpeed);
        enemySpeeeed = enemySpeed;
        dir = betw2_3;

        double Xp = third.location.getX(); // target pos ?
        double Yp = third.location.getY();
        double MovXp = enemyVector.getX();
        double MovYp = enemyVector.getY();
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

        setLaunchProjectileCommand(targetAim.sub(getPosition()).getAngle());
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
