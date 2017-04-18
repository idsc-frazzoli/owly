// code by jph
package ch.ethz.idsc.owly.demo.glc.rn;

import ch.ethz.idsc.owly.glc.adapter.MinTimeCost;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.TimeInvariantRegion;
import ch.ethz.idsc.owly.glc.adapter.ZeroHeuristic;
import ch.ethz.idsc.owly.glc.core.Controls;
import ch.ethz.idsc.owly.glc.core.CostFunction;
import ch.ethz.idsc.owly.glc.core.DynamicalSystem;
import ch.ethz.idsc.owly.glc.core.Heuristic;
import ch.ethz.idsc.owly.glc.core.Trajectory;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.gui.GlcFrame;
import ch.ethz.idsc.owly.math.EllipsoidRegion;
import ch.ethz.idsc.owly.math.integrator.EulerIntegrator;
import ch.ethz.idsc.owly.math.integrator.Integrator;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensors;

public class RnDemo {
  public static void main(String[] args) {
    Integrator integrator = new EulerIntegrator();
    DynamicalSystem dynamicalSystem = new DynamicalSystem(RealScalar.of(.5));
    Controls controls = RnControls.createR2RadialControls(40, RealScalar.of(.7));
    CostFunction costFunction = new MinTimeCost();
    Heuristic heuristic = new ZeroHeuristic();
    TrajectoryRegionQuery goalQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            new EllipsoidRegion(Tensors.vector(5, 0), Tensors.vector(.25, .25))));
    TrajectoryRegionQuery obstacleQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            new EllipsoidRegion(Tensors.vector(2.5, 0), Tensors.vector(2, 2))));
    // ---
    TrajectoryPlanner trajectoryPlanner = new TrajectoryPlanner( //
        integrator, dynamicalSystem, controls, costFunction, heuristic, goalQuery, obstacleQuery);
    trajectoryPlanner.setResolution(Tensors.vector(7, 7));
    trajectoryPlanner.insertRoot(Tensors.vector(0, 0));
    trajectoryPlanner.plan(25);
    Trajectory trajectory = trajectoryPlanner.getPathFromRootToGoal();
    trajectory.print();
    GlcFrame glcFrame = new GlcFrame();
    glcFrame.glcComponent.setTrajectoryPlanner(trajectoryPlanner);
  }
}
