// code by jph
package ch.ethz.idsc.owly.demo.glc.rn;

import ch.ethz.idsc.owly.glc.adapter.MinTimeCost;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.TimeInvariantRegion;
import ch.ethz.idsc.owly.glc.core.Controls;
import ch.ethz.idsc.owly.glc.core.CostFunction;
import ch.ethz.idsc.owly.glc.core.DynamicalSystem;
import ch.ethz.idsc.owly.glc.core.Heuristic;
import ch.ethz.idsc.owly.glc.core.Trajectory;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.gui.GlcFrame;
import ch.ethz.idsc.owly.math.EllipsoidRegion;
import ch.ethz.idsc.owly.math.RegionUnion;
import ch.ethz.idsc.owly.math.integrator.EulerIntegrator;
import ch.ethz.idsc.owly.math.integrator.Integrator;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

public class RnDemo {
  public static void main(String[] args) {
    Integrator integrator = new EulerIntegrator();
    DynamicalSystem dynamicalSystem = new DynamicalSystem(RealScalar.of(.5));
    Controls controls = RnControls.createR2RadialControls(15, RealScalar.of(.7));
    CostFunction costFunction = new MinTimeCost();
    Tensor goal = Tensors.vector(2, 2);
    // Tensors.vector(5, 1);
    Heuristic heuristic = new RnDistanceHeuristic(goal);
    // new ZeroHeuristic();
    TrajectoryRegionQuery goalQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            new EllipsoidRegion(goal, Tensors.vector(.25, .25))));
    TrajectoryRegionQuery obstacleQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            RegionUnion.of( //
                // new EllipsoidRegion(Tensors.vector(-2.2, 0), Tensors.vector(.5, 7)) //
                // new EllipsoidRegion(Tensors.vector(2.5, 0), Tensors.vector(2, 2)) //
                new R2Bubbles())));
    // ---
    TrajectoryPlanner trajectoryPlanner = new TrajectoryPlanner( //
        integrator, dynamicalSystem, controls, costFunction, heuristic, goalQuery, obstacleQuery);
    trajectoryPlanner.setResolution(Tensors.vector(7, 7));
    trajectoryPlanner.insertRoot(Tensors.vector(-2, -2));
    trajectoryPlanner.plan(25);
    Trajectory trajectory = trajectoryPlanner.getPathFromRootToGoal();
    trajectory.print();
    GlcFrame glcFrame = new GlcFrame();
    glcFrame.glcComponent.setTrajectoryPlanner(trajectoryPlanner);
  }
}
