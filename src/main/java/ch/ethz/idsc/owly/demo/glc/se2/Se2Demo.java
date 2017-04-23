// code by jph
package ch.ethz.idsc.owly.demo.glc.se2;

import ch.ethz.idsc.owly.glc.adapter.EmptyRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.MinTimeCost;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.TimeInvariantRegion;
import ch.ethz.idsc.owly.glc.core.Controls;
import ch.ethz.idsc.owly.glc.core.CostFunction;
import ch.ethz.idsc.owly.glc.core.DynamicalSystem;
import ch.ethz.idsc.owly.glc.core.Heuristic;
import ch.ethz.idsc.owly.glc.core.SteepTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.Trajectory;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.gui.GlcFrame;
import ch.ethz.idsc.owly.math.integrator.EulerIntegrator;
import ch.ethz.idsc.owly.math.integrator.Integrator;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensors;

/** (x,y,theta) */
public class Se2Demo {
  public static void main(String[] args) {
    Integrator integrator = new EulerIntegrator();
    DynamicalSystem dynamicalSystem = new DynamicalSystem(RealScalar.of(.25));
    Controls controls = new Se2Controls(10);
    CostFunction costFunction = new MinTimeCost();
    Se2Goal rnGoal = new Se2Goal(Tensors.vector(5, -1, Math.PI / 2), DoubleScalar.of(.2));
    Heuristic heuristic = rnGoal; // new ZeroHeuristic();
    TrajectoryRegionQuery goalQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            rnGoal));
    // new EllipsoidRegion(Tensors.vector(5, -1, Math.PI/2), Tensors.vector(.5, .5, .2)) //
    // ));
    TrajectoryRegionQuery obstacleQuery = //
        new EmptyRegionQuery();
    // new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
    // RegionUnion.of( //
    // new EllipsoidRegion(Tensors.vector(+3, +1), Tensors.vector(1.75, .75)), // speed limit along the way
    // new EllipsoidRegion(Tensors.vector(-2, +0), Tensors.vector(1, 1)) // block to the left
    // )));
    // ---
    TrajectoryPlanner trajectoryPlanner = new SteepTrajectoryPlanner( //
        integrator, dynamicalSystem, controls, costFunction, heuristic, goalQuery, obstacleQuery);
    // ---
    trajectoryPlanner.setResolution(Tensors.vector(3, 3, 3));
    trajectoryPlanner.insertRoot(Tensors.vector(0, 0, 0));
    trajectoryPlanner.plan(25);
    Trajectory trajectory = trajectoryPlanner.getPathFromRootToGoal();
    trajectory.print();
    GlcFrame glcFrame = new GlcFrame();
    glcFrame.glcComponent.setTrajectoryPlanner(trajectoryPlanner);
  }
}
