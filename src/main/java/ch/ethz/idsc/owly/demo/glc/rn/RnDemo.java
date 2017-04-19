// code by jph
package ch.ethz.idsc.owly.demo.glc.rn;

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
import ch.ethz.idsc.owly.math.EllipsoidRegion;
import ch.ethz.idsc.owly.math.RegionUnion;
import ch.ethz.idsc.owly.math.integrator.EulerIntegrator;
import ch.ethz.idsc.owly.math.integrator.Integrator;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

public class RnDemo {
  public static RnDemo createSphere() {
    RnDemo rnDemo = new RnDemo();
    rnDemo.controlSize = 40;
    rnDemo.root = Tensors.vector(0, 0);
    rnDemo.goal = Tensors.vector(5, 0);
    rnDemo.obstacleQuery = // new EmptyRegionQuery();
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            RegionUnion.of( //
                new EllipsoidRegion(Tensors.vector(4, 3), Tensors.vector(2, 2)), //
                new EllipsoidRegion(Tensors.vector(2.5, 0), Tensors.vector(2, 2)) //
            )));
    return rnDemo;
  }

  public static RnDemo createBubbles() {
    RnDemo rnDemo = new RnDemo();
    rnDemo.controlSize = 15;
    rnDemo.root = Tensors.vector(-2, -2);
    rnDemo.goal = Tensors.vector(2, 2);
    rnDemo.obstacleQuery = new SimpleTrajectoryRegionQuery(new TimeInvariantRegion(new R2Bubbles()));
    return rnDemo;
  }

  int controlSize;
  Tensor root;
  Tensor goal;
  TrajectoryRegionQuery obstacleQuery;

  private RnDemo() {
  }

  public static void main(String[] args) {
    RnDemo rnDemo = createBubbles();
    // rnDemo = createSphere();
    Integrator integrator = new EulerIntegrator();
    DynamicalSystem dynamicalSystem = new DynamicalSystem(RealScalar.of(.5));
    Controls controls = RnControls.createR2RadialControls(rnDemo.controlSize, RealScalar.of(.7));
    CostFunction costFunction = new MinTimeCost();
    Heuristic heuristic = new RnDistanceHeuristic(rnDemo.goal);
    TrajectoryRegionQuery goalQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            new EllipsoidRegion(rnDemo.goal, Tensors.vector(.25, .25))));
    TrajectoryRegionQuery obstacleQuery = rnDemo.obstacleQuery;
    // ---
    TrajectoryPlanner trajectoryPlanner = new SteepTrajectoryPlanner( //
        integrator, dynamicalSystem, controls, costFunction, heuristic, goalQuery, obstacleQuery);
    trajectoryPlanner.setResolution(Tensors.vector(7, 7));
    trajectoryPlanner.insertRoot(rnDemo.root);
    trajectoryPlanner.plan(25);
    Trajectory trajectory = trajectoryPlanner.getPathFromRootToGoal();
    trajectory.print();
    GlcFrame glcFrame = new GlcFrame();
    glcFrame.glcComponent.setTrajectoryPlanner(trajectoryPlanner);
  }
}
