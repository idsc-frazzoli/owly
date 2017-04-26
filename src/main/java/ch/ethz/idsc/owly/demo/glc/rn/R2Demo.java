// code by jph
package ch.ethz.idsc.owly.demo.glc.rn;

import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.InvertedRegion;
import ch.ethz.idsc.owly.glc.adapter.MinTimeCost;
import ch.ethz.idsc.owly.glc.adapter.RnPointcloudRegion;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.TimeInvariantRegion;
import ch.ethz.idsc.owly.glc.adapter.ZeroHeuristic;
import ch.ethz.idsc.owly.glc.core.Controls;
import ch.ethz.idsc.owly.glc.core.CostFunction;
import ch.ethz.idsc.owly.glc.core.DefaultTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.Heuristic;
import ch.ethz.idsc.owly.glc.core.StateTime;
import ch.ethz.idsc.owly.glc.core.Trajectory;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.gui.GlcFrame;
import ch.ethz.idsc.owly.math.integrator.EulerIntegrator;
import ch.ethz.idsc.owly.math.integrator.Integrator;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

public class R2Demo {
  public static R2Demo createPointsInside() {
    R2Demo rnDemo = new R2Demo();
    rnDemo.controlSize = 40;
    rnDemo.root = Tensors.vector(0, 0);
    rnDemo.goal = new RnGoal(Tensors.vector(5, 0), DoubleScalar.of(.2));
    Tensor points = Tensors.matrix(new Number[][] { //
        { 0, 0 }, { 0, -1 }, { 0, -2 }, //
        { 1, -2 }, { 2, -2 }, { 3, -2 }, { 4, -2 }, //
        { 5, -2 }, { 5, -1 }, { 5, 0 } //
    });
    rnDemo.obstacleQuery = // new EmptyRegionQuery();
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            new InvertedRegion(RnPointcloudRegion.create(points, RealScalar.of(0.6)))));
    return rnDemo;
  }

  int controlSize;
  Tensor root;
  RnGoal goal;
  TrajectoryRegionQuery obstacleQuery;

  private R2Demo() {
  }

  public static void main(String[] args) {
    Integrator integrator = new EulerIntegrator();
    final Scalar timeStep = RationalScalar.of(1, 5);
    Tensor partitionScale = Tensors.vector(4, 4);
    Controls controls = new R2Controls(36);
    int trajectorySize = 5;
    CostFunction costFunction = new MinTimeCost();
    RnGoal rnGoal = new RnGoal(Tensors.vector(2, 2), DoubleScalar.of(.25));
    // performance depends on heuristic: zeroHeuristic vs rnGoal
    Heuristic heuristic = new ZeroHeuristic(); // rnGoal
    TrajectoryRegionQuery goalQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion(rnGoal));
    TrajectoryRegionQuery obstacleQuery = new SimpleTrajectoryRegionQuery( //
        new TimeInvariantRegion(new R2Bubbles()));
    // ---
    TrajectoryPlanner trajectoryPlanner = new DefaultTrajectoryPlanner( //
        integrator, timeStep, partitionScale, controls, trajectorySize, costFunction, heuristic, goalQuery, obstacleQuery);
    trajectoryPlanner.insertRoot(Tensors.vector(-2, -2));
    trajectoryPlanner.plan(1400);
    List<StateTime> trajectory = trajectoryPlanner.getPathFromRootToGoal();
    Trajectory.print(trajectory);
    GlcFrame glcFrame = new GlcFrame();
    glcFrame.glcComponent.setTrajectoryPlanner(trajectoryPlanner);
  }
}
