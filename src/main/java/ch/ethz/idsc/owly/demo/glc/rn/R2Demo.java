// code by jph
package ch.ethz.idsc.owly.demo.glc.rn;

import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.InvertedRegion;
import ch.ethz.idsc.owly.glc.adapter.RnPointcloudRegion;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.TimeInvariantRegion;
import ch.ethz.idsc.owly.glc.core.DefaultTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.Trajectories;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.gui.GlcFrame;
import ch.ethz.idsc.owly.math.flow.EulerIntegrator;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

class R2Demo {
  public static R2Demo createPointsInside() {
    R2Demo rnDemo = new R2Demo();
    rnDemo.controlSize = 40;
    rnDemo.root = Tensors.vector(0, 0);
    rnDemo.goal = new RnGoalManager(Tensors.vector(5, 0), DoubleScalar.of(.2));
    Tensor points = Tensors.matrix(new Number[][] { //
        { 0, 0 }, { 0, -1 }, { 0, -2 }, { 1, -2 }, { 2, -2 }, //
        { 3, -2 }, { 4, -2 }, { 5, -2 }, { 5, -1 }, { 5, 0 } //
    });
    rnDemo.obstacleQuery = // new EmptyRegionQuery();
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            new InvertedRegion(RnPointcloudRegion.create(points, RealScalar.of(0.6)))));
    return rnDemo;
  }

  int controlSize;
  Tensor root;
  RnGoalManager goal;
  TrajectoryRegionQuery obstacleQuery;

  private R2Demo() {
  }

  public static void main(String[] args) {
    final Scalar timeStep = RationalScalar.of(1, 5);
    Tensor partitionScale = Tensors.vector(4, 4);
    Collection<Flow> controls = R2Controls.createControls(36);
    RnGoalManager rnGoal = new RnGoalManager(Tensors.vector(2, 2), DoubleScalar.of(.25));
    // performance depends on heuristic: zeroHeuristic vs rnGoal
    // Heuristic heuristic = new ZeroHeuristic(); // rnGoal
    TrajectoryRegionQuery obstacleQuery = new SimpleTrajectoryRegionQuery( //
        new TimeInvariantRegion(new R2Bubbles()));
    StateIntegrator stateIntegrator = StateIntegrator.create(new EulerIntegrator(), timeStep, 5);
    // ---
    TrajectoryPlanner trajectoryPlanner = new DefaultTrajectoryPlanner( //
        stateIntegrator, partitionScale, controls, rnGoal, rnGoal, obstacleQuery);
    trajectoryPlanner.insertRoot(Tensors.vector(-2, -2));
    int iters = Expand.maxSteps(trajectoryPlanner, 1400);
    List<StateTime> trajectory = trajectoryPlanner.getPathFromRootToGoal();
    Trajectories.print(trajectory);
    GlcFrame glcFrame = new GlcFrame();
    glcFrame.glcComponent.setTrajectoryPlanner(trajectoryPlanner);
  }
}
