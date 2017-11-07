// code by jph
package ch.ethz.idsc.owly.demo.rn.glc;

import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owly.demo.rn.R2Bubbles;
import ch.ethz.idsc.owly.demo.rn.R2Controls;
import ch.ethz.idsc.owly.demo.rn.RnMinDistSphericalGoalManager;
import ch.ethz.idsc.owly.demo.util.UserHome;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.GlcNodes;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.glc.core.StandardTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.ani.OwlyFrame;
import ch.ethz.idsc.owly.gui.ani.OwlyGui;
import ch.ethz.idsc.owly.math.flow.EulerIntegrator;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.EllipsoidRegion;
import ch.ethz.idsc.owly.math.region.RegionUnion;
import ch.ethz.idsc.owly.math.state.EmptyTrajectoryRegionQuery;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.TensorRuntimeException;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.io.AnimationWriter;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.Ramp;

enum R2DemoSlow {
  ;
  static TrajectoryPlanner simpleEmpty() throws Exception {
    return simple(EmptyTrajectoryRegionQuery.INSTANCE);
  }

  static TrajectoryPlanner simpleR2Bubbles() throws Exception {
    return simple(SimpleTrajectoryRegionQuery.timeInvariant(new R2Bubbles()));
  }

  static TrajectoryPlanner simpleR2Circle() throws Exception {
    return simple(SimpleTrajectoryRegionQuery.timeInvariant( //
        RegionUnion.wrap(Arrays.asList( //
            // TODO add more regions
            new EllipsoidRegion(Tensors.vector(-1, 0), Tensors.vector(2, 2))))));
  }

  private static TrajectoryPlanner simple(TrajectoryRegionQuery obstacleQuery) throws Exception {
    final Tensor stateRoot = Tensors.vector(-2.2, -2.2);
    final Tensor stateGoal = Tensors.vector(2, 3.5);
    final Scalar radius = DoubleScalar.of(0.8);
    // ---
    Tensor eta = Tensors.vector(1.5, 1.5);
    StateIntegrator stateIntegrator = FixedStateIntegrator.create(EulerIntegrator.INSTANCE, RationalScalar.of(1, 5), 5);
    Collection<Flow> controls = R2Controls.createRadial(6);
    GoalInterface goalInterface = RnMinDistSphericalGoalManager.create(stateGoal, radius);
    // ---
    TrajectoryPlanner trajectoryPlanner = new StandardTrajectoryPlanner( //
        eta, stateIntegrator, controls, obstacleQuery, goalInterface);
    trajectoryPlanner.insertRoot(new StateTime(stateRoot, RealScalar.ZERO));
    AnimationWriter gsw = AnimationWriter.of(UserHome.Pictures("R2_Slow.gif"), 400);
    OwlyFrame owly = OwlyGui.start();
    for (int i = 0; i < 20; i++) {
      Optional<GlcNode> optional = trajectoryPlanner.getBest();
      if (optional.isPresent())
        break;
      // int iters =
      Expand.maxSteps(trajectoryPlanner, 1);
      owly.setGlc(trajectoryPlanner);
      gsw.append(owly.offscreen());
    }
    for (int i = 0; i < 4; i++)
      gsw.append(owly.offscreen());
    gsw.close();
    Optional<GlcNode> optional = trajectoryPlanner.getBest();
    if (optional.isPresent()) {
      GlcNode goalNode = optional.get(); // <- throws exception if
      Scalar cost = goalNode.costFromRoot();
      Scalar lowerBound = Ramp.of(Norm._2.between(stateGoal, stateRoot).subtract(radius));
      if (Scalars.lessThan(cost, lowerBound))
        throw TensorRuntimeException.of(cost, lowerBound);
    }
    return trajectoryPlanner;
  }

  static void demo(TrajectoryPlanner trajectoryPlanner) {
    Optional<GlcNode> optional = trajectoryPlanner.getBest();
    if (optional.isPresent()) {
      List<StateTime> trajectory = GlcNodes.getPathFromRootTo(optional.get());
      StateTimeTrajectories.print(trajectory);
    }
    // OwlyGui.glc(trajectoryPlanner);
  }

  public static void main(String[] args) throws Exception {
    simpleR2Circle();
  }
}
