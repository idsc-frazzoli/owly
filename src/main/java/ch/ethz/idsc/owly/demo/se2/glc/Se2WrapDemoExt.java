// code by jph
package ch.ethz.idsc.owly.demo.se2.glc;

import java.util.Collection;
import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owly.demo.se2.Se2Controls;
import ch.ethz.idsc.owly.demo.se2.Se2MinTimeGoalManager;
import ch.ethz.idsc.owly.demo.se2.Se2Wrap;
import ch.ethz.idsc.owly.demo.se2.Se2WrapGoalManagerExt;
import ch.ethz.idsc.owly.glc.adapter.IdentityWrap;
import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.GlcNodes;
import ch.ethz.idsc.owly.glc.core.StandardTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.ani.OwlyGui;
import ch.ethz.idsc.owly.math.CoordinateWrap;
import ch.ethz.idsc.owly.math.RotationUtils;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

/** Se2WrapDemoExt uses a different goal manager than Se2WrapDemo
 * 
 * (x,y,theta) */
enum Se2WrapDemoExt {
  ;
  public static void main(String[] args) {
    Tensor eta = Tensors.vector(3, 3, 50 / Math.PI);
    Tensor radiusVector = Tensors.of(DoubleScalar.of(0.2), DoubleScalar.of(0.2), RotationUtils.DEGREE(15));
    StateIntegrator stateIntegrator = FixedStateIntegrator.createDefault(RationalScalar.of(1, 6), 5);
    System.out.println("scale=" + eta);
    Collection<Flow> controls = Se2Controls.createControls(RotationUtils.DEGREE(45), 6);
    final CoordinateWrap identity = IdentityWrap.INSTANCE;
    CoordinateWrap coordinateWrap;
    coordinateWrap = new Se2Wrap(Tensors.vector(1, 1, 1));
    coordinateWrap = identity;
    Se2MinTimeGoalManager se2DefaultGoalManager = new Se2MinTimeGoalManager( //
        Tensors.vector(-.5, 0, 0), radiusVector, controls);
    Se2WrapGoalManagerExt se2WrapGoalManager = new Se2WrapGoalManagerExt( //
        coordinateWrap, se2DefaultGoalManager);
    TrajectoryRegionQuery obstacleQuery = Se2WrapDemo.obstacleQuery();
    // ---
    TrajectoryPlanner trajectoryPlanner = new StandardTrajectoryPlanner( //
        eta, stateIntegrator, controls, obstacleQuery, se2WrapGoalManager.getGoalInterface());
    trajectoryPlanner.represent = coordinateWrap::represent;
    // ---
    trajectoryPlanner.insertRoot(Tensors.vector(.1, 0, 0));
    int iters = Expand.maxSteps(trajectoryPlanner, 4000);
    System.out.println(iters);
    Optional<GlcNode> optional = trajectoryPlanner.getBest();
    if (optional.isPresent()) {
      List<StateTime> trajectory = GlcNodes.getPathFromRootTo(optional.get());
      StateTimeTrajectories.print(trajectory);
    }
    OwlyGui.glc(trajectoryPlanner);
  }
}
