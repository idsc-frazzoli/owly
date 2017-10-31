// code by jl
package ch.ethz.idsc.owly.demo.rnxt.glc;

import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owly.demo.rn.R2Controls;
import ch.ethz.idsc.owly.demo.rn.R2Parameters;
import ch.ethz.idsc.owly.demo.rn.RnHeuristicEllipsoidGoalManager;
import ch.ethz.idsc.owly.glc.adapter.Parameters;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.GlcNodes;
import ch.ethz.idsc.owly.glc.core.StandardTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.ani.OwlyGui;
import ch.ethz.idsc.owly.math.RotationUtils;
import ch.ethz.idsc.owly.math.StateTimeTensorFunction;
import ch.ethz.idsc.owly.math.flow.EulerIntegrator;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

enum R2xtRingGlcDemo {
  ;
  public static void main(String[] args) {
    RationalScalar resolution = (RationalScalar) RealScalar.of(8);
    Tensor partitionScale = Tensors.vector(16, 16, 64);
    Scalar timeScale = RealScalar.of(6);
    Scalar depthScale = RealScalar.of(100);
    Scalar dtMax = RationalScalar.of(1, 6);
    int maxIter = 1000000;
    Scalar lipschitz = RealScalar.ONE;
    Parameters parameters = new R2Parameters(resolution, timeScale, depthScale, partitionScale, dtMax, maxIter, lipschitz);
    // TODO why does time has resolution only 1 !?
    System.out.println("1/DomainSize: " + parameters.getEta());
    StateIntegrator stateIntegrator = FixedStateIntegrator.create(EulerIntegrator.INSTANCE, parameters.getdtMax(), //
        parameters.getTrajectorySize());
    Collection<Flow> controls = R2Controls.createRadial(parameters.getResolutionInt());
    controls.add(RnxTControls.stayPut(2));
    Tensor goal = Tensors.vector(5, 5);
    RnHeuristicEllipsoidGoalManager rnGoal = new RnHeuristicEllipsoidGoalManager(//
        goal, Tensors.of(RealScalar.of(0.2), RealScalar.of(0.2)));
    // HeuristicGoalManager expands only 10% of nodes
    // RnxtEllipsoidGoalManager rnGoal = new RnxtEllipsoidGoalManager(//
    // goal, Tensors.of(RealScalar.of(0.2), RealScalar.of(0.2), DoubleScalar.POSITIVE_INFINITY));
    // GoalRegion at x:5, y= 5 and all time
    TrajectoryRegionQuery obstacleQuery = //
        new SimpleTrajectoryRegionQuery(new TimeDependentTurningRingRegion( //
            Tensors.vector(0, 0), //
            RotationUtils.DEGREE(90), //
            RotationUtils.DEGREE(40), //
            RealScalar.of(0.5), //
            RealScalar.of(3)));
    // ---
    StateTime root = new StateTime(Tensors.vector(0, 0), RealScalar.ZERO);
    obstacleQuery.isDisjoint(Collections.singletonList(root));
    rnGoal.isDisjoint(Collections.singletonList(root));
    rnGoal.minCostToGoal(root.state());
    TrajectoryPlanner trajectoryPlanner = new StandardTrajectoryPlanner( //
        parameters.getEta(), stateIntegrator, controls, obstacleQuery, rnGoal);
    trajectoryPlanner.represent = StateTimeTensorFunction.withTime();
    trajectoryPlanner.insertRoot(root);
    int iters = Expand.maxSteps(trajectoryPlanner, maxIter);
    System.out.println(iters);
    Optional<GlcNode> optional = trajectoryPlanner.getBest();
    if (optional.isPresent()) {
      List<StateTime> trajectory = GlcNodes.getPathFromRootTo(optional.get());
      StateTimeTrajectories.print(trajectory);
    }
    OwlyGui.glc(trajectoryPlanner);
  }
}
