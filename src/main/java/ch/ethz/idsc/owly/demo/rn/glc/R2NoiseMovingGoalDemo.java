// code by jph
package ch.ethz.idsc.owly.demo.rn.glc;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owl.data.Lists;
import ch.ethz.idsc.owl.data.Stopwatch;
import ch.ethz.idsc.owl.glc.adapter.Expand;
import ch.ethz.idsc.owl.glc.adapter.GlcNodes;
import ch.ethz.idsc.owl.glc.adapter.MultiCostGoalAdapter;
import ch.ethz.idsc.owl.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owl.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owl.glc.core.GlcNode;
import ch.ethz.idsc.owl.glc.core.GoalInterface;
import ch.ethz.idsc.owl.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owl.glc.par.Parameters;
import ch.ethz.idsc.owl.glc.std.StandardTrajectoryPlanner;
import ch.ethz.idsc.owl.gui.ani.OwlyFrame;
import ch.ethz.idsc.owl.gui.ani.OwlyGui;
import ch.ethz.idsc.owl.math.SingleIntegratorStateSpaceModel;
import ch.ethz.idsc.owl.math.StateSpaceModel;
import ch.ethz.idsc.owl.math.StateSpaceModels;
import ch.ethz.idsc.owl.math.StateTimeTensorFunction;
import ch.ethz.idsc.owl.math.flow.EulerIntegrator;
import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.owl.math.flow.RungeKutta45Integrator;
import ch.ethz.idsc.owl.math.region.Region;
import ch.ethz.idsc.owl.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owl.math.state.StateIntegrator;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.owl.math.state.TrajectorySample;
import ch.ethz.idsc.owly.demo.rn.R2Flows;
import ch.ethz.idsc.owly.demo.rn.R2NoiseCostFunction;
import ch.ethz.idsc.owly.demo.rn.R2NoiseRegion;
import ch.ethz.idsc.owly.demo.rn.R2Parameters;
import ch.ethz.idsc.owly.demo.rn.R2xTEllipsoidStateTimeRegion;
import ch.ethz.idsc.owly.demo.rn.RnMinTimeMovingGoalGoalManager;
import ch.ethz.idsc.owly.demo.util.TrajectoryTranslationFamily;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Array;

/** expands: 1491
 * computation time: 0.876993604 */
enum R2NoiseMovingGoalDemo {
  ;
  public static void main(String[] args) {
    // SETUP
    RationalScalar resolution = (RationalScalar) RationalScalar.of(9, 1);
    Tensor partitionScale = Tensors.vector(16, 16, 8);
    Scalar timeScale = RealScalar.of(5);
    Scalar depthScale = RealScalar.of(10000);
    Scalar dtMax = RationalScalar.of(1, 6);
    int maxIter = 200000;
    Scalar lipschitz = RealScalar.ONE;
    Parameters parameters = new R2Parameters(resolution, timeScale, depthScale, //
        partitionScale, dtMax, maxIter, lipschitz);
    System.out.println("1/DomainSize: " + parameters.getEta());
    R2Flows r2Config = new R2Flows(RealScalar.ONE);
    StateIntegrator stateIntegrator = FixedStateIntegrator.create(EulerIntegrator.INSTANCE, parameters.getdtMax(), //
        parameters.getTrajectorySize());
    final Scalar threshold = RealScalar.of(0.1);
    Region<Tensor> region = new R2NoiseRegion(threshold);
    Collection<Flow> controls = r2Config.getFlows(parameters.getResolutionInt());
    final Tensor center = Tensors.vector(5, 1);
    final Tensor radius = Tensors.vector(0.2, 0.2);
    final Scalar goalTrajectoryPrintTime = RealScalar.of(20);
    // starting point
    StateTime goalStartPoint = new StateTime(center, RealScalar.ZERO);
    StateSpaceModel stateSpaceModel = SingleIntegratorStateSpaceModel.INSTANCE;
    Flow flow = StateSpaceModels.createFlow(stateSpaceModel, Tensors.vector(-0.8, 0.3));
    StateIntegrator goalStateIntegrator = FixedStateIntegrator.create( //
        RungeKutta45Integrator.INSTANCE, parameters.getdtMax(), RealScalar.of(120).divide(parameters.getdtMax()).number().intValue());
    Region<StateTime> goalRegion = new R2xTEllipsoidStateTimeRegion(radius, //
        TrajectoryTranslationFamily.create(goalStateIntegrator, goalStartPoint, flow), //
        null);
    List<TrajectorySample> renderedGoalTrajectory = new ArrayList<>();
    renderedGoalTrajectory.add(new TrajectorySample(goalStartPoint, null));
    StateTime next = goalStartPoint;
    for (int i = 6; Scalars.lessThan(RealScalar.of(i), goalTrajectoryPrintTime.divide(parameters.getExpandTime())); i++) {
      next = Lists.getLast(stateIntegrator.trajectory(next, flow));
      renderedGoalTrajectory.add(new TrajectorySample(next, flow));
    }
    GoalInterface goalInterface = MultiCostGoalAdapter.of( //
        new RnMinTimeMovingGoalGoalManager(goalRegion), //
        Arrays.asList(new R2NoiseCostFunction(threshold.subtract(RealScalar.of(0.3)))));
    TrajectoryRegionQuery obstacleQuery = SimpleTrajectoryRegionQuery.timeInvariant(region);
    // ---
    TrajectoryPlanner trajectoryPlanner = new StandardTrajectoryPlanner( //
        partitionScale, stateIntegrator, controls, obstacleQuery, goalInterface);
    trajectoryPlanner.represent = StateTimeTensorFunction.withTime();
    trajectoryPlanner.insertRoot(new StateTime(Array.zeros(2), RealScalar.ZERO));
    Stopwatch stopwatch = Stopwatch.started();
    int iters = Expand.maxSteps(trajectoryPlanner, 10000);
    System.out.println(iters + " " + stopwatch.display_seconds());
    Optional<GlcNode> optional = trajectoryPlanner.getBest();
    if (optional.isPresent()) {
      List<StateTime> trajectory = GlcNodes.getPathFromRootTo(optional.get());
      StateTimeTrajectories.print(trajectory);
    }
    OwlyFrame owlyFrame = OwlyGui.glc(trajectoryPlanner);
    owlyFrame.addTrajectory(renderedGoalTrajectory, new Color(224, 168, 0, 224));
    owlyFrame.configCoordinateOffset(100, 300);
  }
}
