// code by jl and jph
package ch.ethz.idsc.owly.demo.se2.any;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owl.glc.adapter.Expand;
import ch.ethz.idsc.owl.glc.adapter.GlcNodes;
import ch.ethz.idsc.owl.glc.adapter.GlcTrajectories;
import ch.ethz.idsc.owl.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owl.glc.adapter.Trajectories;
import ch.ethz.idsc.owl.glc.core.DebugUtils;
import ch.ethz.idsc.owl.glc.core.GlcNode;
import ch.ethz.idsc.owl.glc.core.GoalInterface;
import ch.ethz.idsc.owl.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owl.glc.std.StandardTrajectoryPlanner;
import ch.ethz.idsc.owl.gui.GeometricLayer;
import ch.ethz.idsc.owl.gui.ani.AbstractAnyEntity;
import ch.ethz.idsc.owl.gui.ani.PlannerType;
import ch.ethz.idsc.owl.math.CoordinateWrap;
import ch.ethz.idsc.owl.math.Degree;
import ch.ethz.idsc.owl.math.flow.EulerIntegrator;
import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.owl.math.flow.Integrator;
import ch.ethz.idsc.owl.math.map.Se2Utils;
import ch.ethz.idsc.owl.math.region.EllipsoidRegion;
import ch.ethz.idsc.owl.math.region.InvertedRegion;
import ch.ethz.idsc.owl.math.region.Region;
import ch.ethz.idsc.owl.math.region.Regions;
import ch.ethz.idsc.owl.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owl.math.state.SimpleEpisodeIntegrator;
import ch.ethz.idsc.owl.math.state.StateIntegrator;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owl.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.owly.demo.rn.EuclideanDistanceDiscoverRegion;
import ch.ethz.idsc.owly.demo.rn.R2Flows;
import ch.ethz.idsc.owly.demo.rn.RnMinDistSphericalGoalManager;
import ch.ethz.idsc.owly.demo.se2.CarStandardFlows;
import ch.ethz.idsc.owly.demo.se2.Se2CarIntegrator;
import ch.ethz.idsc.owly.demo.se2.Se2MinTimeGoalManager;
import ch.ethz.idsc.owly.demo.se2.Se2StateSpaceModel;
import ch.ethz.idsc.owly.demo.se2.Se2TrajectoryGoalManager;
import ch.ethz.idsc.owly.demo.se2.Se2Wrap;
import ch.ethz.idsc.owly.demo.se2.glc.Se2Parameters;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Array;
import ch.ethz.idsc.tensor.sca.Sqrt;

/* package */ class Se2AnyEntity extends AbstractAnyEntity {
  private static final Integrator INTEGRATOR = Se2CarIntegrator.INSTANCE;
  /** {vx=0,vy=0,rate=0} */
  private static final Tensor FALLBACK_CONTROL = Array.zeros(3).unmodifiable();
  private static final Tensor SHAPE = Tensors.matrixDouble( //
      new double[][] { //
          { .2, +.07, 1 }, //
          { .2, -.07, 1 }, //
          { -.1, -.07, 1 }, //
          { -.1, +.07, 1 } //
      }).unmodifiable();
  private static final Se2Wrap SE2WRAP = new Se2Wrap(Tensors.vector(1, 1, 2));
  // ---
  private final Tensor goalRadius;
  private static final Scalar DELAY_HINT = RealScalar.of(1.5);
  private static final Scalar EXPAND_TIME = RealScalar.of(1);
  private Region<Tensor> environmentRegion;

  /** @param state initial position of entity */
  public Se2AnyEntity(StateTime state, int resolution) {
    super(new Se2Parameters( //
        (RationalScalar) RealScalar.of(resolution), // resolution
        RealScalar.of(2), // TimeScale
        RealScalar.of(200), // DepthScale
        Tensors.vector(5, 5, 50 / Math.PI), // PartitionScale 50/pi == 15.9155
        RationalScalar.of(1, 6), // dtMax
        2000, // maxIter
        Se2StateSpaceModel.INSTANCE.getLipschitz()), // Lipschitz
        new CarStandardFlows(RealScalar.ONE, Degree.of(60)).getFlows(resolution), //
        // ---
        new SimpleEpisodeIntegrator( //
            Se2StateSpaceModel.INSTANCE, //
            INTEGRATOR, //
            state),
        // ---
        DELAY_HINT, EXPAND_TIME); //
    final Scalar goalRadius_xy = Sqrt.of(RealScalar.of(2)).divide(parameters.getEta().Get(0));
    final Scalar goalRadius_theta = Sqrt.of(RealScalar.of(2)).divide(parameters.getEta().Get(2));
    goalRadius = Tensors.of(goalRadius_xy, goalRadius_xy, goalRadius_theta);
  }

  @Override
  public PlannerType getPlannerType() {
    return PlannerType.ANY;
  }

  @Override
  public Scalar distance(Tensor x, Tensor y) {
    return SE2WRAP.distance(x, y);
  }

  @Override
  protected Tensor fallbackControl() {
    return FALLBACK_CONTROL;
  }

  @Override
  public void render(GeometricLayer geometricLayer, Graphics2D graphics) {
    {// indicate current position
      StateTime stateTime = getStateTimeNow();
      Color color = new Color(64, 64, 64, 128);
      graphics.setColor(color);
      Tensor matrix = Se2Utils.toSE2Matrix(stateTime.state());
      Path2D path2d = geometricLayer.toPath2D(Tensor.of(SHAPE.stream().map(matrix::dot)));
      graphics.fill(path2d);
    }
    { // indicate position delay[s] into the future
      Tensor state = getEstimatedLocationAt(delayHint());
      Point2D point = geometricLayer.toPoint2D(state);
      graphics.setColor(new Color(255, 128, 64, 192));
      graphics.fill(new Rectangle2D.Double(point.getX() - 2, point.getY() - 2, 5, 5));
    }
    {
      graphics.setColor(new Color(0, 128, 255, 192));
      Tensor matrix = geometricLayer.getMouseSe2Matrix();
      Path2D path2d = geometricLayer.toPath2D(Tensor.of(SHAPE.stream().map(matrix::dot)));
      graphics.fill(path2d);
    }
  }

  @Override
  protected final GoalInterface createGoal(Tensor goal) {
    return createGoalFromR2(goal);
    // return new Se2MinDistGoalManager(goal, goalRadius).getGoalInterface();
  }

  @Override
  protected Region<Tensor> createGoalCheckHelp(Tensor goal) {
    return new InvertedRegion(Regions.emptyRegion());
  }

  protected final GoalInterface createGoalFromR2(Tensor goal) {
    Tensor currentState = getEstimatedLocationAt(DELAY_HINT).extract(0, 2);
    Tensor r2goal = goal.extract(0, 2);
    Tensor goalRadiusR2 = goalRadius.extract(0, 2).append(DoubleScalar.POSITIVE_INFINITY);
    long tic = System.nanoTime();
    // ---
    Tensor eta = Tensors.vector(8, 8);
    StateIntegrator stateIntegratorR2 = FixedStateIntegrator.create(EulerIntegrator.INSTANCE, RationalScalar.of(1, 5), 5);
    R2Flows r2Config = new R2Flows(RealScalar.ONE);
    Collection<Flow> controlsR2 = r2Config.getFlows(10);
    // TODO JONAS extract R2planner from Se2AnyEntity
    GoalInterface rnGoal = RnMinDistSphericalGoalManager.create(r2goal, goalRadius.Get(0));
    // ---
    TrajectoryRegionQuery obstacleQueryR2 = SimpleTrajectoryRegionQuery.timeInvariant( //
        EuclideanDistanceDiscoverRegion.of(environmentRegion, currentState, RealScalar.of(2)));
    obstacleQueryR2 = new SimpleTrajectoryRegionQuery(new TimeInvariantRegion(environmentRegion));
    TrajectoryPlanner trajectoryPlanner = new StandardTrajectoryPlanner( //
        eta, stateIntegratorR2, controlsR2, obstacleQueryR2, rnGoal);
    if (rnGoal.isMember(new StateTime(currentState, RealScalar.ZERO)))
      return new Se2MinTimeGoalManager(goal, goalRadius, controls).getGoalInterface();
    trajectoryPlanner.insertRoot(new StateTime(currentState, RealScalar.ZERO));
    // int iters =
    Expand.maxTime(trajectoryPlanner, RealScalar.of(1.5)); // 1.5 [s]
    Optional<GlcNode> optional = trajectoryPlanner.getFinalGoalNode();
    DebugUtils.heuristicConsistencyCheck(trajectoryPlanner);
    List<StateTime> trajectory = GlcNodes.getPathFromRootTo(optional.get());
    List<Region<Tensor>> goalRegionsList = new ArrayList<>();
    for (StateTime entry : trajectory) {
      Tensor goalTemp = Tensors.of(entry.state().Get(0), entry.state().Get(1), RealScalar.ZERO);
      if (trajectory.indexOf(entry) == trajectory.size() - 1) // if last entry of trajectory
        goalRegionsList.add(new EllipsoidRegion(goal, goalRadius));
      else
        goalRegionsList.add(new EllipsoidRegion(goalTemp, goalRadiusR2));
    }
    long toc = System.nanoTime();
    System.err.println("TrajectoryGoalcreation took: " + (toc - tic) * 1e-9 + "s");
    System.out.println("Coarse guidance trajectory");
    Trajectories.print( //
        GlcTrajectories.detailedTrajectoryTo(trajectoryPlanner.getStateIntegrator(), optional.get()));
    return new Se2TrajectoryGoalManager(goalRegionsList, trajectory, goalRadiusR2, controls);
  }

  @Override
  protected TrajectoryRegionQuery initializeObstacle(Region<Tensor> oldEnvironmentRegion, StateTime currentState) {
    environmentRegion = oldEnvironmentRegion;
    return SimpleTrajectoryRegionQuery.timeInvariant(oldEnvironmentRegion);
    // return updateObstacle(oldEnvironmentRegion, currentState);
  }

  @Override
  protected TrajectoryRegionQuery updateObstacle(Region<Tensor> oldEnvironmentRegion, StateTime currentState) {
    environmentRegion = oldEnvironmentRegion; // environment stays the same
    return SimpleTrajectoryRegionQuery.timeInvariant( //
        EuclideanDistanceDiscoverRegion.of(oldEnvironmentRegion, currentState.state(), RealScalar.of(4.5)));
  }

  @Override
  protected StateIntegrator createIntegrator() {
    return FixedStateIntegrator.create(INTEGRATOR, parameters.getdtMax(), parameters.getTrajectorySize());
  }

  @Override
  protected CoordinateWrap getWrap() {
    return SE2WRAP;
  }
}
