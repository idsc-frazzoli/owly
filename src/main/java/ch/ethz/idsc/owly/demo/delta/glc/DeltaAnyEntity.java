// code by jl and jph
//TODO
package ch.ethz.idsc.owly.demo.delta.glc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owl.glc.adapter.Expand;
import ch.ethz.idsc.owl.glc.adapter.GlcNodes;
import ch.ethz.idsc.owl.glc.adapter.IdentityWrap;
import ch.ethz.idsc.owl.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owl.glc.core.DebugUtils;
import ch.ethz.idsc.owl.glc.core.GlcNode;
import ch.ethz.idsc.owl.glc.core.GoalInterface;
import ch.ethz.idsc.owl.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owl.glc.std.StandardTrajectoryPlanner;
import ch.ethz.idsc.owl.gui.ani.AbstractAnyEntity;
import ch.ethz.idsc.owl.gui.ani.PlannerType;
import ch.ethz.idsc.owl.math.CoordinateWrap;
import ch.ethz.idsc.owl.math.StateSpaceModels;
import ch.ethz.idsc.owl.math.flow.EulerIntegrator;
import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.owl.math.flow.Integrator;
import ch.ethz.idsc.owl.math.region.EllipsoidRegion;
import ch.ethz.idsc.owl.math.region.ImageRegion;
import ch.ethz.idsc.owl.math.region.InvertedRegion;
import ch.ethz.idsc.owl.math.region.Region;
import ch.ethz.idsc.owl.math.region.RegionUnion;
import ch.ethz.idsc.owl.math.region.Regions;
import ch.ethz.idsc.owl.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owl.math.state.SimpleEpisodeIntegrator;
import ch.ethz.idsc.owl.math.state.StateIntegrator;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owl.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.owly.demo.delta.DeltaAltStateSpaceModel;
import ch.ethz.idsc.owly.demo.delta.DeltaFlows;
import ch.ethz.idsc.owly.demo.delta.DeltaHeuristicGoalManager;
import ch.ethz.idsc.owly.demo.delta.DeltaParameters;
import ch.ethz.idsc.owly.demo.delta.DeltaTrajectoryGoalManager;
import ch.ethz.idsc.owly.demo.delta.ImageGradient;
import ch.ethz.idsc.owly.demo.rn.EuclideanDistanceDiscoverRegion;
import ch.ethz.idsc.owly.demo.rn.R2Flows;
import ch.ethz.idsc.owly.demo.rn.R2xTEllipsoidStateTimeRegion;
import ch.ethz.idsc.owly.demo.rn.RnMinDistSphericalGoalManager;
import ch.ethz.idsc.owly.demo.util.TrajectoryTranslationFamily;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.io.ResourceData;
import ch.ethz.idsc.tensor.sca.Power;
import ch.ethz.idsc.tensor.sca.Sqrt;

/* package */ class DeltaAnyEntity extends AbstractAnyEntity {
  private static final Integrator INTEGRATOR = EulerIntegrator.INSTANCE;
  public static final Tensor FALLBACK_CONTROL = Tensors.vectorDouble(0, 0).unmodifiable();
  public static final Scalar AMP = RealScalar.of(-.02);
  private static final Scalar U_NORM = RealScalar.of(0.1);
  public static final Tensor IMAGE_OBSTACLE = ResourceData.of("/io/delta_free.png");
  public static final Tensor RANGE = Tensors.vector(9, 6.5).unmodifiable(); // overall size of map
  public static final ImageGradient IMAGEGRADIENT = ImageGradient.linear(ResourceData.of("/io/delta_uxy.png"), RANGE, AMP);
  private static final DeltaAltStateSpaceModel DELTASTATESPACEMODEL = //
      new DeltaAltStateSpaceModel(IMAGEGRADIENT, U_NORM);
  /** preserve 1[s] of the former trajectory */
  private static final Scalar DELAY_HINT = RealScalar.of(4);
  private static final Scalar EXPAND_TIME = RealScalar.of(3);
  private static final Scalar SENSING_RADIUS = RealScalar.of(4.5);
  private static final Region<Tensor> ENVIRONMENTOBSTACLEMAP = //
      new ImageRegion(ResourceData.of("/io/delta_free.png"), RANGE, true);
  // --
  private final List<R2xTEllipsoidStateTimeRegion> undiscoveredObstaclesList = new ArrayList<>();
  private TrajectoryRegionQuery obstacleQuery;
  private final Scalar maxSpeed;
  private final Tensor goalRadius;

  /** @param state initial position of entity */
  public DeltaAnyEntity(List<Tensor> undiscoveredObstacleTensorList, StateTime state, int resolution) {
    super(new DeltaParameters( //
        (RationalScalar) RealScalar.of(resolution), // resolution
        RealScalar.of(50), // TimeScale
        RealScalar.of(100), // DepthScale
        Tensors.vector(120, 120), // PartitionScale
        RationalScalar.of(1, 6), // dtMax
        2000, // maxIter
        DELTASTATESPACEMODEL.getLipschitz()), // Lipschitz
        // -- Controls
        new DeltaFlows(DELTASTATESPACEMODEL, U_NORM). //
            getFlows(Power.of(RealScalar.of(resolution), FALLBACK_CONTROL.length()).number().intValue()), // From B. Paden's PhdThesis
        // ---Integrator
        new SimpleEpisodeIntegrator( //
            DELTASTATESPACEMODEL, //
            INTEGRATOR, //
            state),
        // ---
        DELAY_HINT, EXPAND_TIME); //
    final Scalar goalRadius_xy = Sqrt.of(RealScalar.of(2)).divide(parameters.getEta().Get(0));
    this.goalRadius = Tensors.of(goalRadius_xy, goalRadius_xy);
    this.maxSpeed = DELTASTATESPACEMODEL.getLipschitz().add(U_NORM);
    System.out.println("MaxGradient of field is: " + IMAGEGRADIENT.maxNormGradient());
    // Obstacle creation
    Flow flow = StateSpaceModels.createFlow(DELTASTATESPACEMODEL, DeltaEntity.FALLBACK_CONTROL);
    StateIntegrator stateIntegratorObstacles = FixedStateIntegrator.create( //
        INTEGRATOR, RationalScalar.of(1, 10), 120 * 10);
    for (Tensor temp : undiscoveredObstacleTensorList) {
      Tensor radius = Tensors.of(temp.Get(2), temp.Get(2));
      Tensor pos = temp.extract(0, 2);
      undiscoveredObstaclesList.add(new R2xTEllipsoidStateTimeRegion(radius, //
          TrajectoryTranslationFamily.create(stateIntegratorObstacles, new StateTime(pos, RealScalar.ZERO), flow), //
          () -> getStateTimeNow().time()));
    }
  }

  @Override
  public PlannerType getPlannerType() {
    return PlannerType.ANY;
  }

  public TrajectoryRegionQuery getTrajectoryRegionQuery() {
    return obstacleQuery;
  }

  public Region<StateTime> getFloatingObstacle(int i) {
    return undiscoveredObstaclesList.get(i);
  }

  @Override
  protected Scalar distance(Tensor x, Tensor y) {
    return IdentityWrap.INSTANCE.distance(x, y);
  }

  @Override
  protected Tensor fallbackControl() {
    return FALLBACK_CONTROL;
  }

  @Override
  protected final GoalInterface createGoal(Tensor goal) {
    return createGoalFromR2(goal);
  }

  @Override
  protected Region<Tensor> createGoalCheckHelp(Tensor goal) {
    return new InvertedRegion(Regions.emptyRegion());
  }

  protected final GoalInterface createGoalFromR2(Tensor goal) {
    Tensor currentState = getEstimatedLocationAt(DELAY_HINT).extract(0, 2);
    Tensor r2GoalState = goal.extract(0, 2);
    System.out.println("creating waypoints from R2 planner to" + goal);
    long tic = System.nanoTime();
    // ---
    Tensor eta = Tensors.vector(8, 8);
    StateIntegrator stateIntegratorR2 = FixedStateIntegrator.create(EulerIntegrator.INSTANCE, RationalScalar.of(1, 5), 5);
    R2Flows r2Config = new R2Flows(RealScalar.ONE);
    Collection<Flow> controlsR2 = r2Config.getFlows(8);
    // TODO JONAS extract R2planner from DeltaAnyEntity
    GoalInterface r2GoalManager = RnMinDistSphericalGoalManager.create(r2GoalState, goalRadius.Get(0));
    // ---
    TrajectoryRegionQuery obstacleQueryR2 = new SimpleTrajectoryRegionQuery(new TimeInvariantRegion(ENVIRONMENTOBSTACLEMAP));
    TrajectoryPlanner trajectoryPlanner = new StandardTrajectoryPlanner( //
        eta, stateIntegratorR2, controlsR2, obstacleQueryR2, r2GoalManager);
    if (r2GoalManager.isMember(new StateTime(currentState, RealScalar.ZERO)))
      return new DeltaHeuristicGoalManager(r2GoalState, goalRadius, maxSpeed);
    trajectoryPlanner.insertRoot(new StateTime(currentState, RealScalar.ZERO));
    Expand.maxTime(trajectoryPlanner, RealScalar.of(1.5)); // 1.5 [s]
    Optional<GlcNode> optional = trajectoryPlanner.getFinalGoalNode();
    DebugUtils.heuristicConsistencyCheck(trajectoryPlanner);
    List<StateTime> trajectory = GlcNodes.getPathFromRootTo(optional.get());
    List<Region<Tensor>> goalRegionsList = new ArrayList<>();
    for (StateTime entry : trajectory) {
      Tensor goalTemp = entry.state();
      goalRegionsList.add(new EllipsoidRegion(goalTemp, goalRadius));
    }
    long toc = System.nanoTime();
    System.err.println("TrajectoryGoalcreation took: " + (toc - tic) * 1e-9 + "s");
    System.out.println("Coarse guidance trajectory size: " + trajectory.size());
    return new DeltaTrajectoryGoalManager(goalRegionsList, trajectory, goalRadius, maxSpeed);
  }

  @Override
  protected TrajectoryRegionQuery initializeObstacle(Region<Tensor> region, StateTime currentState) {
    System.out.println("Created Initial Obstacle Region");
    return updateObstacle(region, currentState);
  }

  @Override
  protected TrajectoryRegionQuery updateObstacle(Region<Tensor> region, StateTime currentState) {
    System.out.println("Updating Obstacle");
    obstacleQuery = SimpleTrajectoryRegionQuery.timeInvariant( //
        EuclideanDistanceDiscoverRegion.of(//
            RegionUnion.wrap(Arrays.asList(ENVIRONMENTOBSTACLEMAP, //
                undiscoveredObstaclesList.get(0).regionAtTime(), //
                undiscoveredObstaclesList.get(1).regionAtTime(), //
                undiscoveredObstaclesList.get(2).regionAtTime(), //
                undiscoveredObstaclesList.get(3).regionAtTime())),
            currentState.state(), SENSING_RADIUS));
    return obstacleQuery;
  }

  @Override
  protected StateIntegrator createIntegrator() {
    return FixedStateIntegrator.create(INTEGRATOR, parameters.getdtMax(), parameters.getTrajectorySize());
  }

  @Override
  protected CoordinateWrap getWrap() {
    return IdentityWrap.INSTANCE;
  }
}
