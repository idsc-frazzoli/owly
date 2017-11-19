// code by jph
package ch.ethz.idsc.owly.demo.se2.twd;

import java.util.Collection;

import ch.ethz.idsc.owly.demo.se2.Se2CarIntegrator;
import ch.ethz.idsc.owly.demo.se2.Se2LateralAcceleration;
import ch.ethz.idsc.owly.demo.se2.Se2MinTimeGoalManager;
import ch.ethz.idsc.owly.demo.se2.Se2StateSpaceModel;
import ch.ethz.idsc.owly.demo.se2.Se2Wrap;
import ch.ethz.idsc.owly.demo.se2.glc.Se2Entity;
import ch.ethz.idsc.owly.glc.adapter.MultiCostGoalAdapter;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.glc.core.StandardTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.ani.PlannerType;
import ch.ethz.idsc.owly.math.StateTimeTensorFunction;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.SimpleEpisodeIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Sqrt;

/* package */ class TwdEntity extends Se2Entity {
  private static final Tensor PARTITIONSCALE = Tensors.vector(6, 6, 50 / Math.PI); // 50/pi == 15.9155
  private static final Scalar SQRT2 = Sqrt.of(RealScalar.of(2));
  // triangle
  private static final Tensor SHAPE = Tensors.matrixDouble( //
      new double[][] { { .3, 0 }, { -.1, -.1 }, { -.1, +.1 } }).unmodifiable();
  private static final Se2Wrap SE2WRAP = new Se2Wrap(Tensors.vectorDouble(1, 1, 2));

  public static TwdEntity createDuckie(StateTime stateTime) {
    TwdEntity twdEntity = new TwdEntity( //
        new TwdDuckieFlows(RealScalar.ONE, RealScalar.ONE), stateTime);
    twdEntity.extraCosts.add(Se2LateralAcceleration.COSTFUNCTION);
    return twdEntity;
  }

  public static TwdEntity createJ2B2(StateTime stateTime) {
    return new TwdEntity(new TwdForwardFlows(RealScalar.ONE, RealScalar.ONE), stateTime);
  }

  // ---
  final Collection<Flow> controls;
  final Scalar goalRadius_xy;
  final Scalar goalRadius_theta;

  /** @param twdConfig
   * @param stateTime initial position */
  protected TwdEntity(TwdFlows twdConfig, StateTime stateTime) {
    super(new SimpleEpisodeIntegrator(Se2StateSpaceModel.INSTANCE, Se2CarIntegrator.INSTANCE, stateTime));
    controls = twdConfig.getFlows(4);
    Tensor eta = eta();
    System.out.println("ETA = " + eta);
    goalRadius_xy = SQRT2.divide(eta.Get(0));
    goalRadius_theta = SQRT2.divide(eta.Get(2));
  }

  @Override
  protected Scalar distance(Tensor x, Tensor y) {
    return SE2WRAP.distance(x, y);
  }

  @Override
  public Scalar delayHint() {
    return RealScalar.ONE;
  }

  @Override
  public PlannerType getPlannerType() {
    return PlannerType.STANDARD;
  }

  @Override
  public TrajectoryPlanner createTrajectoryPlanner(TrajectoryRegionQuery obstacleQuery, Tensor goal) {
    this.obstacleQuery = obstacleQuery;
    StateIntegrator stateIntegrator = //
        FixedStateIntegrator.create(Se2CarIntegrator.INSTANCE, RationalScalar.of(1, 10), 4);
    Tensor radiusVector = Tensors.of(goalRadius_xy, goalRadius_xy, goalRadius_theta);
    GoalInterface goalInterface = MultiCostGoalAdapter.of( //
        Se2MinTimeGoalManager.create(goal, radiusVector, controls), //
        extraCosts);
    TrajectoryPlanner trajectoryPlanner = new StandardTrajectoryPlanner( //
        eta(), stateIntegrator, controls, obstacleQuery, goalInterface);
    trajectoryPlanner.represent = StateTimeTensorFunction.state(SE2WRAP::represent);
    return trajectoryPlanner;
  }

  @Override
  protected Tensor eta() {
    return PARTITIONSCALE;
  }

  @Override
  protected Tensor shape() {
    return SHAPE;
  }
}
