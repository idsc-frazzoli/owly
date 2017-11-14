// code by jph
package ch.ethz.idsc.owly.demo.rn.glc;

import java.util.Collection;
import java.util.LinkedList;

import ch.ethz.idsc.owly.demo.rn.R2Config;
import ch.ethz.idsc.owly.demo.rn.RnMinDistSphericalGoalManager;
import ch.ethz.idsc.owly.glc.adapter.MultiCostGoalAdapter;
import ch.ethz.idsc.owly.glc.core.CostFunction;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.glc.core.StandardTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.ani.AbstractCircularEntity;
import ch.ethz.idsc.owly.math.SingleIntegratorStateSpaceModel;
import ch.ethz.idsc.owly.math.flow.EulerIntegrator;
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
import ch.ethz.idsc.tensor.red.Norm2Squared;

/** omni-directional movement with constant speed
 * 
 * the implementation chooses certain values */
/* package */ class R2Entity extends AbstractCircularEntity {
  /** radius of spherical goal region */
  private final Scalar goalRadius = RealScalar.of(0.2);
  /** extra cost functions, for instance to prevent cutting corners */
  public final Collection<CostFunction> extraCosts = new LinkedList<>();

  /** @param state initial position of entity */
  public R2Entity(Tensor state) {
    super(new SimpleEpisodeIntegrator( //
        SingleIntegratorStateSpaceModel.INSTANCE, //
        EulerIntegrator.INSTANCE, //
        new StateTime(state, RealScalar.ZERO)));
  }

  @Override
  protected Scalar distance(Tensor x, Tensor y) {
    return Norm2Squared.between(x, y);
  }

  @Override
  protected final Tensor fallbackControl() {
    return Tensors.vectorDouble(0, 0).unmodifiable();
  }

  @Override
  public Scalar delayHint() {
    /** preserve 0.5[s] of the former trajectory
     * planning should not exceed that duration, otherwise
     * the entity may not be able to follow a planned trajectory */
    return RealScalar.of(0.5);
  }

  @Override
  public TrajectoryPlanner createTrajectoryPlanner(TrajectoryRegionQuery obstacleQuery, Tensor goal) {
    Tensor partitionScale = eta();
    StateIntegrator stateIntegrator = //
        FixedStateIntegrator.create(EulerIntegrator.INSTANCE, RationalScalar.of(1, 12), 4);
    final Tensor center = goal.extract(0, 2);
    GoalInterface goalInterface = //
        MultiCostGoalAdapter.of(RnMinDistSphericalGoalManager.create(center, goalRadius), extraCosts);
    return new StandardTrajectoryPlanner( //
        partitionScale, stateIntegrator, createControls(), obstacleQuery, goalInterface);
  }

  Collection<Flow> createControls() {
    /** 36 corresponds to 10[Degree] resolution */
    R2Config r2Config = new R2Config(RealScalar.ONE);
    return r2Config.createRadial(36);
  }

  protected Tensor eta() {
    return Tensors.vector(8, 8);
  }
}
