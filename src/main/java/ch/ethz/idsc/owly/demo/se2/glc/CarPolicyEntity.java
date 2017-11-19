// code by jph
package ch.ethz.idsc.owly.demo.se2.glc;

import java.awt.Graphics2D;
import java.awt.geom.Point2D;
import java.util.LinkedList;
import java.util.List;
import java.util.Objects;

import ch.ethz.idsc.owly.demo.se2.Se2CarIntegrator;
import ch.ethz.idsc.owly.demo.se2.Se2StateSpaceModel;
import ch.ethz.idsc.owly.demo.util.LidarEmulator;
import ch.ethz.idsc.owly.gui.GeometricLayer;
import ch.ethz.idsc.owly.gui.ani.PolicyEntity;
import ch.ethz.idsc.owly.math.Degree;
import ch.ethz.idsc.owly.math.r2.CirclePoints;
import ch.ethz.idsc.owly.math.state.SimpleEpisodeIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.subare.core.Policy;
import ch.ethz.idsc.subare.core.RewardInterface;
import ch.ethz.idsc.subare.core.StepInterface;
import ch.ethz.idsc.subare.core.adapter.StepAdapter;
import ch.ethz.idsc.subare.core.util.PolicyWrap;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Ordering;
import ch.ethz.idsc.tensor.alg.Subdivide;
import ch.ethz.idsc.tensor.sca.Chop;

public class CarPolicyEntity extends PolicyEntity implements RewardInterface {
  private final Tensor SHAPE = CirclePoints.of(5).multiply(RealScalar.of(.1));
  // ---
  private LidarEmulator lidarEmulator;
  private final CarDiscreteModel carDiscreteModel;
  private final Policy policy;

  public CarPolicyEntity(CarDiscreteModel carDiscreteModel, Policy policy, StateTime stateTime, TrajectoryRegionQuery raytraceQuery) {
    super(new SimpleEpisodeIntegrator(Se2StateSpaceModel.INSTANCE, Se2CarIntegrator.INSTANCE, stateTime));
    this.carDiscreteModel = carDiscreteModel;
    this.policy = policy;
    // ---
    lidarEmulator = new LidarEmulator( //
        Subdivide.of(Degree.of(+90), Degree.of(-90), carDiscreteModel.resolution - 1), //
        this::getStateTimeNow, raytraceQuery);
    SHAPE.set(Tensors.vector(.2, 0), 0);
  }

  private final List<StepInterface> episodeLog = new LinkedList<>();
  private Tensor prev_state;
  private Tensor prev_action;
  private Scalar prev_reward;

  @Override // from AnimationInterface
  public final void integrate(Scalar now) {
    // implementation does not require that current position is perfectly located on trajectory
    // Tensor u = fallbackControl(); // default control
    StateTime stateTime = getStateTimeNow();
    Tensor state = represent(stateTime); // may be augmented state time, and/or observation etc.
    if (Objects.nonNull(prev_state) && !carDiscreteModel.isTerminal(prev_state)) {
      episodeLog.add(new StepAdapter(prev_state, prev_action, prev_reward, state));
    }
    // <- compute reward based on prev_state,
    if (Objects.nonNull(prev_state))
      prev_reward = reward(prev_state, prev_action, state);
    // System.out.println(episodeLog.size() + " " + prev_reward);
    prev_state = state;
    // System.out.println(state);
    PolicyWrap policyWrap = new PolicyWrap(policy);
    Tensor actions = carDiscreteModel.actions(state);
    prev_action = policyWrap.next(state, actions);
    episodeIntegrator.move(prev_action, now);
  }

  @Override
  public Tensor represent(StateTime stateTime) {
    Tensor range = lidarEmulator.detectRange(stateTime);
    if (Chop.NONE.allZero(range))
      return CarDiscreteModel.COLLISION;
    return Tensors.vectorInt(Ordering.DECREASING.of(range));
  }

  @Override // from RewardInterface
  public Scalar reward(Tensor state, Tensor action, Tensor next) {
    return !carDiscreteModel.isTerminal(state) && carDiscreteModel.isTerminal(next) //
        ? RealScalar.of(-1)
        : RealScalar.ZERO;
  }

  @Override
  protected Tensor shape() {
    return SHAPE;
  }

  @Override
  public void render(GeometricLayer geometricLayer, Graphics2D graphics) {
    super.render(geometricLayer, graphics);
    {
      StateTime stateTime = getStateTimeNow();
      Point2D p = geometricLayer.toPoint2D(stateTime.state());
      graphics.drawString(":-)", (int) p.getX(), (int) p.getY());
    }
  }
}
