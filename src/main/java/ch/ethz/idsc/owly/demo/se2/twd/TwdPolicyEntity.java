// code by jph
package ch.ethz.idsc.owly.demo.se2.twd;

import java.util.Collection;

import ch.ethz.idsc.owly.demo.se2.Se2CarIntegrator;
import ch.ethz.idsc.owly.demo.se2.Se2StateSpaceModel;
import ch.ethz.idsc.owly.demo.util.LidarEmulator;
import ch.ethz.idsc.owly.gui.ani.PolicyEntity;
import ch.ethz.idsc.owly.math.Degree;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.SimpleEpisodeIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Array;
import ch.ethz.idsc.tensor.alg.Ordering;
import ch.ethz.idsc.tensor.alg.Range;
import ch.ethz.idsc.tensor.alg.Subdivide;
import ch.ethz.idsc.tensor.lie.Permutations;
import ch.ethz.idsc.tensor.sca.Rationalize;

public class TwdPolicyEntity extends PolicyEntity {
  private final Tensor states = Permutations.of(Range.of(0, 5)).unmodifiable();
  private final Tensor actions;
  private LidarEmulator lidarEmulator;

  public TwdPolicyEntity(StateTime stateTime, TrajectoryRegionQuery raytraceQuery) {
    super(new SimpleEpisodeIntegrator( //
        Se2StateSpaceModel.INSTANCE, //
        Se2CarIntegrator.INSTANCE, //
        stateTime));
    TwdFlows twdFlows = new TwdForwardFlows(RealScalar.ONE, RealScalar.ONE);
    Collection<Flow> collection = twdFlows.getFlows(5);
    actions = Tensor.of(collection.stream() //
        .map(Flow::getU) //
        .map(u -> Rationalize.of(u, 10)) //
    ).unmodifiable();
    lidarEmulator = new LidarEmulator( //
        Subdivide.of(Degree.of(+90), Degree.of(-90), 4), this::getStateTimeNow, raytraceQuery);
  }

  @Override
  protected Tensor fallbackControl() {
    return Array.zeros(3);
  }

  @Override
  public Tensor represent(StateTime stateTime) {
    Tensor range = lidarEmulator.detectRange(stateTime);
    return Tensors.vectorInt(Ordering.DECREASING.of(range));
  }

  @Override // from DiscreteModel
  public Tensor states() {
    return states;
  }

  @Override // from DiscreteModel
  public Tensor actions(Tensor state) {
    return actions;
  }

  @Override // from DiscreteModel
  public Scalar gamma() {
    return RealScalar.ONE;
  }
}
