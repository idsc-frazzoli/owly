// code by jph
package ch.ethz.idsc.owly.demo.se2.twd;

import java.awt.Graphics2D;
import java.awt.geom.Point2D;
import java.util.Collection;

import ch.ethz.idsc.owly.demo.se2.Se2CarIntegrator;
import ch.ethz.idsc.owly.demo.se2.Se2StateSpaceModel;
import ch.ethz.idsc.owly.demo.util.LidarEmulator;
import ch.ethz.idsc.owly.gui.GeometricLayer;
import ch.ethz.idsc.owly.gui.ani.PolicyEntity;
import ch.ethz.idsc.owly.math.CirclePoints;
import ch.ethz.idsc.owly.math.Degree;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.SimpleEpisodeIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.subare.core.adapter.FixedRandomPolicy;
import ch.ethz.idsc.subare.core.util.EquiprobablePolicy;
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

public class CarPolicyEntity extends PolicyEntity {
  private static final int RES = 5;
  private final Tensor SHAPE = CirclePoints.of(5).multiply(RealScalar.of(.1));
  // ---
  private final Tensor states = Permutations.of(Range.of(0, RES)).unmodifiable();
  private final Tensor actions;
  private LidarEmulator lidarEmulator;

  public CarPolicyEntity(StateTime stateTime, TrajectoryRegionQuery raytraceQuery) {
    super(new SimpleEpisodeIntegrator(Se2StateSpaceModel.INSTANCE, Se2CarIntegrator.INSTANCE, stateTime));
    // ---
    TwdFlows twdFlows = new TwdForwardFlows(RealScalar.of(.3), RealScalar.ONE);
    Collection<Flow> collection = twdFlows.getFlows(6);
    actions = Tensor.of(collection.stream() //
        .map(Flow::getU) //
        .map(u -> Rationalize.of(u, 100)) //
    ).unmodifiable();
    // ---
    lidarEmulator = new LidarEmulator( //
        Subdivide.of(Degree.of(+90), Degree.of(-90), RES - 1), this::getStateTimeNow, raytraceQuery);
    policy = new EquiprobablePolicy(this);
    policy = new FixedRandomPolicy(this);
    SHAPE.set(Tensors.vector(.2, 0), 0);
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
      graphics.drawString("here", (int) p.getX(), (int) p.getY());
    }
  }
}
