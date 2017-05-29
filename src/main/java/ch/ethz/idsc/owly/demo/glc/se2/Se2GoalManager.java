// code by jph
package ch.ethz.idsc.owly.demo.glc.se2;

import java.util.ArrayList;
import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.math.SignedCurvature2D;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.state.CostFunction;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.Trajectories;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.ZeroScalar;
import ch.ethz.idsc.tensor.red.Max;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.Floor;
import ch.ethz.idsc.tensor.sca.Mod;
import ch.ethz.idsc.tensor.sca.Power;

/** Se2 goal region is not elliptic, therefore we implement {@link Region} */
public class Se2GoalManager implements Region, CostFunction {
  static final Mod PRINCIPAL = Mod.function(RealScalar.of(2 * Math.PI), RealScalar.of(-Math.PI));
  // ---
  final Tensor xy;
  final Scalar angle;
  final Scalar radius;
  final Scalar angle_delta;

  public Se2GoalManager(Tensor xy, Scalar angle, Scalar radius, Scalar angle_delta) {
    this.xy = xy;
    this.angle = angle;
    this.radius = radius;
    this.angle_delta = angle_delta;
  }

  @Override
  /** Cost Function */
  public Scalar costIncrement(StateTime from, List<StateTime> trajectory, Flow flow) {
    int endIndex = trajectory.size() - 1;
    if (endIndex < 3) // can not calculated curvature with 2 points
      throw new RuntimeException();
    int middleIndex = Floor.of(RealScalar.of(endIndex / 2)).Get().number().intValue();
    List<Integer> indices1 = new ArrayList<Integer>();
    List<Integer> indices2 = new ArrayList<Integer>();
    indices1.add(0);
    indices2.add(2);
    Tensor a = from.x().block(indices1, indices2);
    Tensor b = trajectory.get(middleIndex).x().block(indices1, indices2);
    Tensor c = trajectory.get(endIndex).x().block(indices1, indices2);
    Scalar curvature = SignedCurvature2D.of(a, b, c);
    // TODO Tensor libary needs to be fixed Power(0,1) = 0 NOT 1
    if (curvature.equals(RealScalar.of(0)))
      return RealScalar.ONE.multiply(Trajectories.timeIncrement(from, trajectory));
    return (RealScalar.ONE.add //
    (Power.of(curvature.abs(), 2))).multiply(Trajectories.timeIncrement(from, trajectory));
    // integrate(1,t)
    // return Trajectories.timeIncrement(from, trajectory);
  }

  @Override
  /** Heuristic function */
  public Scalar minCostToGoal(Tensor x) {
    Tensor cur_xy = x.extract(0, 2);
    Scalar cur_angle = x.Get(2);
    Scalar dxy = Norm._2.of(cur_xy.subtract(xy)).subtract(radius);
    // Scalar dangle = PRINCIPAL.apply(cur_angle.subtract(angle)).abs().subtract(angle_delta);
    return Max.of(dxy, ZeroScalar.get());
    // return Max.of(Norm._2.of(tensor.subtract(center)).subtract(radius), ZeroScalar.get());
    // return ZeroScalar.get();
  }

  @Override
  public boolean isMember(Tensor tensor) {
    Tensor cur_xy = tensor.extract(0, 2);
    Scalar cur_angle = tensor.Get(2);
    boolean status = true;
    status &= Scalars.lessEquals(Norm._2.of(cur_xy.subtract(xy)), radius);
    status &= Scalars.lessEquals(PRINCIPAL.apply(cur_angle.subtract(angle)).abs(), angle_delta);
    return status;
  }

  public TrajectoryRegionQuery goalQuery() {
    return new SimpleTrajectoryRegionQuery(new TimeInvariantRegion(this));
  }
}