// code by jph
package ch.ethz.idsc.owly.math.region;

import java.util.Iterator;
import java.util.List;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.Ramp;

//TODO TEST: write Test
/** evaluate does not correspond to Euclidean distance */
public class EllipsoidListRegion extends ImplicitFunctionRegion {
  protected final List<Tensor> centers;
  private final Tensor invert;

  /** @param centers of the Ellipsoid
   * @param radius of the different axes.
   * all components of radius must be non-negative.
   * if a component of radius is RealScalar.POSITIVE_INFINITY, this corresponds to a Cylinder */
  public EllipsoidListRegion(List<Tensor> centers, Tensor radius) {
    this.centers = centers;
    if (!radius.equals(Ramp.of(radius))) // assert that radius are non-negative
      throw new RuntimeException();
    invert = radius.map(Scalar::invert); // throws an exception if any radius == 0
  }

  /** @param Tensor to test if in Obstacle
   * @return -1 if in Obstacle, 0 if touching Obstacle, 1 if not touching obstacle */
  @Override
  public Scalar evaluate(Tensor tensor) {
    // TODO needs math derivation
    Iterator<Tensor> iterator = centers.iterator();
    Scalar test_value = RealScalar.ONE; // returns 1 if obstacle is non existent
    boolean touching = false;
    while (iterator.hasNext()) {
      Tensor delta = iterator.next().subtract(tensor).pmul(invert);
      test_value = Norm._2.of(delta).subtract(RealScalar.ONE);
      if (Scalars.lessThan(test_value, RealScalar.ZERO))
        return RealScalar.ONE.negate(); //as soon as collision is detected return -1
      if (Scalars.isZero(test_value))
        touching = true; // touching is saved for later, as inside obstacle is more significant
    }
    if (touching)
      return RealScalar.ZERO; //tensor is not in obstacle, but touching it
    return RealScalar.ONE;
  }
}
