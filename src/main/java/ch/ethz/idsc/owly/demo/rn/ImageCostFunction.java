// code by jph
package ch.ethz.idsc.owly.demo.rn;

import java.util.List;

import ch.ethz.idsc.owly.data.GlobalAssert;
import ch.ethz.idsc.owly.glc.core.CostIncrementFunction;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Array;
import ch.ethz.idsc.tensor.alg.Dimensions;
import ch.ethz.idsc.tensor.alg.MatrixQ;
import ch.ethz.idsc.tensor.alg.VectorQ;
import ch.ethz.idsc.tensor.sca.Floor;

public class ImageCostFunction implements CostIncrementFunction {
  private final Tensor image;
  private final List<Integer> dimensions;
  private final Tensor range;
  private final Tensor scale;
  private final Scalar outside;
  private final int max_y;

  /** @param image has to be a matrix
   * @param range effective size of image in coordinate space
   * @param outside point member status */
  public ImageCostFunction(Tensor image, Tensor range, Scalar outside) {
    GlobalAssert.that(MatrixQ.of(image));
    GlobalAssert.that(VectorQ.ofLength(range, 2));
    this.image = image;
    dimensions = Dimensions.of(image);
    max_y = dimensions.get(0) - 1;
    this.range = range;
    scale = Tensors.vector(dimensions.get(1), dimensions.get(0)).pmul(range.map(Scalar::reciprocal));
    this.outside = outside;
  }

  private Scalar _pointcost(Tensor tensor) {
    if (tensor.length() != 2)
      tensor = tensor.extract(0, 2);
    // TODO code redundant
    Tensor pixel = Floor.of(tensor.pmul(scale));
    int pix = pixel.Get(0).number().intValue();
    if (0 <= pix && pix < dimensions.get(1)) {
      int piy = max_y - pixel.Get(1).number().intValue();
      if (0 <= piy && piy < dimensions.get(0))
        return image.Get(piy, pix);
    }
    return outside;
  }

  public Tensor image() {
    return image.unmodifiable();
  }

  public Tensor range() {
    return range.unmodifiable();
  }

  public Tensor origin() {
    return Array.zeros(2);
  }

  @Override
  public Scalar costIncrement(GlcNode glcNode, List<StateTime> trajectory, Flow flow) {
    Scalar prev = glcNode.stateTime().time();
    Scalar sum = RealScalar.ZERO;
    for (StateTime stateTime : trajectory) {
      Scalar next = stateTime.time();
      Scalar dt = next.subtract(prev);
      sum = sum.add(_pointcost(stateTime.state()).multiply(dt));
      prev = next;
    }
    return sum;
  }

  public static void main(String[] args) {
  }
}
