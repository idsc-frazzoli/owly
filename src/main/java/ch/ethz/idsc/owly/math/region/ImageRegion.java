// code by jph
package ch.ethz.idsc.owly.math.region;

import java.util.List;

import ch.ethz.idsc.owly.data.GlobalAssert;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Array;
import ch.ethz.idsc.tensor.alg.Dimensions;
import ch.ethz.idsc.tensor.alg.MatrixQ;
import ch.ethz.idsc.tensor.alg.VectorQ;
import ch.ethz.idsc.tensor.sca.Floor;

/** only the first two coordinates are tested for membership
 * a location is available if the grayscale value of the pixel equals 0 */
public class ImageRegion implements Region<Tensor> {
  private final Tensor image;
  private final List<Integer> dimensions;
  private final Tensor range;
  private final Tensor scale;
  private final boolean outside;
  private final int max_y;

  /** @param image has to be a matrix
   * @param range effective size of image in coordinate space
   * @param outside point member status */
  public ImageRegion(Tensor image, Tensor range, boolean outside) {
    GlobalAssert.that(MatrixQ.of(image));
    GlobalAssert.that(VectorQ.ofLength(range, 2));
    this.image = image;
    dimensions = Dimensions.of(image);
    max_y = dimensions.get(0) - 1;
    this.range = range;
    scale = Tensors.vector(dimensions.get(1), dimensions.get(0)).pmul(range.map(Scalar::reciprocal));
    this.outside = outside;
  }

  @Override
  public boolean isMember(Tensor tensor) {
    if (tensor.length() != 2)
      tensor = tensor.extract(0, 2);
    Tensor pixel = Floor.of(tensor.pmul(scale));
    int pix = pixel.Get(0).number().intValue();
    if (0 <= pix && pix < dimensions.get(1)) {
      int piy = max_y - pixel.Get(1).number().intValue();
      if (0 <= piy && piy < dimensions.get(0))
        return Scalars.nonZero(image.Get(piy, pix));
    }
    return outside;
  }

  public Tensor image() {
    return image.unmodifiable();
  }

  public Tensor range() {
    return range.unmodifiable();
  }

  public Tensor scale() {
    return scale.unmodifiable();
  }

  public Tensor origin() {
    return Array.zeros(2);
  }
}
