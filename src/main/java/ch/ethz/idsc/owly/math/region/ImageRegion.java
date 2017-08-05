// code by jph
package ch.ethz.idsc.owly.math.region;

import java.util.List;

import ch.ethz.idsc.owly.data.GlobalAssert;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Dimensions;
import ch.ethz.idsc.tensor.alg.MatrixQ;
import ch.ethz.idsc.tensor.sca.Floor;

/** only the first two coordinates are tested for membership
 * a location is available if the grayscale value of the pixel equals 0 */
public class ImageRegion implements Region {
  private final Tensor image;
  private final List<Integer> dimensions;
  private final Tensor range;
  private final Tensor scale;
  private final boolean outside;

  /** @param image has to be a matrix
   * @param range effective size of image in coordinate space
   * @param outside point member status */
  public ImageRegion(Tensor image, Tensor range, boolean outside) {
    GlobalAssert.that(MatrixQ.of(image));
    this.image = image;
    dimensions = Dimensions.of(image);
    this.range = range;
    scale = Tensors.vector(dimensions).pmul(range.map(Scalar::reciprocal));
    this.outside = outside;
  }

  @Override
  public boolean isMember(Tensor tensor) {
    if (tensor.length() != 2)
      tensor = tensor.extract(0, 2);
    Tensor pixel = Floor.of(tensor.pmul(scale));
    int pix = pixel.Get(0).number().intValue();
    int piy = pixel.Get(1).number().intValue();
    if (0 <= pix && pix < dimensions.get(0) && 0 <= piy && piy < dimensions.get(1))
      return Scalars.nonZero(image.Get(pix, piy));
    return outside;
  }

  public Tensor image() {
    return image.unmodifiable();
  }

  public Tensor range() {
    return range.unmodifiable();
  }
}
