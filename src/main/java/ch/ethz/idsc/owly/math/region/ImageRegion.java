// code by jph
package ch.ethz.idsc.owly.math.region;

import java.util.List;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Dimensions;
import ch.ethz.idsc.tensor.alg.TensorRank;
import ch.ethz.idsc.tensor.sca.Floor;

/** only the first two coordinates are tested for membership
 * a location is available if the grayscale value of the pixel equals 0 */
public class ImageRegion implements Region {
  private final Tensor image;
  private final List<Integer> dimensions;
  private final Tensor scale;
  private final boolean outside;

  /** @param image has to be a matrix
   * @param range effective size of image in coordinate space
   * @param outside point member status */
  public ImageRegion(Tensor image, Tensor range, boolean outside) {
    if (TensorRank.of(image) != 2)
      throw new RuntimeException();
    this.image = image;
    dimensions = Dimensions.of(image);
    scale = Tensors.vector(dimensions).pmul(range.map(Scalar::invert));
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
}
