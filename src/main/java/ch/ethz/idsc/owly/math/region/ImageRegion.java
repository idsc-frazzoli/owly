// code by jph
package ch.ethz.idsc.owly.math.region;

import java.util.List;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.ZeroScalar;
import ch.ethz.idsc.tensor.alg.Dimensions;
import ch.ethz.idsc.tensor.sca.Floor;

/** TODO describe conventions of {@link ImageRegion} */
public class ImageRegion implements Region {
  private final Tensor image;
  private final List<Integer> dimensions;
  private final Tensor scale;

  /** @param image has to be a matrix
   * @param range */
  public ImageRegion(Tensor image, Tensor range) {
    this.image = image;
    dimensions = Dimensions.of(image);
    scale = Tensors.vector(dimensions).pmul(range.map(Scalar::invert));
  }

  // TODO not sure if coordinates should be "rotated"
  @Override
  public boolean isMember(Tensor tensor) {
    if (tensor.length() != 2)
      tensor = tensor.extract(0, 2);
    Tensor pixel = Floor.of(tensor.pmul(scale));
    int pix = pixel.Get(0).number().intValue();
    int piy = pixel.Get(1).number().intValue();
    if (0 <= pix && pix < dimensions.get(0) && 0 <= piy && piy < dimensions.get(1))
      return !image.Get(pix, piy).equals(ZeroScalar.get());
    return false;
  }
}
