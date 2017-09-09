// code by jph
package ch.ethz.idsc.owly.demo.rn.glc;

import java.io.IOException;
import java.util.Set;

import ch.ethz.idsc.owly.data.CharImage;
import ch.ethz.idsc.owly.demo.rn.R2ImageRegions;
import ch.ethz.idsc.owly.demo.util.FloodFill2D;
import ch.ethz.idsc.owly.demo.util.ImageCostFunction;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.CostFunction;
import ch.ethz.idsc.owly.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owly.math.region.ImageRegion;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Transpose;
import ch.ethz.idsc.tensor.io.ImageFormat;

/** demo shows the use of a cost image that is added to the distance cost
 * which gives an incentive to stay clear of obstacles */
enum R2ImageAnimationDemo {
  ;
  static CostFunction createImageCost() throws IOException {
    CharImage charImage = R2ImageRegions.inside_gtob_charImage();
    final Tensor tensor = Transpose.of(ImageFormat.from(charImage.bufferedImage()));
    Set<Tensor> seeds = FloodFill2D.seeds(tensor);
    final int ttl = 15; // magic const
    Tensor cost = FloodFill2D.of(seeds, RealScalar.of(ttl), tensor);
    return new ImageCostFunction(cost.divide(DoubleScalar.of(ttl)), Tensors.vector(12, 12), RealScalar.ZERO);
  }

  public static void main(String[] args) throws IOException {
    OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
    R2Entity r2Entity = new R2Entity(Tensors.vector(7, 6));
    r2Entity.costFunction = createImageCost();
    owlyAnimationFrame.set(r2Entity);
    ImageRegion imageRegion = R2ImageRegions.inside_gtob();
    owlyAnimationFrame.setObstacleQuery(new SimpleTrajectoryRegionQuery(new TimeInvariantRegion(imageRegion)));
    owlyAnimationFrame.addBackground(imageRegion);
    owlyAnimationFrame.configCoordinateOffset(50, 700);
    owlyAnimationFrame.jFrame.setVisible(true);
  }
}
