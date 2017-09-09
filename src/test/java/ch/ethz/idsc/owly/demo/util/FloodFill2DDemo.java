// code by jph
package ch.ethz.idsc.owly.demo.util;

import java.io.IOException;
import java.util.Set;

import ch.ethz.idsc.owly.data.CharImage;
import ch.ethz.idsc.owly.data.Stopwatch;
import ch.ethz.idsc.owly.demo.rn.R2ImageRegions;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.io.Export;
import ch.ethz.idsc.tensor.io.ImageFormat;

enum FloodFill2DDemo {
  ;
  public static void main(String[] args) throws IOException {
    CharImage charImage = R2ImageRegions.inside_gtob_charImage();
    final Tensor tensor = ImageFormat.from(charImage.bufferedImage());
    Export.of(UserHome.Pictures("image.png"), tensor);
    Set<Tensor> seeds = FloodFill2D.seeds(tensor);
    Scalar ttl = RealScalar.of(30);
    Stopwatch stopwatch = Stopwatch.started();
    Tensor cost = FloodFill2D.of(seeds, ttl, tensor);
    System.out.println("floodfill " + stopwatch.display_seconds());
    Export.of(UserHome.Pictures("image_cost.png"), cost);
  }
}
