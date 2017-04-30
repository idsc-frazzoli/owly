// code by jph
package ch.ethz.idsc.owly.demo.util;

import java.io.File;
import java.io.IOException;
import java.util.zip.DataFormatException;

import ch.ethz.idsc.owly.math.ImageRegion;
import ch.ethz.idsc.owly.math.Region;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.alg.TensorRank;
import ch.ethz.idsc.tensor.io.Import;

public class ImageRegions {
  public static Region load(String string, Tensor range) throws ClassNotFoundException, DataFormatException, IOException {
    String path = "".getClass().getResource(string).getPath();
    Tensor image = Import.of(new File(path));
    if (TensorRank.of(image) == 3)
      image = image.get(Tensor.ALL, Tensor.ALL, 0);
    return new ImageRegion(image, range);
  }
}
