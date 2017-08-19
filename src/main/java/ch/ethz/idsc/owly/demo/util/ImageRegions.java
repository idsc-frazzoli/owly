// code by jph
package ch.ethz.idsc.owly.demo.util;

import java.io.File;
import java.io.IOException;
import java.util.zip.DataFormatException;

import ch.ethz.idsc.owly.math.region.ImageRegion;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.alg.TensorRank;
import ch.ethz.idsc.tensor.io.Import;
import ch.ethz.idsc.tensor.io.ResourceData;

public enum ImageRegions {
  ;
  private static ImageRegion _universal(Tensor image, Tensor range, boolean strict) {
    if (TensorRank.of(image) == 3)
      image = image.get(Tensor.ALL, Tensor.ALL, 0);
    return new ImageRegion(image, range, strict);
  }

  // for files in repo
  public static ImageRegion loadFromRepository(String string, Tensor range, boolean strict) throws ClassNotFoundException, DataFormatException, IOException {
    return _universal(ResourceData.of(string), range, strict);
  }

  // for files on local machine
  public static ImageRegion loadFromLocalFile(File file, Tensor range, boolean strict) throws ClassNotFoundException, DataFormatException, IOException {
    return _universal(Import.of(file), range, strict);
  }
}
