// code by jph
package ch.ethz.idsc.owly.demo.util;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.zip.DataFormatException;

import ch.ethz.idsc.owly.math.region.ImageRegion;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.alg.TensorRank;
import ch.ethz.idsc.tensor.io.ImageFormat;
import ch.ethz.idsc.tensor.io.Import;

public enum ImageRegions {
  ;
  // for files in repo
  public static ImageRegion loadFromRepository(String string, Tensor range, boolean strict) throws ClassNotFoundException, DataFormatException, IOException {
    return loadFromLocalFile(new File("".getClass().getResource(string).getPath()), range, strict);
  }

  // for files on local machine
  public static ImageRegion loadFromLocalFile(File file, Tensor range, boolean strict) throws ClassNotFoundException, DataFormatException, IOException {
    Tensor image = Import.of(file);
    if (TensorRank.of(image) == 3)
      image = image.get(Tensor.ALL, Tensor.ALL, 0);
    return new ImageRegion(image, range, strict);
  }

  public static ImageRegion fromGrayscale(BufferedImage bufferedImage, Tensor range, boolean strict) {
    return new ImageRegion( //
        Images.displayOrientation(ImageFormat.from(bufferedImage)), range, strict);
  }
}
