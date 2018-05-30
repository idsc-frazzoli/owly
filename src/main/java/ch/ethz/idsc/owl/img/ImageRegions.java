// code by jph
package ch.ethz.idsc.owl.img;

import java.io.File;
import java.io.IOException;

import ch.ethz.idsc.owl.math.region.ImageRegion;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.TensorRank;
import ch.ethz.idsc.tensor.io.Import;
import ch.ethz.idsc.tensor.io.ResourceData;

public enum ImageRegions {
  ;
  private static ImageRegion _universal(Tensor image, Tensor range, int channel, boolean strict) {
    if (TensorRank.of(image) == 3) // the rank of images with a color palette is 3
      image = image.get(Tensor.ALL, Tensor.ALL, channel); 
    return new ImageRegion(image, range, strict);
  }

  // for files in repo
  public static ImageRegion loadFromRepository(String string, Tensor range, boolean strict) {
    return _universal(ResourceData.of(string), range, 0, strict); //use RED channel for obs check
  }
  
  // for files in repo
  public static ImageRegion loadFromRepository(String string, Tensor range, int channel, boolean strict) {
    return _universal(ResourceData.of(string), range, channel, strict);
  }
  

  // for files on local machine
  public static ImageRegion loadFromLocalFile(File file, Tensor range, boolean strict) throws IOException {
    return _universal(Import.of(file), range, 0, strict);
  }
}
