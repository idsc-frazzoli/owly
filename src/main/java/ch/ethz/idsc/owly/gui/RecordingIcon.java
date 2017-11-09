// code by jph
package ch.ethz.idsc.owly.gui;

import javax.swing.Icon;
import javax.swing.ImageIcon;

import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.img.ArrayPlot;
import ch.ethz.idsc.tensor.img.ColorDataFunction;
import ch.ethz.idsc.tensor.img.ColorDataGradients;
import ch.ethz.idsc.tensor.io.ImageFormat;
import ch.ethz.idsc.tensor.opt.GaussianMatrix;

public enum RecordingIcon {
  ;
  public static Icon standard() {
    ColorDataFunction colorDataFunction = ColorDataGradients.THERMOMETER;
    Tensor matrix = GaussianMatrix.of(7);
    matrix = matrix.map(scalar -> Scalars.lessThan(RealScalar.of(0.001), scalar) ? scalar : DoubleScalar.INDETERMINATE);
    Tensor image = ArrayPlot.of(matrix, colorDataFunction);
    // image = image.block(Arrays.asList(0, 0, 0), Arrays.asList(16, 16, 4));
    // image = ArrayPad.of(image, Arrays.asList(0, 0, 0), Arrays.asList(1, 1, 0));
    // System.out.println(Dimensions.of(image));
    return new ImageIcon(ImageFormat.of(image));
  }

  public static void main(String[] args) {
    standard();
  }
}
