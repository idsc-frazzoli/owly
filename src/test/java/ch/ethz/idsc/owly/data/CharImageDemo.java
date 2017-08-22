// code by jph
package ch.ethz.idsc.owly.data;

import java.awt.Dimension;
import java.awt.Font;
import java.awt.Point;
import java.awt.image.BufferedImage;
import java.io.IOException;

import javax.imageio.ImageIO;

import ch.ethz.idsc.owly.demo.util.UserHome;

enum CharImageDemo {
  ;
  // demo
  public static void main(String[] args) throws IOException {
    CharImage charImage = CharImage.fillWhite(new Dimension(640, 640));
    charImage.setFont(new Font(Font.DIALOG, Font.BOLD, 400));
    charImage.draw('G', new Point(0, 310));
    charImage.draw('T', new Point(280, 323));
    charImage.draw('I', new Point(480, 323));
    charImage.draw('O', new Point(20, 560));
    charImage.draw('B', new Point(280, 580));
    BufferedImage bufferedImage = charImage.bufferedImage();
    ImageIO.write(bufferedImage, "png", UserHome.Pictures("letter.png"));
  }
}
