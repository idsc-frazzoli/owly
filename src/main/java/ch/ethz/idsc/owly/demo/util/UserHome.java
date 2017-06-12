// code by jph
package ch.ethz.idsc.owly.demo.util;

import java.io.File;

public enum UserHome {
  ;
  public static File file(String filename) {
    return new File(System.getProperty("user.home"), filename);
  }

  public static File Pictures(String filename) {
    return new File(file("Pictures"), filename);
  }
}
