/**
 * This file is part of LibLaserCut.
 * Copyright (C) 2011 - 2014 Thomas Oster <mail@thomas-oster.de>
 *
 * LibLaserCut is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * LibLaserCut is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with LibLaserCut. If not, see <http://www.gnu.org/licenses/>.
 *
 **/
package com.t_oster.liblasercut.drivers;

import com.t_oster.liblasercut.*;
import com.t_oster.liblasercut.platform.Point;
import com.t_oster.liblasercut.platform.Util;
import java.io.BufferedOutputStream;
import java.io.BufferedInputStream;
import java.io.ByteArrayOutputStream;
import java.io.PrintStream;
import java.io.UnsupportedEncodingException;
import java.util.*;
import purejavacomm.CommPort;
import purejavacomm.CommPortIdentifier;
import purejavacomm.SerialPort;

/**
 * This class implements a driver for  Grbl-based platforms.
 *
 * https://github.com/grbl/grbl
 *
 * grbl is a common GCODE parser for arduino, smoothieboard, etc.
 * Currently the driver supports basic G-Code and provides a mechanism for the
 * user to supply pre and post job G-Code instructions (home, absolute
 * coordinates, etc).
 *
 * This driver was written specifically for the Smoothieboard controller but can
 * be used for any Grbl-based USB connected platforms.
 *
 * http://smoothieware.org
 *
 * dmitchell.ml@gmail.com
 *
 * Based on the Lasersaur driver written by:
 * @author Thomas Oster <thomas.oster@rwth-aachen.de>
 */
public class Grbl extends LaserCutter {

  private static final String SETTING_COMPORT = "COM-Port";
  private static final String SETTING_COMBAUD = "COM-Port baud rate";
  private static final String SETTING_BEDWIDTH = "Laserbed width";
  private static final String SETTING_BEDHEIGHT = "Laserbed height";
  private static final String SETTING_FLIPX = "X axis goes right to left (yes/no)";
  private static final String SETTING_RASTER_WHITESPACE = "Additional space per Raster line (mm)";
  private static final String SETTING_SEEK_RATE = "Max. Seek Rate (mm/min)";
  private static final String SETTING_LASER_RATE = "Max. Laser Rate (mm/min)";
  private static final String SETTING_JOB_PRE_GCODE = "G-Code to send before each job (use ; between commands)";
  private static final String SETTING_JOB_POST_GCODE = "G-Code to send after each job (use ; between commands)";
  private static final int UART_BYTE_CHUNK_SIZE = 8;
  private static final int MAX_CTS_LOOPS = 100;

  @Override
  public String getModelName() {
    return "Grbl";
  }

  private double addSpacePerRasterLine = 0.5;

  /**
   * Get the value of addSpacePerRasterLine
   *
   * @return the value of addSpacePerRasterLine
   */
  public double getAddSpacePerRasterLine() {
    return addSpacePerRasterLine;
  }

  /**
   * Set the value of addSpacePerRasterLine
   *
   * @param addSpacePerRasterLine new value of addSpacePerRasterLine
   */
  public void setAddSpacePerRasterLine(double addSpacePerRasterLine) {
    this.addSpacePerRasterLine = addSpacePerRasterLine;
  }

  private double seekRate = 2000;

  /**
   * Get the value of seekRate
   *
   * @return the value of seekRate
   */
  public double getSeekRate() {
    return seekRate;
  }

  /**
   * Set the value of seekRate
   *
   * @param seekRate new value of seekRate
   */
  public void setSeekRate(double seekRate) {
    this.seekRate = seekRate;
  }

  private double laserRate = 2000;

  /**
   * Get the value of laserRate
   *
   * @return the value of laserRate
   */
  public double getLaserRate() {
    return laserRate;
  }

  /**
   * Set the value of laserRate
   *
   * @param laserRate new value of laserRate
   */
  public void setLaserRate(double laserRate) {
    this.laserRate = laserRate;
  }

  protected boolean flipXaxis = false;

  /**
   * Get the value of flipXaxis
   *
   * @return the value of flipXaxis
   */
  public boolean isFlipXaxis() {
    return flipXaxis;
  }

  /**
   * Set the value of flipXaxis
   *
   * @param flipXaxis new value of flipXaxis
   */
  public void setFlipXaxis(boolean flipXaxis) {
    this.flipXaxis = flipXaxis;
  }

  protected String comPort = "ttyACM0";

  /**
   * Get the value of port
   *
   * @return the value of port
   */
  public String getComPort() {
    return comPort;
  }

  /**
   * Set the value of port
   *
   * @param comPort new value of port
   */
  public void setComPort(String comPort) {
    this.comPort = comPort;
  }

  protected int comBaud = 115200;

  /**
   * Get the baud rate of the port
   *
   * @return the baud rate of the port
   */
  public int getComBaud() {
    return comBaud;
  }

  /**
   * Set the baud rate of the port
   *
   * @param comBaud new value of port
   */
  public void setComBaud(int comBaud) {
    this.comBaud = comBaud;
  }

  protected String jobPreGCode = "G28;G21;G90";

  /**
   * Set the pre-job g-code string
   *
   * @param preGCode g-code string for pre-job init
   */
  public void setJobPreGCode(String preGCode) {
    this.jobPreGCode=preGCode;
  }

  /**
   * Get the pre-job g-code string
   *
   * @return the pre-job g-code string
   */
  public String getJobPreGCode() {
    return jobPreGCode;
  }

  protected String jobPostGCode = "G28";

  /**
   * Set the post-job g-code string
   *
   * @param postGCode g-code string for post-job restore
   */
  public void setJobPostGCode(String postGCode) {
    this.jobPostGCode = postGCode;
  }

  /**
   * Get the post-job g-code string
   *
   * @return the post-job g-code string
   */
  public String getJobPostGCode() {
    return jobPostGCode;
  }

  private byte[] generateVectorGCode(VectorPart vp, double resolution) throws UnsupportedEncodingException {
    ByteArrayOutputStream result = new ByteArrayOutputStream();
    PrintStream out = new PrintStream(result, true, "US-ASCII");
    for (VectorCommand cmd : vp.getCommandList()) {
      switch (cmd.getType()) {
        case MOVETO:
          int x = cmd.getX();
          int y = cmd.getY();
          move(out, x, y, resolution);
          break;
        case LINETO:
          x = cmd.getX();
          y = cmd.getY();
          line(out, x, y, resolution);
          break;
        case SETPROPERTY:
          PowerSpeedFocusFrequencyProperty p = (PowerSpeedFocusFrequencyProperty) cmd.getProperty();
          setPower(out, p.getPower());
          setSpeed(out, p.getSpeed());
          break;
      }
    }
    return result.toByteArray();
  }
  private int currentPower = -1;
  private int currentSpeed = -1;

  private void setSpeed(PrintStream out, int speedInPercent) {
    if (speedInPercent != currentSpeed) {
      out.printf(Locale.US, "G1 F%d\n", (int) ((double) speedInPercent * this.getLaserRate() / 100));
      currentSpeed = speedInPercent;
    }
  }

  private void setPower(PrintStream out, int powerInPercent) {
    if (powerInPercent != currentPower) {
      out.printf(Locale.US, "S%d\n", (int) (255d * powerInPercent / 100));
      currentPower = powerInPercent;
    }
  }

  private void move(PrintStream out, int x, int y, double resolution) {
    out.printf(Locale.US, "G0 X%f Y%f\n", Util.px2mm(isFlipXaxis() ? Util.mm2px(bedWidth, resolution) - x : x, resolution), Util.px2mm(y, resolution));
  }

  private void line(PrintStream out, int x, int y, double resolution) {
    out.printf(Locale.US, "G1 X%f Y%f\n", Util.px2mm(isFlipXaxis() ? Util.mm2px(bedWidth, resolution) - x : x, resolution), Util.px2mm(y, resolution));
  }

  private byte[] generatePseudoRaster3dGCode(Raster3dPart rp, double resolution) throws UnsupportedEncodingException {
    ByteArrayOutputStream result = new ByteArrayOutputStream();
    PrintStream out = new PrintStream(result, true, "US-ASCII");
    boolean dirRight = true;
    Point rasterStart = rp.getRasterStart();
    PowerSpeedFocusProperty prop = (PowerSpeedFocusProperty) rp.getLaserProperty();
    setSpeed(out, prop.getSpeed());
    for (int line = 0; line < rp.getRasterHeight(); line++) {
      Point lineStart = rasterStart.clone();
      lineStart.y += line;
      List<Byte> bytes = rp.getRasterLine(line);
      //remove heading zeroes
      while (bytes.size() > 0 && bytes.get(0) == 0) {
        bytes.remove(0);
        lineStart.x += 1;
      }
      //remove trailing zeroes
      while (bytes.size() > 0 && bytes.get(bytes.size() - 1) == 0) {
        bytes.remove(bytes.size() - 1);
      }
      if (bytes.size() > 0) {
        if (dirRight) {
          //move to the first nonempyt point of the line
          move(out, lineStart.x, lineStart.y, resolution);
          byte old = bytes.get(0);
          for (int pix = 0; pix < bytes.size(); pix++) {
            if (bytes.get(pix) != old) {
              if (old == 0) {
                move(out, lineStart.x + pix, lineStart.y, resolution);
              } else {
                setPower(out, prop.getPower() * (0xFF & old) / 255);
                line(out, lineStart.x + pix - 1, lineStart.y, resolution);
                move(out, lineStart.x + pix, lineStart.y, resolution);
              }
              old = bytes.get(pix);
            }
          }
          //last point is also not "white"
          setPower(out, prop.getPower() * (0xFF & bytes.get(bytes.size() - 1)) / 255);
          line(out, lineStart.x + bytes.size() - 1, lineStart.y, resolution);
        } else {
          //move to the last nonempty point of the line
          move(out, lineStart.x + bytes.size() - 1, lineStart.y, resolution);
          byte old = bytes.get(bytes.size() - 1);
          for (int pix = bytes.size() - 1; pix >= 0; pix--) {
            if (bytes.get(pix) != old || pix == 0) {
              if (old == 0) {
                move(out, lineStart.x + pix, lineStart.y, resolution);
              } else {
                setPower(out, prop.getPower() * (0xFF & old) / 255);
                line(out, lineStart.x + pix + 1, lineStart.y, resolution);
                move(out, lineStart.x + pix, lineStart.y, resolution);
              }
              old = bytes.get(pix);
            }
          }
          //last point is also not "white"
          setPower(out, prop.getPower() * (0xFF & bytes.get(0)) / 255);
          line(out, lineStart.x, lineStart.y, resolution);
        }
      }
      dirRight = !dirRight;
    }
    return result.toByteArray();
  }

  private byte[] generatePseudoRasterGCode(RasterPart rp, double resolution) throws UnsupportedEncodingException {
    ByteArrayOutputStream result = new ByteArrayOutputStream();
    PrintStream out = new PrintStream(result, true, "US-ASCII");
    boolean dirRight = true;
    Point rasterStart = rp.getRasterStart();
    PowerSpeedFocusProperty prop = (PowerSpeedFocusProperty) rp.getLaserProperty();
    setSpeed(out, prop.getSpeed());
    setPower(out, prop.getPower());
    for (int line = 0; line < rp.getRasterHeight(); line++) {
      Point lineStart = rasterStart.clone();
      lineStart.y += line;
      List<Byte> bytes = new LinkedList<Byte>();
      boolean lookForStart = true;
      for (int x = 0; x < rp.getRasterWidth(); x++) {
        if (lookForStart) {
          if (rp.isBlack(x, line)) {
            lookForStart = false;
            bytes.add((byte) 255);
          } else {
            lineStart.x += 1;
          }
        } else {
          bytes.add(rp.isBlack(x, line) ? (byte) 255 : (byte) 0);
        }
      }
      //remove trailing zeroes
      while (bytes.size() > 0 && bytes.get(bytes.size() - 1) == 0) {
        bytes.remove(bytes.size() - 1);
      }
      if (bytes.size() > 0) {
        if (dirRight) {
          //add some space to the left
          move(out, Math.max(0, (int) (lineStart.x - Util.mm2px(this.addSpacePerRasterLine, resolution))), lineStart.y, resolution);
          //move to the first nonempyt point of the line
          move(out, lineStart.x, lineStart.y, resolution);
          byte old = bytes.get(0);
          for (int pix = 0; pix < bytes.size(); pix++) {
            if (bytes.get(pix) != old) {
              if (old == 0) {
                move(out, lineStart.x + pix, lineStart.y, resolution);
              } else {
                setPower(out, prop.getPower() * (0xFF & old) / 255);
                line(out, lineStart.x + pix - 1, lineStart.y, resolution);
                move(out, lineStart.x + pix, lineStart.y, resolution);
              }
              old = bytes.get(pix);
            }
          }
          //last point is also not "white"
          setPower(out, prop.getPower() * (0xFF & bytes.get(bytes.size() - 1)) / 255);
          line(out, lineStart.x + bytes.size() - 1, lineStart.y, resolution);
          //add some space to the right
          move(out, Math.min((int) Util.mm2px(bedWidth, resolution), (int) (lineStart.x + bytes.size() - 1 + Util.mm2px(this.addSpacePerRasterLine, resolution))), lineStart.y, resolution);
        } else {
          //add some space to the right
          move(out, Math.min((int) Util.mm2px(bedWidth, resolution), (int) (lineStart.x + bytes.size() - 1 + Util.mm2px(this.addSpacePerRasterLine, resolution))), lineStart.y, resolution);
          //move to the last nonempty point of the line
          move(out, lineStart.x + bytes.size() - 1, lineStart.y, resolution);
          byte old = bytes.get(bytes.size() - 1);
          for (int pix = bytes.size() - 1; pix >= 0; pix--) {
            if (bytes.get(pix) != old || pix == 0) {
              if (old == 0) {
                move(out, lineStart.x + pix, lineStart.y, resolution);
              } else {
                setPower(out, prop.getPower() * (0xFF & old) / 255);
                line(out, lineStart.x + pix + 1, lineStart.y, resolution);
                move(out, lineStart.x + pix, lineStart.y, resolution);
              }
              old = bytes.get(pix);
            }
          }
          //last point is also not "white"
          setPower(out, prop.getPower() * (0xFF & bytes.get(0)) / 255);
          line(out, lineStart.x, lineStart.y, resolution);
          //add some space to the left
          move(out, Math.max(0, (int) (lineStart.x - Util.mm2px(this.addSpacePerRasterLine, resolution))), lineStart.y, resolution);
        }
      }
      dirRight = !dirRight;
    }
    return result.toByteArray();
  }

  @Override
  public void sendJob(LaserJob job, ProgressListener pl, List<String> warnings) throws IllegalJobException, Exception {
    pl.progressChanged(this, 0);
    this.currentPower = -1;
    this.currentSpeed = -1;
    BufferedOutputStream out;
    BufferedInputStream in;
    PrintStream outStrings;

    pl.taskChanged(this, "checking job");
    checkJob(job);
    job.applyStartPoint();
    pl.taskChanged(this, "connecting");
    CommPortIdentifier cpi = null;
    //since the CommPortIdentifier.getPortIdentifier(String name) method
    //is not working as expected, we have to manually find our port.
    Enumeration en = CommPortIdentifier.getPortIdentifiers();
    while (en.hasMoreElements())
    {
      Object o = en.nextElement();
      if (o instanceof CommPortIdentifier && ((CommPortIdentifier) o).getName().equals(this.getComPort()))
      {
        cpi = (CommPortIdentifier) o;
        break;
      }
    }
    if (cpi == null)
    {
      pl.taskChanged(this, "COM-Port not found");
      throw new Exception("Error: No such COM-Port '"+this.getComPort()+"'");
    }
    CommPort tmp = cpi.open("VisiCut", 10000);
    if (tmp == null)
    {
      pl.taskChanged(this, "Couldn't open COM-Port");
      throw new Exception("Error: Could not Open COM-Port '"+this.getComPort()+"'");
    }
    if (!(tmp instanceof SerialPort))
    {
      pl.taskChanged(this, "Not a serial port");
      throw new Exception("Port '"+this.getComPort()+"' is not a serial port.");
    }
    SerialPort port = (SerialPort) tmp;
    port.setFlowControlMode(SerialPort.FLOWCONTROL_RTSCTS_OUT | SerialPort.FLOWCONTROL_RTSCTS_OUT);
    port.setSerialPortParams(this.comBaud, SerialPort.DATABITS_8, SerialPort.STOPBITS_1, SerialPort.PARITY_NONE);
    out = new BufferedOutputStream(port.getOutputStream(), UART_BYTE_CHUNK_SIZE);
    in = new BufferedInputStream(port.getInputStream());

    // add a PrintStream class wrapper around BufferedOutputStream to allow easy string printing.
    outStrings = new PrintStream(out);

    pl.taskChanged(this, "sending");
    // convert the ';' to newline and append a newline
    outStrings.print(this.jobPreGCode.replaceAll(";", "\n") + "\n");
    pl.progressChanged(this, 20);
    int i = 0;
    int max = job.getParts().size();
    for (JobPart p : job.getParts())
    {
      byte[] gcode_stream = null;
      int offset = 0;
      int stride = UART_BYTE_CHUNK_SIZE / 2;

      if (p instanceof Raster3dPart)
      {
        gcode_stream = this.generatePseudoRaster3dGCode((Raster3dPart) p, p.getDPI());
      }
      else if (p instanceof RasterPart)
      {
        gcode_stream = this.generatePseudoRasterGCode((RasterPart) p, p.getDPI());
      }
      else if (p instanceof VectorPart)
      {
        gcode_stream = this.generateVectorGCode((VectorPart) p, p.getDPI());
      }

      if (gcode_stream == null)
      {
        throw new Exception("Unknown job type!");
      }

      //send the gcode stream out the serial port in a controlled way
      while(gcode_stream.length > (offset + 1)) {
        int loopct;

        //FIXME: this should be a timer
        for (loopct = 0; loopct < MAX_CTS_LOOPS; loopct++) {
          if (port.isCTS())
            break;
        }

        if (loopct == MAX_CTS_LOOPS) {
            outStrings.close();
            out.close();
            port.close();
            pl.taskChanged(this, "CTS timeout");
            throw new Exception("Error: COM port ClearToSend never signaled!");
        }

        if ((gcode_stream.length - (offset + 1)) < stride) {
          stride = gcode_stream.length - (offset + 1);
        }

        out.write(gcode_stream, offset, stride);
        offset += stride;

        //FIXME: should check port input for error ; for now keep it empty
        if (in.available() > 0)
            in.read();
       }
       // Progress reflects subjobs
       i++;
       pl.progressChanged(this, 20 + (int) (i*(double) 60/max));
    }
    pl.taskChanged(this, "finishing");
    // convert the ';' to newline and insert + append a newline
    outStrings.print("\n" + this.jobPostGCode.replaceAll(";", "\n") + "\n");
    outStrings.close();
    out.close();
    port.close();
    pl.taskChanged(this, "sent.");
    pl.progressChanged(this, 100);
  }
  private List<Double> resolutions;

  @Override
  public List<Double> getResolutions() {
    if (resolutions == null) {
      resolutions = Arrays.asList(new Double[]{
                500d
              });
    }
    return resolutions;
  }
  protected double bedWidth = 250;

  /**
   * Get the value of bedWidth
   *
   * @return the value of bedWidth
   */
  @Override
  public double getBedWidth() {
    return bedWidth;
  }

  /**
   * Set the value of bedWidth
   *
   * @param bedWidth new value of bedWidth
   */
  public void setBedWidth(double bedWidth) {
    this.bedWidth = bedWidth;
  }
  protected double bedHeight = 280;

  /**
   * Get the value of bedHeight
   *
   * @return the value of bedHeight
   */
  @Override
  public double getBedHeight() {
    return bedHeight;
  }

  /**
   * Set the value of bedHeight
   *
   * @param bedHeight new value of bedHeight
   */
  public void setBedHeight(double bedHeight) {
    this.bedHeight = bedHeight;
  }
  private static String[] settingAttributes = new String[]{
    SETTING_BEDWIDTH,
    SETTING_BEDHEIGHT,
    SETTING_FLIPX,
    SETTING_COMPORT,
    SETTING_COMBAUD,
    SETTING_LASER_RATE,
    SETTING_SEEK_RATE,
    SETTING_RASTER_WHITESPACE,
    SETTING_JOB_PRE_GCODE,
    SETTING_JOB_POST_GCODE
  };

  @Override
  public String[] getPropertyKeys() {
    return settingAttributes;
  }

  @Override
  public Object getProperty(String attribute) {
    if (SETTING_RASTER_WHITESPACE.equals(attribute)) {
      return this.getAddSpacePerRasterLine();
    } else if (SETTING_COMPORT.equals(attribute)) {
      return this.getComPort();
    } else if (SETTING_COMBAUD.equals(attribute)) {
      return this.getComBaud();
    } else if (SETTING_FLIPX.equals(attribute)) {
      return this.isFlipXaxis();
    } else if (SETTING_LASER_RATE.equals(attribute)) {
      return this.getLaserRate();
    } else if (SETTING_SEEK_RATE.equals(attribute)) {
      return this.getSeekRate();
    } else if (SETTING_BEDWIDTH.equals(attribute)) {
      return this.getBedWidth();
    } else if (SETTING_BEDHEIGHT.equals(attribute)) {
      return this.getBedHeight();
    } else if (SETTING_JOB_PRE_GCODE.equals(attribute)) {
      return this.getJobPreGCode();
    } else if (SETTING_JOB_POST_GCODE.equals(attribute)) {
      return this.getJobPostGCode();
    }
    return null;
  }

  @Override
  public void setProperty(String attribute, Object value) {
    if (SETTING_RASTER_WHITESPACE.equals(attribute)) {
      this.setAddSpacePerRasterLine((Double) value);
    } else if (SETTING_COMPORT.equals(attribute)) {
      this.setComPort((String) value);
    } else if (SETTING_COMBAUD.equals(attribute)) {
      this.setComBaud((Integer) value);
    } else if (SETTING_LASER_RATE.equals(attribute)) {
      this.setLaserRate((Double) value);
    } else if (SETTING_SEEK_RATE.equals(attribute)) {
      this.setSeekRate((Double) value);
    } else if (SETTING_FLIPX.equals(attribute)) {
      this.setFlipXaxis((Boolean) value);
    } else if (SETTING_BEDWIDTH.equals(attribute)) {
      this.setBedWidth((Double) value);
    } else if (SETTING_BEDHEIGHT.equals(attribute)) {
      this.setBedHeight((Double) value);
    }
  }

  @Override
  public LaserCutter clone() {
    Grbl clone = new Grbl();
    clone.comPort = comPort;
    clone.comBaud = comBaud;
    clone.laserRate = laserRate;
    clone.seekRate = seekRate;
    clone.bedHeight = bedHeight;
    clone.bedWidth = bedWidth;
    clone.flipXaxis = flipXaxis;
    clone.addSpacePerRasterLine = addSpacePerRasterLine;
    clone.jobPreGCode = jobPreGCode;
    clone.jobPostGCode = jobPostGCode;
    return clone;
  }
}
