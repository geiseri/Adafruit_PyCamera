#include "Adafruit_PyCamera.h"
#include "esp_camera.h"
#include "sensor.h"
#include <array>
#include <algorithm>

namespace detail {
  int JPEGDraw(JPEGDRAW *pDraw)
  {
    auto self = reinterpret_cast<PyCameraFB*>(pDraw->pUser);
    if (self->width() < pDraw->iWidth || self->height() < pDraw->iHeight) {
      ESP_LOGE("PYCAM", "JPEGDraw: Out of bounds: x=%d, y=%d, width=%d, height=%d", pDraw->x, pDraw->y, pDraw->iWidth, pDraw->iHeight);
      return 0;
    }
    self->drawRGBBitmap((int16_t)pDraw->x, (int16_t)pDraw->y, (uint16_t *)pDraw->pPixels, (int16_t)pDraw->iWidth, (int16_t)pDraw->iHeight);
    return 1;
  } /* JPEGDraw() */
  
  class scoped_cleanup {
    public:
    scoped_cleanup(std::function<void()> func) : func_(func) {}
    ~scoped_cleanup() { 
      func_(); 
    }
    private:
    std::function<void()> func_;
  };
}


/**************************************************************************/
/**
 * @brief Construct a new Adafruit_PyCamera object.
 *
 * @details Initializes the display with specified TFT parameters.
 *
 */
/**************************************************************************/
Adafruit_PyCamera::Adafruit_PyCamera(void)
    : Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RESET) {
    fb = new PyCameraFB(240, 240);
    jpeg_ = new JPEGDEC();
    }

/**************************************************************************/
/**
 * @brief Initializes the PyCamera.
 *
 * @details Sets up the speaker, Neopixel, Neopixel Ring, shutter button, and
 * performs I2C scan. Initializes the display, expander, camera, frame size, SD
 * card (if detected), and accelerometer. Creates a new framebuffer for the
 * camera.
 *
 * @return true if initialization is successful, false otherwise.
 */
/**************************************************************************/
bool Adafruit_PyCamera::setup(uint32_t freq) {
  ESP_LOGI("PYCAM", "Init PyCamera object");
  //begin(freq);
  // Setup and turn off speaker
  pinMode(SPEAKER, OUTPUT);
  digitalWrite(SPEAKER, LOW);

  // Setup and turn off Neopixel
  pixel.setPin(PIN_NEOPIXEL);
  pixel.updateLength(1);
  pixel.begin();
  pixel.setBrightness(50);
  setNeopixel(0x0);

  // Setup and turn off Neopixel Ring
  ring.setPin(A1);
  ring.updateType(NEO_GRBW + NEO_KHZ800);
  ring.updateLength(8);
  ring.begin();
  ring.setBrightness(255);
  setRing(0x0);

  // boot button is also shutter
  pinMode(SHUTTER_BUTTON, INPUT_PULLUP);

  I2Cscan();

  if (!initDisplay())
    return false;
  if (!initExpander())
    return false;
  if (!initCamera(true))
    return false;
  if (!setFramesize(FRAMESIZE_240X240))
    return false;
  if (SDdetected())
    initSD();
  if (!initAccel())
    return false;

  _timestamp = millis();
  return true;
}

/**************************************************************************/
/**
 * @brief Initializes the SD card.
 *
 * @details Checks for SD card presence and attempts initialization. Performs a
 * power reset, reinitializes SPI for SD card communication, and checks for
 * errors during SD card initialization. Also lists files on the SD card if
 * initialization is successful.
 *
 * @return true if SD card is successfully initialized, false otherwise.
 */
/**************************************************************************/
bool Adafruit_PyCamera::initSD(void) {

  if (!SDdetected()) {
    ESP_LOGI("PYCAM", "No SD card inserted");
    return false;
  }

  ESP_LOGI("PYCAM", "SD card inserted, trying to init");

  // power reset
  aw.pinMode(AWEXP_SD_PWR, OUTPUT);
  aw.digitalWrite(AWEXP_SD_PWR, HIGH); // turn off

  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, LOW);

  ESP_LOGI("PYCAM", "De-initing SPI");
  SPI.end();
  // Configure SPI pins as outputs and set them LOW before reinitializing SPI
  pinMode(SCK, OUTPUT);
  digitalWrite(SCK, LOW);
  pinMode(MOSI, OUTPUT);
  digitalWrite(MOSI, LOW);  // Fixed: was writing MISO before pinMode
  pinMode(MISO, OUTPUT);
  digitalWrite(MISO, LOW);

  delay(50);

  ESP_LOGI("PYCAM", "Re-init SPI");
  digitalWrite(SD_CS, HIGH);
  SPI.begin();
  aw.digitalWrite(AWEXP_SD_PWR, LOW); // turn on
  delay(100);

  if (!sd.begin(SD_CS, SD_SCK_MHZ(4))) {
    if (sd.card()->errorCode()) {
      ESP_LOGE("PYCAM", "SD card init failure with code 0x%x data %d",
                    sd.card()->errorCode(), (int)sd.card()->errorData());
    } else if (sd.vol()->fatType() == 0) {
      ESP_LOGE("PYCAM", "Can't find a valid FAT16/FAT32 partition.");
    } else {
      ESP_LOGE("PYCAM", "SD begin failed, can't determine error type");
    }
    aw.digitalWrite(AWEXP_SD_PWR, HIGH); // turn off power
    return false;
  }

  ESP_LOGI("PYCAM", "Card successfully initialized");
  uint32_t size = sd.card()->sectorCount();
  if (size == 0) {
    ESP_LOGE("PYCAM", "Can't determine the card size");
  } else {
    uint32_t sizeMB = 0.000512 * size + 0.5;
    ESP_LOGI("PYCAM", "Card size: %d MB FAT%d", (int)sizeMB, sd.vol()->fatType());
  }
  ESP_LOGI("PYCAM", "Files found (date time size name):");
  sd.ls(LS_R | LS_DATE | LS_SIZE);
  return true;
}

/**************************************************************************/
/**
 * @brief Ends the SD card session.
 *
 * @details Powers off the SD card to end the session.
 */
/**************************************************************************/
void Adafruit_PyCamera::endSD() {
  // aw.pinMode(AWEXP_SD_PWR, OUTPUT);
  // aw.digitalWrite(AWEXP_SD_PWR, HIGH); // start off
}

/**************************************************************************/
/**
 * @brief Initializes the AW9523 I/O expander.
 *
 * @details Sets up the AW9523 expander, configuring speaker, SD power, and SD
 * detection pins.
 *
 * @return true if the expander is successfully initialized, false otherwise.
 */
/**************************************************************************/
bool Adafruit_PyCamera::initExpander(void) {
  ESP_LOGI("PYCAM", "Init AW9523...");
  if (!aw.begin(0x58)) {
    ESP_LOGE("PYCAM", "AW9523 not found!");
    return false;
  }
  ESP_LOGI("PYCAM", "OK!");
  aw.pinMode(AWEXP_SPKR_SD, OUTPUT);
  aw.digitalWrite(AWEXP_SPKR_SD, LOW); // start muted
  aw.pinMode(AWEXP_SD_PWR, OUTPUT);
  aw.digitalWrite(AWEXP_SD_PWR, LOW); // start SD powered
  aw.pinMode(AWEXP_SD_DET, INPUT);
  return true;
}

/**************************************************************************/
/**
 * @brief Initializes the display.
 *
 * @details This method sets up the display for the PyCamera. It starts by
 * initializing the backlight control, then initializes the ST7789 screen with
 * the specified dimensions and rotation. Finally, it fills the screen with a
 * green color and turns on the backlight.
 *
 * @return true if the display is successfully initialized, false otherwise.
 */
/**************************************************************************/
bool Adafruit_PyCamera::initDisplay(void) {
  ESP_LOGI("PYCAM", "Init display....");

  pinMode(TFT_BACKLIGHT, OUTPUT);
  digitalWrite(TFT_BACKLIGHT, LOW);
  init(240, 240); // Initialize ST7789 screen
  setRotation(1);
  fillScreen(ST77XX_GREEN);

  digitalWrite(TFT_BACKLIGHT, HIGH);
  ESP_LOGI("PYCAM", "done!");
  return true;
}

/**************************************************************************/
/**
 * @brief Sets the frame size for the camera.
 *
 * @details Configures the camera to use the specified frame size. If the frame
 * size cannot be set, it outputs an error message with the error code.
 *
 * @param framesize The desired frame size to set for the camera.
 * @return true if the frame size is successfully set, false if there is an
 * error.
 */
/**************************************************************************/
bool Adafruit_PyCamera::setFramesize(framesize_t framesize) {
  // Validate that the framesize is in the Framesize array
  auto it = std::find_if(Framesize.begin(), Framesize.end(),
                         [framesize](const FramesizeInfo& info) {
                           return info.framesize == framesize;
                         });
  if (it == Framesize.end()) {
    ESP_LOGE("PYCAM", "Invalid framesize: %d (not in valid framesizes list)", framesize);
    return false;
  }
  
  uint8_t ret = camera->set_framesize(camera, framesize);
  if (ret != 0) {
    ESP_LOGE("PYCAM", "Could not set resolution: error 0x%x", ret);
    return false;
  }
  framesize_ = framesize;
  ESP_LOGI("PYCAM", "Set framesize: %d", framesize);
  ESP_LOGI("PYCAM", "Camera framesize: %d", camera->status.framesize);
  return true;
}

framesize_t Adafruit_PyCamera::getFramesize() {
  return framesize_;
}

/**************************************************************************/
/**
 * @brief Cycles to the next framesize in the valid framesizes list.
 *
 * @details Gets the current framesize, finds it in the valid framesizes list,
 * and moves to the next one. If already at the maximum, it clamps and stays
 * at the maximum. If the current framesize is not found in the valid list,
 * it wraps to the first valid framesize.
 *
 * @return true if the framesize was successfully changed, false otherwise.
 */
/**************************************************************************/
bool Adafruit_PyCamera::cycleFramesizeForward() {
  framesize_t current = getFramesize();
  
  auto it = std::find_if(Framesize.begin(), Framesize.end(),
                         [current](const FramesizeInfo& info) {
                           return info.framesize == current;
                         });
  
  framesize_t next_framesize;
  if (it == Framesize.end()) {
    next_framesize = Framesize.front().framesize;
  } else {
    ++it;
    next_framesize = (it == Framesize.end()) ? Framesize.back().framesize : it->framesize;
  }
  
  return setFramesize(next_framesize);
}

/**************************************************************************/
/**
 * @brief Cycles to the previous framesize in the valid framesizes list.
 *
 * @details Gets the current framesize, finds it in the valid framesizes list,
 * and moves to the previous one. If already at the minimum, it clamps and stays
 * at the minimum. If the current framesize is not found in the valid list,
 * it wraps to the last valid framesize.
 *
 * @return true if the framesize was successfully changed, false otherwise.
 */
/**************************************************************************/
bool Adafruit_PyCamera::cycleFramesizeBackward() {
  framesize_t current = getFramesize();
  
  auto it = std::find_if(Framesize.begin(), Framesize.end(),
                         [current](const FramesizeInfo& info) {
                           return info.framesize == current;
                         });
  
  framesize_t prev_framesize;
  if (it == Framesize.end()) {
    prev_framesize = Framesize.back().framesize;
  } else if (it == Framesize.begin()) {
    prev_framesize = Framesize.front().framesize;
  } else {
    --it;
    prev_framesize = it->framesize;
  }
  
  return setFramesize(prev_framesize);
}

uint8_t Adafruit_PyCamera::getSpecialEffect() {
  return camera->status.special_effect;
}

/**************************************************************************/
/**
 * @brief Sets a special effect on the camera.
 *
 * @details Applies a specified special effect to the camera's output. If the
 * effect cannot be set, it outputs an error message with the error code.
 *
 * @param effect The special effect identifier to apply.
 * @return true if the special effect is successfully set, false if there is an
 * error.
 */
/**************************************************************************/
bool Adafruit_PyCamera::setSpecialEffect(uint8_t effect) {
  // Validate that the effect is in the SpecialEffect array
  auto it = std::find_if(SpecialEffect.begin(), SpecialEffect.end(),
                         [effect](const SpecialEffectInfo& info) {
                           return info.effect == effect;
                         });
  if (it == SpecialEffect.end()) {
    ESP_LOGE("PYCAM", "Invalid special effect: %d (must be 0-6)", effect);
    return false;
  }
  
  uint8_t ret = camera->set_special_effect(camera, effect);
  if (ret != 0) {
    ESP_LOGE("PYCAM", "Could not set effect: error 0x%x", ret);
    return false;
  }
  return true;
}

/**************************************************************************/
/**
 * @brief Cycles to the next special effect in the valid effects list.
 *
 * @details Gets the current special effect, finds it in the valid effects list,
 * and moves to the next one. If already at the maximum, it clamps and stays
 * at the maximum. If the current effect is not found in the valid list,
 * it wraps to the first valid effect.
 *
 * @return true if the effect was successfully changed, false otherwise.
 */
/**************************************************************************/
bool Adafruit_PyCamera::cycleSpecialEffectForward() {
  uint8_t current = getSpecialEffect();
  
  auto it = std::find_if(SpecialEffect.begin(), SpecialEffect.end(),
                         [current](const SpecialEffectInfo& info) {
                           return info.effect == current;
                         });
  
  uint8_t next_effect;
  if (it == SpecialEffect.end()) {
    next_effect = SpecialEffect.front().effect;
  } else {
    ++it;
    next_effect = (it == SpecialEffect.end()) ? SpecialEffect.back().effect : it->effect;
  }
  
  return setSpecialEffect(next_effect);
}

/**************************************************************************/
/**
 * @brief Cycles to the previous special effect in the valid effects list.
 *
 * @details Gets the current special effect, finds it in the valid effects list,
 * and moves to the previous one. If already at the minimum, it clamps and stays
 * at the minimum. If the current effect is not found in the valid list,
 * it wraps to the last valid effect.
 *
 * @return true if the effect was successfully changed, false otherwise.
 */
/**************************************************************************/
bool Adafruit_PyCamera::cycleSpecialEffectBackward() {
  uint8_t current = getSpecialEffect();
  
  auto it = std::find_if(SpecialEffect.begin(), SpecialEffect.end(),
                         [current](const SpecialEffectInfo& info) {
                           return info.effect == current;
                         });
  
  uint8_t prev_effect;
  if (it == SpecialEffect.end()) {
    prev_effect = SpecialEffect.back().effect;
  } else if (it == SpecialEffect.begin()) {
    prev_effect = SpecialEffect.front().effect;
  } else {
    --it;
    prev_effect = it->effect;
  }
  
  return setSpecialEffect(prev_effect);
}

/**************************************************************************/
/**
 * @brief Initializes the camera module.
 *
 * @details Configures and initializes the camera with specified settings.
 * It sets up various camera parameters like LEDC channel and timer, pin
 * configuration, XCLK frequency, frame buffer location, pixel format, frame
 * size, and JPEG quality. It also handles the hardware reset if specified.
 * After configuration, it initializes the camera and checks for errors. If
 * successful, it retrieves the camera sensor information and sets horizontal
 * mirror and vertical flip settings.
 *
 * @param hwreset Flag to determine if a hardware reset is needed.
 * @return true if the camera is successfully initialized, false if there is an
 * error.
 */
/**************************************************************************/
bool Adafruit_PyCamera::initCamera(bool hwreset) {
  ESP_LOGI("PYCAM", "Config camera...");
  Wire.begin();

  if (hwreset) {
  }
  camera_config.ledc_channel = LEDC_CHANNEL_0;
  camera_config.ledc_timer = LEDC_TIMER_0;
  camera_config.pin_d0 = Y2_GPIO_NUM;
  camera_config.pin_d1 = Y3_GPIO_NUM;
  camera_config.pin_d2 = Y4_GPIO_NUM;
  camera_config.pin_d3 = Y5_GPIO_NUM;
  camera_config.pin_d4 = Y6_GPIO_NUM;
  camera_config.pin_d5 = Y7_GPIO_NUM;
  camera_config.pin_d6 = Y8_GPIO_NUM;
  camera_config.pin_d7 = Y9_GPIO_NUM;
  camera_config.pin_xclk = XCLK_GPIO_NUM;
  camera_config.pin_pclk = PCLK_GPIO_NUM;
  camera_config.pin_vsync = VSYNC_GPIO_NUM;
  camera_config.pin_href = HREF_GPIO_NUM;
  camera_config.pin_sccb_sda = -1;
  camera_config.pin_sccb_scl = -1;
  // use the built in I2C port
  camera_config.sccb_i2c_port = 0; // use the 'first' i2c port
  camera_config.pin_pwdn = PWDN_GPIO_NUM;
  camera_config.pin_reset = RESET_GPIO_NUM;
  camera_config.xclk_freq_hz = 20000000;
  camera_config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  camera_config.fb_location = CAMERA_FB_IN_PSRAM;

  ESP_LOGI("PYCAM", "Config format...");
  /*
     // using RGB565 for immediate blitting
     camera_config.pixel_format = PIXFORMAT_RGB565;
     camera_config.frame_size = FRAMESIZE_240X240;
     camera_config.fb_count = 1;
   */
  camera_config.pixel_format = PIXFORMAT_JPEG;
  camera_config.frame_size =
      Adafruit_PyCamera::Framesize.back().framesize; // start with biggest possible image supported!!! do not
                      // change this
  camera_config.jpeg_quality = 4;
  camera_config.fb_count = 2;

  ESP_LOGI("PYCAM", "Initializing...");
  // camera init
  esp_err_t err = esp_camera_init(&camera_config);

  if (err != ESP_OK) {
    ESP_LOGE("PYCAM", "Camera init failed with error 0x%x", err);
    return false;
  }

  ESP_LOGI("PYCAM", "OK");

  camera = esp_camera_sensor_get();
  ESP_LOGI("PYCAM", "Found camera PID %04X", camera->id.PID);
  camera->set_hmirror(camera, 0);
  camera->set_vflip(camera, 1);

  return true;
}

/**************************************************************************/
/**
 * @brief Reads the battery voltage.
 *
 * @details Measures the battery voltage through the BATT_MONITOR pin.
 * The reading is scaled to account for the voltage divider and ADC resolution.
 *
 * @return The battery voltage in volts.
 */
/**************************************************************************/
float Adafruit_PyCamera::readBatteryVoltage(void) {
  return analogRead(BATT_MONITOR) * 2.0 * 3.3 / 4096;
}

/**************************************************************************/
/**
 * @brief Checks if an SD card is detected.
 *
 * @details Reads the state of the SD card detection pin using the AW9523
 * expander. A high state indicates that an SD card is present.
 *
 * @return true if an SD card is detected, false otherwise.
 */
/**************************************************************************/
bool Adafruit_PyCamera::SDdetected(void) {
  return aw.digitalRead(AWEXP_SD_DET);
}

/**************************************************************************/
/**
 * @brief Reads the current state of the buttons.
 *
 * @details Retrieves the state of all buttons connected to the AW9523 expander
 * and the shutter button. The state is updated and stored in the button_state
 * variable. The previous state is stored in last_button_state.
 *
 * @return The current state of the buttons as a 32-bit unsigned integer.
 */
/**************************************************************************/
uint32_t Adafruit_PyCamera::readButtons(void) {
  last_button_state = button_state;
  button_state = aw.inputGPIO() & static_cast<uint32_t>(ButtonMask::INPUTS);
  button_state |= (bool)digitalRead(SHUTTER_BUTTON);
  return button_state;
}

/**************************************************************************/
/**
 * @brief Checks if a button was just pressed.
 *
 * @details Determines if the specified button has transitioned from a
 * non-pressed to a pressed state since the last read. This function is useful
 * for detecting button press events.
 *
 * @param button_pin The pin number of the button to check.
 * @return true if the button was just pressed, false otherwise.
 */
/**************************************************************************/
bool Adafruit_PyCamera::justPressed(uint8_t button_pin) {
  return ((last_button_state & (1UL << button_pin)) && // was not pressed before
          !(button_state & (1UL << button_pin)));      // and is pressed now
}

/**************************************************************************/
/**
 * @brief Checks if a button was just released.
 *
 * @details Determines if the specified button has transitioned from a pressed
 * to a non-pressed state since the last read. This function is useful for
 * detecting button release events.
 *
 * @param button_pin The pin number of the button to check.
 * @return true if the button was just released, false otherwise.
 */
/**************************************************************************/
bool Adafruit_PyCamera::justReleased(uint8_t button_pin) {
  return (!(last_button_state & (1UL << button_pin)) && // was pressed before
          (button_state & (1UL << button_pin)));        // and isnt pressed now
}

/**************************************************************************/
/**
 * @brief Plays a tone through the speaker.
 *
 * @details Generates a tone of a specified frequency and duration through the
 * speaker. It unmutes the speaker before playing the tone and mutes it again
 * after the tone is played. The function uses a blocking delay for the duration
 * of the tone.
 *
 * @param tonefreq The frequency of the tone in Hertz.
 * @param tonetime The duration of the tone in milliseconds.
 */
/**************************************************************************/
void Adafruit_PyCamera::speaker_tone(uint32_t tonefreq, uint32_t tonetime) {
  aw.digitalWrite(AWEXP_SPKR_SD, HIGH); // un-mute
  tone(SPEAKER, tonefreq, tonetime);    // tone1 - B5
  delay(tonetime);
  aw.digitalWrite(AWEXP_SPKR_SD, LOW); // mute
}

/**************************************************************************/
/**
 * @brief Captures a photo and saves it to an SD card.
 *
 * @details This function captures a photo with the camera at the specified
 * resolution, and saves it to the SD card with a filename based on the provided
 * base name. It handles SD card detection, initialization, and file creation.
 * The function also manages camera frame buffer acquisition and release, and
 * sets the camera resolution.
 *
 * @param filename_base Base name for the file to be saved. The function appends
 * a numerical suffix to create a unique filename.
 * @param framesize The resolution at which the photo should be captured.
 * @return true if the photo is successfully captured and saved, false
 * otherwise.
 */
/**************************************************************************/
bool Adafruit_PyCamera::takePhoto(const char *filename_base,
                                  framesize_t framesize) {
  auto original_framesize = getFramesize();
  auto cleanup = detail::scoped_cleanup([this, original_framesize]() {
    setFramesize(original_framesize);
  });
  File file;

  if (!SDdetected()) {
    ESP_LOGE("PYCAM", "No SD card inserted");
    return false;
  }

  if (!sd.card() || (sd.card()->sectorCount() == 0)) {
    ESP_LOGE("PYCAM", "No SD card found");
    // try to initialize?
    if (!initSD())
    {
      setFramesize(original_framesize);
      ESP_LOGE("PYCAM", "SD card init failed");
      return false;
    }
  }

  // we're probably going to succeed in saving the file so we should
  // change rez now since we need to grab two frames worth to clear out a cache
  ESP_LOGI("PYCAM", "Reconfiguring resolution");
  if (!setFramesize(framesize))
    return false;

  // capture and toss first internal buffer
  camera_fb_t *frame = esp_camera_fb_get();
  auto  frame_cleanup = detail::scoped_cleanup([frame]() {
      esp_camera_fb_return(frame);
  });
  if (!frame) {
    ESP_LOGE("PYCAM", "Couldnt capture first frame");
    return false;
  }

  ESP_LOGI("PYCAM", "\t\t\tSnapped 1st %d bytes (%d x %d) in %d ms", frame->len,
                frame->width, frame->height, (int)timestamp());
  esp_camera_fb_return(frame);

  // capture and toss second internal buffer
  frame = esp_camera_fb_get();
  if (!frame) {
    ESP_LOGE("PYCAM", "Couldnt capture second frame");
    return false;
  }
  ESP_LOGI("PYCAM", "\t\t\tSnapped 2nd %d bytes (%d x %d) in %d ms", frame->len,
                frame->width, frame->height, (int)timestamp());
  char fullfilename[64];
  for (int inc = 0; inc <= 1000; inc++) {
    if (inc == 1000)
      return false;
    snprintf(fullfilename, sizeof(fullfilename), "%s%03d.jpg", filename_base,
             inc);
    if (!sd.exists(fullfilename))
      break;
  }
  // Create the file / cleanup the file
  auto file_cleanup = detail::scoped_cleanup([&file]() {
    file.close();
  });
  if (file.open(fullfilename, FILE_WRITE)) {

    if (file.write(frame->buf, frame->len)) {
      ESP_LOGI("PYCAM", "Saved JPEG to filename %s", fullfilename);
      // flush check what we wrote!

      file.flush();
      return sd.ls(LS_R | LS_DATE | LS_SIZE);
    } else {
      ESP_LOGE("PYCAM", "Couldn't write JPEG data to file");
    }
  }

  // even if it doesnt work out, reset camera size and close file
  return false;
}

/**************************************************************************/
/**
 * @brief Returns the time elapsed since the last call to this function.
 *
 * @details This function calculates the time difference (in milliseconds)
 * between the current time and the last time this function was called. It
 * updates the internal timestamp to the current time at each call.
 *
 * @return The time elapsed (in milliseconds) since the last call to this
 * function.
 */
/**************************************************************************/
uint32_t Adafruit_PyCamera::timestamp(void) {
  uint32_t delta = millis() - _timestamp;
  _timestamp = millis();
  return delta;
}

/**************************************************************************/
/**
 * @brief Prints a timestamped message to the Serial output.
 *
 * @details This function prints a message to the Serial output, prefixed with a
 * timestamp. The timestamp represents the time elapsed in milliseconds since
 * the last call to `timestamp()` function. It is useful for debugging and
 * performance measurement.
 *
 * @param msg The message to be printed along with the timestamp.
 */
/**************************************************************************/
void Adafruit_PyCamera::timestampPrint(const char *msg) {
  ESP_LOGI("PYCAM", "%s: %d ms elapsed", msg, (int)timestamp());
}

/**************************************************************************/
/**
 * @brief Captures a frame from the camera and processes it.
 *
 * @details This function captures a frame from the camera and processes it
 * based on the current pixel format setting. It handles both JPEG and RGB565
 * formats. For JPEG, it scales and draws the image onto a framebuffer. For
 * RGB565, it flips the endians of the frame buffer. This function is essential
 * for capturing and displaying camera frames.
 *
 * @return bool Returns true if the frame was successfully captured and
 * processed, false otherwise.
 */
/**************************************************************************/
bool Adafruit_PyCamera::captureFrame(int x, int y, int width, int height) {
  // Serial.println("Capturing...");
  // esp_err_t res = ESP_OK;
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
  int64_t fr_start = esp_timer_get_time();
#endif

  camera_fb_t *frame = esp_camera_fb_get();
  auto  frame_cleanup = detail::scoped_cleanup([frame]() {
    esp_camera_fb_return(frame);
  });
  if (!frame) {
    ESP_LOGE("PYCAM", "Camera frame capture failed");
    return false;
  }
  //ESP_LOGI("PYCAM", "Frame captured: %d x %d", frame->width, frame->height);

  if (camera_config.pixel_format == PIXFORMAT_JPEG) {

    jpeg_->openRAM(frame->buf, frame->len, &detail::JPEGDraw);

    int scale = 0;
    int xoff = x, yoff = y;
    fb->fillScreen(ST77XX_BLACK);
    // Find the largest scale that fits within the target rectangle
    // Check both width and height to ensure the scaled frame fits
    if (frame->width <= width && frame->height <= height) {
      scale = 0;  // No scaling needed
    } else if ((frame->width / 2) <= width && (frame->height / 2) <= height) {
      scale = JPEG_SCALE_HALF;  // 1/2 scale
    } else if ((frame->width / 4) <= width && (frame->height / 4) <= height) {
      scale = JPEG_SCALE_QUARTER;  // 1/4 scale
    } else {
      scale = JPEG_SCALE_EIGHTH;  // 1/8 scale (guaranteed to fit or be smallest)
    }
    // Clamp offsets to non-negative values to prevent out-of-bounds errors
    if (xoff < 0) xoff = 0;
    if (yoff < 0) yoff = 0;
    //ESP_LOGI("PYCAM", "Decoding JPEG: %d x %d, scale: %d, xoff: %d, yoff: %d", frame->width, frame->height, scale, xoff, yoff);
    jpeg_->setUserPointer(fb);
    jpeg_->setCropArea(0, 0, width, height);
    jpeg_->decode(xoff, yoff, scale);
    jpeg_->close();
  } else if (camera_config.pixel_format == PIXFORMAT_RGB565) {
    // flip endians
    for (uint32_t i = 0; i < frame->len; i += 2) {
      fb->getBuffer()[i + 0] = frame->buf[i + 1];
      fb->getBuffer()[i + 1] = frame->buf[i + 0];
    }
  }

  return true;
}

bool Adafruit_PyCamera::captureFrame(std::function<bool(camera_fb_t*)> hook) {
  camera_fb_t* frame = esp_camera_fb_get();
  auto  frame_cleanup = detail::scoped_cleanup([frame]() {
      esp_camera_fb_return(frame);
  });
  if (!frame) {
    ESP_LOGE(TAG, "Camera frame capture failed");
    return false;
  }
  auto res = hook(frame);
  return res;
}

/**************************************************************************/
/**
 * @brief Blits the current frame buffer to the display.
 *
 * @details This function draws the current frame buffer onto the display at the
 * specified coordinates. It is used to update the display with the latest
 * camera frame. After drawing, it returns the frame buffer to the camera for
 * reuse.
 */
/**************************************************************************/
void Adafruit_PyCamera::blitFrame(void) {
  drawRGBBitmap(0, 0, fb->getBuffer(), 240, 240);
}

/**************************************************************************/
/**
 * @brief Initializes the accelerometer.
 *
 * @details This function initializes the accelerometer by setting up the I2C
 * device, checking the chip ID, and configuring the control registers for
 * normal mode, data rate, resolution, and range. It ensures that the
 * accelerometer is ready for data reading.
 *
 * @return bool Returns true if the accelerometer is successfully initialized,
 * false otherwise.
 */
/**************************************************************************/
bool Adafruit_PyCamera::initAccel(void) {
  lis_dev = new Adafruit_I2CDevice(0x19, &Wire);
  if (!lis_dev->begin()) {
    return false;
  }
  Adafruit_BusIO_Register _chip_id =
      Adafruit_BusIO_Register(lis_dev, Adafruit_PyCamera::LIS3DH_REG_WHOAMI, 1);
  if (_chip_id.read() != 0x33) {
    return false;
  }
  Adafruit_BusIO_Register _ctrl1 =
      Adafruit_BusIO_Register(lis_dev, Adafruit_PyCamera::LIS3DH_REG_CTRL1, 1);
  _ctrl1.write(0x07); // enable all axes, normal mode
  Adafruit_BusIO_RegisterBits data_rate_bits =
      Adafruit_BusIO_RegisterBits(&_ctrl1, 4, 4);
  data_rate_bits.write(0b0111); // set to 400Hz update

  Adafruit_BusIO_Register _ctrl4 =
      Adafruit_BusIO_Register(lis_dev, Adafruit_PyCamera::LIS3DH_REG_CTRL4, 1);
  _ctrl4.write(0x88); // High res & BDU enabled
  Adafruit_BusIO_RegisterBits range_bits =
      Adafruit_BusIO_RegisterBits(&_ctrl4, 2, 4);
  range_bits.write(0b11);

  ESP_LOGI("PYCAM", "Found LIS3DH");
  return true;
}

/**************************************************************************/
/**
 * @brief Reads accelerometer data.
 *
 * @details This function reads the X, Y, and Z acceleration data from the
 * accelerometer. It sets up the register address for auto-increment to read
 * consecutive data registers and then reads the 6 bytes of data corresponding
 * to the X, Y, and Z axes.
 *
 * @param[out] x Pointer to store the X-axis acceleration data.
 * @param[out] y Pointer to store the Y-axis acceleration data.
 * @param[out] z Pointer to store the Z-axis acceleration data.
 * @return bool Returns true if the data is successfully read, false otherwise.
 */
/**************************************************************************/
bool Adafruit_PyCamera::readAccelData(int16_t *x, int16_t *y, int16_t *z) {
  uint8_t register_address = Adafruit_PyCamera::LIS3DH_REG_OUT_X_L;
  register_address |= 0x80; // set [7] for auto-increment

  Adafruit_BusIO_Register xl_data =
      Adafruit_BusIO_Register(lis_dev, register_address, 6);

  uint8_t buffer[6];
  if (!xl_data.read(buffer, 6))
    return false;

  *x = buffer[0];
  *x |= ((uint16_t)buffer[1]) << 8;
  *y = buffer[2];
  *y |= ((uint16_t)buffer[3]) << 8;
  *z = buffer[4];
  *z |= ((uint16_t)buffer[5]) << 8;

  return true;
}

/**************************************************************************/
/**
 * @brief Reads accelerometer data and converts it to g-force values.
 *
 * @details This function reads the raw accelerometer data for X, Y, and Z axes
 * using readAccelData() and then converts these values to g-force. The
 * conversion factor depends on the accelerometer's sensitivity setting (here
 * assumed for 16G range).
 *
 * @param[out] x_g Pointer to store the X-axis acceleration in g-force.
 * @param[out] y_g Pointer to store the Y-axis acceleration in g-force.
 * @param[out] z_g Pointer to store the Z-axis acceleration in g-force.
 * @return bool Returns true if the data is successfully read and converted,
 * false otherwise.
 */
/**************************************************************************/
bool Adafruit_PyCamera::readAccelData(float *x_g, float *y_g, float *z_g) {
  int16_t x, y, z;
  if (!readAccelData(&x, &y, &z))
    return false;

  uint8_t lsb_value = 48; // for 16G
  *x_g = lsb_value * ((float)x / Adafruit_PyCamera::LIS3DH_LSB16_TO_KILO_LSB10);
  *y_g = lsb_value * ((float)y / Adafruit_PyCamera::LIS3DH_LSB16_TO_KILO_LSB10);
  *z_g = lsb_value * ((float)z / Adafruit_PyCamera::LIS3DH_LSB16_TO_KILO_LSB10);
  return true;
}

/**************************************************************************/
/**
 * @brief Scans the I2C bus and prints the addresses of all connected devices.
 *
 * @details This function iterates through all possible I2C addresses (0x00 to
 * 0x7F) and attempts to initiate a transmission to each. If a device
 * acknowledges the transmission, its address is printed to the Serial output.
 * This is useful for debugging and identifying connected I2C devices.
 */
/**************************************************************************/
void Adafruit_PyCamera::I2Cscan(void) {
  Wire.begin();
  ESP_LOGI("PYCAM", "I2C Scan");
  for (int addr = 0; addr <= 0x7F; addr++) {
    Wire.beginTransmission(addr);
    bool found = (Wire.endTransmission() == 0);
    if (found) {
      ESP_LOGI("PYCAM", "Found device at address 0x%02X", addr);
    }
  }
  ESP_LOGI("PYCAM", "Done");
}

/**************************************************************************/
/**
 * @brief Sets the color of the Neopixel.
 *
 * @param c The color to set the Neopixel to, in 32-bit RGB format.
 *
 * @details This function sets the color of the Neopixel LED. It uses the `fill`
 * method of the Adafruit_NeoPixel class to set all pixels to the specified
 * color and then calls `show` to update the LED with the new color.
 */
/**************************************************************************/
void Adafruit_PyCamera::setNeopixel(uint32_t c) {
  pixel.fill(c);
  pixel.show(); // Initialize all pixels to 'off'
}

/**************************************************************************/
/**
 * @brief Sets the color of the Neopixel Ring.
 *
 * @param c The color to set the Neopixel Ring to, in 32-bit RGB format.
 *
 * @details This function sets the color of the Neopixel Ring. It uses the
 * `fill` method of the Adafruit_NeoPixel class to set all pixels in the ring to
 * the specified color and then calls `show` to update the ring with the new
 * color.
 */
/**************************************************************************/
void Adafruit_PyCamera::setRing(uint32_t c) {
  ring.fill(c);
  ring.show(); // Initialize all pixels to 'off'
}

/**************************************************************************/
/*!
    @brief   Input a value 0 to 255 to get a color value. The colours are a
   transition r - g - b - back to r.
    @param  WheelPos The position in the wheel, from 0 to 255
    @returns  The 0xRRGGBB color
*/
/**************************************************************************/
uint32_t Adafruit_PyCamera::Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return Adafruit_NeoPixel::Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return Adafruit_NeoPixel::Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return Adafruit_NeoPixel::Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
