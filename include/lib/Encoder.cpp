/* 
 * File:   Encoder.cpp
 * Author: root
 *
 * Created on March 8, 2018, 11:18 AM
 */

#include "Encoder.h"
//**************************************************************************
// on Attach Handler
//**************************************************************************
static void CCONV onAttachHandler(PhidgetHandle h, void *ctx) {
  int* channel = (int*) ctx;
  printf("channel %d on yor device attached\n", *channel);
}
//**************************************************************************
// on Detach Handler
//**************************************************************************
static void CCONV onDetachHandler(PhidgetHandle h, void *ctx) {
  int* channel = (int*) ctx;
  printf("channel %d on your device detached\n", *channel);
}
//**************************************************************************
// error Handler
//**************************************************************************
static void CCONV errorHandler(PhidgetHandle h, void *ctx,
                               Phidget_ErrorEventCode errorCode, const char *errorString) {
  fprintf(stderr, "Error: %s (%d)\n", errorString, errorCode);
}
//**************************************************************************
// on Position Change Handler
//**************************************************************************
static void CCONV onPositionChangeHandler(PhidgetEncoderHandle ch,
                                          void *ctx, int positionChange, double timeChange,
                                          int indexTriggered) {
  int* channel = (int*) ctx;
  tmp_counts[*channel] = positionChange;
  //printf("timeChange %f \n", timeChange);
}
//**************************************************************************
// Encoder
//**************************************************************************
Encoder::Encoder(){
}
//**************************************************************************
// Encoder
//**************************************************************************
Encoder::Encoder(bool x) {
  // x is just to differ methods
  // Initialize parameters
  _count[0] = 0;
  _count[1] = 0;
  _count[2] = 0;
  _index[0] = 0;
  _index[1] = 1;
  _index[2] = 2;
  _reset_index[0] = 0;
  _reset_index[1] = 0;
  _reset_index[2] = 0;
  _serial = 0;
  _is_enbaled[0] = false;
  _is_enbaled[1] = false;
  _is_enbaled[2] = false;
  PhidgetReturnCode res;
  const char *errs;
  // initialize encoders
  for (int i = 0; i < 3; ++i) {
    if (init(i)) {
      // Set encoder to a specific channel
      setChannel(i);

      // get info of encoder device
      getInfo(i);

      // Open the channel synchronously: waiting a maximum of 5 seconds.
      res = Phidget_openWaitForAttachment((PhidgetHandle) _eh[i], 5000);
      if (res != EPHIDGET_OK) {
        if (res == EPHIDGET_TIMEOUT) {
          printf("Channel did not attach after 5 seconds: please check that the device is attached\n");
        } else {
          Phidget_getErrorDescription(res, &errs);
          fprintf(stderr, "failed to open channel:%s\n", errs);
        }
      }
      if (res != EPHIDGET_OK) {
        printf("Initialization of channel %d on device %d is successful\n", i, _serial);
      }
      setEnable(true, i);

      uint32_t tmp;
      // Set data interval for each channel to 8 ms
      PhidgetEncoder_setDataInterval(_eh[i], (uint32_t) 8);
      PhidgetEncoder_getDataInterval( _eh[i], &tmp);
      printf("PhidgetEncoder_getDataInterval %u %u \n", (uint32_t) 8, tmp);

      // Set data interval for device to 8 ms
      Phidget_setDataInterval((PhidgetHandle) _eh[i], (uint32_t) 8);
      Phidget_getDataInterval((PhidgetHandle) _eh[i], &tmp);
      printf("Phidget_getDataInterval %u %u \n", (uint32_t) 8, tmp);

      // The channel firing events every DataInterval
      PhidgetEncoder_setPositionChangeTrigger( _eh[i],(uint32_t) 0);
      PhidgetEncoder_getPositionChangeTrigger( _eh[i], &tmp);
      printf("PhidgetEncoder_getPositionChangeTrigger %u %u \n", (uint32_t) 0, tmp);

    } else {
      printf("Initialization of channel %d on device %d is unsuccessful\n", i, _serial);
    }
  }
}
//**************************************************************************
// ~Encoder
//**************************************************************************
Encoder::~Encoder() {
  for (int i = 0; i < 3; ++i) {
    Phidget_close((PhidgetHandle) _eh[i]);
    PhidgetEncoder_delete(&_eh[i]);
  }
}
//**************************************************************************
// setChannel
//**************************************************************************
void Encoder::setChannel(int i) {
  _channel[i] = i;
  Phidget_setChannel((PhidgetHandle) _eh[i], _channel[i]);
}
//**************************************************************************
// enablResetIndex
//**************************************************************************
void Encoder::enablResetIndex( bool enable[3]) {
  for (int ch = 0; ch < 3; ++ch) {
    _reset_index[ch] = enable[ch];
  }
}
//**************************************************************************
// updateCounts
//**************************************************************************
void Encoder::updateCounts() {
  for (int ch = 0; ch < 3; ++ch) {
    PhidgetEncoder_getPosition(_eh[ch], &_count[ch]);
    PhidgetEncoder_getIndexPosition(_eh[ch], &_index[ch]);
    if (_reset_index[ch])
      _count[ch] = _count[ch] - _index[ch];
  }
}
//**************************************************************************
// getCounts
//**************************************************************************
void Encoder::getCounts(long counts[]) const {
  for (int ch = 0; ch < 3; ++ch) {
    counts[ch] = _count[ch];
  }
}
//**************************************************************************
// readAnglesRad
//**************************************************************************
void Encoder::readAnglesRad(float angle[]) const {
  static float count = 0;
  count = count + 0.1;
  for (int ch = 0; ch < 3; ++ch) {
    //angle[ch] = tmp_counts[ch] / MAXCOUNT * 2 * PI;
    angle[ch] = _count[ch] / MAXCOUNT * 2 * PI;
    }
}
//**************************************************************************
// readAnglesDeg
//**************************************************************************
void Encoder::readAnglesDeg(float angle[]) const {
  for (int ch = 0; ch < 3; ++ch) {
    angle[ch] = _count[ch] / MAXCOUNT * 360.0;
  }
}
//**************************************************************************
// setCount
//**************************************************************************
void Encoder::setCount(const int ch, const long int count) {
  PhidgetEncoder_setPosition(_eh[ch], count);
  _count[ch] = count;
}
//**************************************************************************
// setCounts
//**************************************************************************
void Encoder::setCounts(const long int count[]) {
  for (int ch = 0; ch < 3; ++ch) {
    setCount(ch, count[ch]);
  }
}
//**************************************************************************
// setEnable
//**************************************************************************
bool Encoder::setEnable(bool en, int i) {
  PhidgetReturnCode res;

  // Change encoder enable case
  res = PhidgetEncoder_setEnabled(_eh[i], 1);

  // No error occurred
  if (res == EPHIDGET_OK) {
    if (en) {
      printf("Encoder on ch:%d is now enable\n", _channel[i]);
      _is_enbaled[i] = true;
    } else {
      printf("Encoder on ch:%d is now disabled\n", _channel[i]);
      _is_enbaled[i] = true;
    }
    return true;
  } else {
    // An error occurred
    printf("Can not change the enable of encoder for ch:%d error code %x\n", _channel[i], res);
    return false;
  }
}
//**************************************************************************
// init
//**************************************************************************
bool Encoder::init(int i) {
  PhidgetReturnCode res;

  // Enable logging to stdout
  PhidgetLog_enable(PHIDGET_LOG_INFO, NULL);

  // Create encoder handler
  res = PhidgetEncoder_create(&_eh[i]);
  if (res != EPHIDGET_OK) {
    fprintf(stderr, "failed to create voltage ratio input channel\n");
    return false;
  }

  // Set attach handler function
  res = Phidget_setOnAttachHandler((PhidgetHandle) _eh[i], onAttachHandler, &_channel[i]);
  if (res != EPHIDGET_OK) {
    fprintf(stderr, "failed to assign on attach handler\n");
    return false;
  }

  // Set detach handler function
  res = Phidget_setOnDetachHandler((PhidgetHandle) _eh[i], onDetachHandler, &_channel[i]);
  if (res != EPHIDGET_OK) {
    fprintf(stderr, "failed to assign on detach handler\n");
    return false;
  }

  // Set error handler function
  res = Phidget_setOnErrorHandler((PhidgetHandle) _eh[i], errorHandler, &_channel[i]);
  if (res != EPHIDGET_OK) {
    fprintf(stderr, "failed to assign on error handler\n");
    return false;
  }

//  res = PhidgetEncoder_setOnPositionChangeHandler(_eh[i], onPositionChangeHandler, &_channel[i]);
//  if (res != EPHIDGET_OK) {
//    fprintf(stderr, "failed to assign OnPositionChange handler\n");
//    return false;
//  }

  return true;
}
//**************************************************************************
// getInfo
//**************************************************************************
bool Encoder::getInfo(int i) {
  PhidgetReturnCode res;

  res = Phidget_getDeviceSerialNumber((PhidgetHandle) _eh[i], &_serial);
  if (res != EPHIDGET_OK) {
    fprintf(stderr, "failed to get device serial number\n");
    return false;
  }

  res = Phidget_getChannel((PhidgetHandle) _eh[i], &_channel[i]);
  if (res != EPHIDGET_OK) {
    fprintf(stderr, "failed to get channel number\n");
    return false;
  }

  return true;
}
