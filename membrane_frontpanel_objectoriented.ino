#include <OSCMessage.h>
#include <OSCBundle.h>
#include <OSCBoards.h>
#ifdef BOARD_HAS_USB_SERIAL
#include <SLIPEncodedUSBSerial.h>
SLIPEncodedUSBSerial SLIPSerial( thisBoardsSerialUSB );
#else
#include <SLIPEncodedSerial.h>
SLIPEncodedSerial SLIPSerial(Serial); // Change to Serial1 or Serial2 etc. for boards with multiple serial ports that don’t have Serial
#endif


enum inputType {bluetooth = 0, street = 890, line = 1024};
enum statusType {idle = 0, read = 1, write = 2};
enum inputType inputState;
enum statusType status = idle;
int writeCounter = 0;
int writeCounterThreshold = 20; //amount of cycles after last local change that state==write should remain set until it goes back to idle
int readCounter = 0;
int readCounterThreshold = 2; //amount of cycles until it switches from idle to read


class Poti
{
  private:
    float deviation = 1.03f;
    int idleDeviation = 2;
  public:
    int enablePin, in1, in2, potiPin, potiVal, targetVal;

    Poti(int _enablePin, int _in1, int _in2, int _potiPin) {
      enablePin = _enablePin;
      in1 = _in1;
      in2 = _in2;
      potiPin = _potiPin;
      setPinModes();
    }
    void setPinModes() {
      pinMode(enablePin, OUTPUT);
      pinMode(in1, OUTPUT);
      pinMode(in2, OUTPUT);
      pinMode(potiPin, INPUT);
    }
    void turnRight() {
      digitalWrite(enablePin, HIGH);
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
    }
    void turnLeft() {
      digitalWrite(enablePin, HIGH);
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
    }
    void stopMotor() {
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      digitalWrite(enablePin, LOW);
    }
    void read() {
      potiVal = analogRead(potiPin);
    }
    void update() {
      if (status == write) {
        stopMotor();
        return;
      }
      if ((potiVal >= lowerDeviation(targetVal)) && (potiVal <= upperDeviation(targetVal))) {
        stopMotor();
        return;
      }
      if (potiVal < lowerDeviation(targetVal)) turnRight();
      if (potiVal > upperDeviation(targetVal)) turnLeft();
    }
    boolean isSynced(int32_t _val) {
      //deviation for logarithmic scale!
      if ((potiVal >= lowerDeviation(_val)) && (potiVal <= upperDeviation(_val)))return true;
      else return false;
    }

    boolean hasChanged(int32_t val_) {
      if ((potiVal > val_ + idleDeviation) || (potiVal < val_ - idleDeviation))return true;
      else return false;
    }

    //calculate logarithmic deviations based on reference value
    float upperDeviation(float reference) {
      return pow(10, log10(reference) + log10(deviation));
    }

    float lowerDeviation(float reference) {
      return pow(10, log10(reference) - log10(deviation));
    }
};

Poti *input, *membrane, *volume;




#define buttonLED 11
#define button 12

#define ledA A3
#define ledB A4
#define ledC A5

boolean on = true;
boolean buttonPressed, buttonPressed_, buttonReleased;
boolean synced, changed;
unsigned long lastDebounceTime = 0;  // the last time the button pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

//global values from incoming OSC Bundle
int32_t _on, _membrane, _volume, _inputState;

//device values from previous loop
int32_t on_, membrane_, volume_, inputState_;




void setup() {
  input = new Poti(3, 2, 4, A0);
  membrane = new Poti(6, 5, 7, A1);
  volume = new Poti(9, 8, 10, A2);

  pinMode(ledA, OUTPUT);
  digitalWrite(ledA, HIGH);
  pinMode(ledB, OUTPUT);
  digitalWrite(ledB, HIGH);
  pinMode(ledC, OUTPUT);
  digitalWrite(ledC, HIGH);
  pinMode(buttonLED, OUTPUT);
  digitalWrite(buttonLED, HIGH);
  pinMode(button, INPUT);

  //begin SLIPSerial just like Serial
  SLIPSerial.begin(57600);   // set this as high as you can reliably run on your platform


  //init values
  input->read();
  membrane->read();
  volume->read();
  updateInputState();
  update_();
  volume->targetVal = volume->potiVal;
  membrane->targetVal = membrane->potiVal;
  input->targetVal = inputState;

}

void loop() {

  input->read();
  membrane->read();
  volume->read();
  updateInputState();


  //update potis
  input->update();
  membrane->update();
  volume->update();


  //also needs to listen for incoming bundles!
  listenForIncomingBundles();



  //at the end of each loop: update previous cycle values_
  update_();
}

void readButton() {
  // read the state of the pushbutton value:
  buttonPressed = digitalRead(button);
  if (buttonPressed != buttonPressed_ ) {
    lastDebounceTime = millis();
    buttonReleased = true;
  }

  if ((millis() - lastDebounceTime) > debounceDelay && buttonReleased) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:
    buttonReleased = false;
    on = !on;
  }
  digitalWrite(buttonLED, on);
}

void update_() {
  membrane_ = membrane->potiVal;
  volume_ = volume->potiVal;
  on_ = on;
  inputState_ = inputState;
  buttonPressed_ = buttonPressed;
}

void listenForIncomingBundles() {
  OSCBundle bundleIN;
  int size;
  while (!SLIPSerial.endofPacket())
    if ( (size = SLIPSerial.available()) > 0)
    {
      while (size--)
        bundleIN.fill(SLIPSerial.read());
    }

  if (!bundleIN.hasError()) {
    bundleIN.dispatch("/on", _getOn);
    bundleIN.dispatch("/inputState", _getInputState);
    bundleIN.dispatch("/membrane", _getMembrane);
    bundleIN.dispatch("/volume", _getVolume);
    compare();
    sendOSCBundle();
  }
}

void _getOn(OSCMessage &msg) {
  if (msg.isInt(0))
  {
    _on = msg.getInt(0);
  }
}

void _getInputState(OSCMessage &msg) {
  if (msg.isInt(0))
  {
    _inputState = inputType(msg.getInt(0));
  }
}

void _getMembrane(OSCMessage &msg) {
  if (msg.isInt(0))
  {
    _membrane = msg.getInt(0);
  }
}

void _getVolume(OSCMessage &msg) {
  if (msg.isInt(0))
  {
    _volume = msg.getInt(0);
  }
}

void compare() {
  //check if local values are synced to global values
  synced = ((on == _on) && (inputState == _inputState) && (membrane->isSynced(_membrane)) && (volume->isSynced(_volume)));

  //check if local values have changed since last loop
  changed = ((on != on_) || (inputState != inputState_) || (membrane->hasChanged(membrane_)) || (volume->hasChanged(volume_)));


  switch (status) {
    case idle:
      if (synced) {
        readCounter = 0;
      }
      if (!synced) {
        readCounter++;
      }
      if (readCounter >= readCounterThreshold) {
        status = read;
        return;
      }
      if (!synced && changed) {
        writeCounter = 0;
        status = write;
        return;
      }
      if (synced && changed) {
        writeCounter = 0;
        status = write;
        return;
      }
      break;
    case read:
      if (synced && !changed) {
        status = idle;
        return;
      }
      if (!synced && !changed) {
        adaptRead();
        return;
      }
      if (!synced && changed) {
        adaptRead();
        return;
      }
      if (synced && changed) {
        status = write;
        return;
      }
      break;
    case write:
      writeCounter++;
      if (synced && !changed && writeCounter >= writeCounterThreshold) {
        status = idle;
        return;
      }
      if (!synced && !changed) {
        adaptWrite();
        return;
      }
      if (!synced && changed) {
        writeCounter = 0;
        adaptWrite();
        return;
      }
      if (synced && changed) {
        writeCounter = 0;
        adaptWrite();
        return;
      }
      break;
  }
}

//takeover changes from incoming values
void adaptRead() {
  on = _on;
  inputState = (inputType)_inputState;
  volume->targetVal = _volume;
  membrane->targetVal = _membrane;
  input->targetVal = _inputState;
}

//takeover changes from local interaction
void adaptWrite() {
  readButton();
  volume->targetVal = volume->potiVal;
  membrane->targetVal = membrane->potiVal;
  input->targetVal = input->potiVal;
}


void updateInputState() {
  if (input->potiVal < 600) {
    digitalWrite(ledA, HIGH);
    digitalWrite(ledB, LOW);
    digitalWrite(ledC, LOW);
    inputState = bluetooth;
  }
  else if ((input->potiVal >= 600) && (input->potiVal < 960)) {
    digitalWrite(ledA, LOW);
    digitalWrite(ledB, HIGH);
    digitalWrite(ledC, LOW);
    inputState = street;
  } else {
    digitalWrite(ledA, LOW);
    digitalWrite(ledB, LOW);
    digitalWrite(ledC, HIGH);
    inputState = line;
  }
}

void sendOSCBundle() {
  //declare the bundle
  OSCBundle bndl;
  //BOSCBundle's add' returns the OSCMessage so the message's 'add' can be composed together
  bndl.add("/status").add((int32_t)status);
  bndl.add("/on").add((int32_t)on);
  bndl.add("/inputState").add((int32_t)inputState);
  bndl.add("/input").add((int32_t)input->potiVal);
  bndl.add("/membrane").add((int32_t)membrane->potiVal);
  bndl.add("/volume").add((int32_t)volume->potiVal);
  bndl.add("/synced").add((int32_t)synced);
  bndl.add("/changed").add((int32_t)changed);

  SLIPSerial.beginPacket();
  bndl.send(SLIPSerial); // send the bytes to the SLIP stream
  SLIPSerial.endPacket(); // mark the end of the OSC Packet
  bndl.empty(); // empty the bundle to free room for a new one
  delay(100);
}



/*
  void sendOSCMessage() {
  //the message wants an OSC address as first argument
  OSCMessage msg("/inputState");
  msg.add((int32_t)inputState);

  SLIPSerial.beginPacket();
  msg.send(SLIPSerial); // send the bytes to the SLIP stream
  SLIPSerial.endPacket(); // mark the end of the OSC Packet
  msg.empty(); // free space occupied by message
  }

  void logPotis() {
  Serial.print(inputState);
  Serial.print("\t");
  Serial.print(input->potiVal);
  Serial.print("\t");
  Serial.print(membrane->potiVal);
  Serial.print("\t");
  Serial.println(volume->potiVal);
  }*/
