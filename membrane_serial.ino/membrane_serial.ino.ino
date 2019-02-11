int baud = 9600;
boolean communicationStarted = false;
enum inputType {bluetooth = 600, street = 120, line = 20};  //1024-300  300-40  40-0
enum statusType {idle = 0, read = 1, write = 2};
enum inputType inputState;
enum statusType status = idle;
int writeCounter = 0;
int writeCounterThreshold = 20;//20; //amount of cycles after last local change that state==write should remain set until it goes back to idle
int readCounter = 0;
int readCounterThreshold = 20;//20; //amount of cycles until it switches from idle to read
int timeout = 0; //timeout if no OSC communication

class Poti
{
  private:
    float deviation = 1.03f;  //1.03f
    int idleDeviation = 5;  //2
    float alpha = 0.4;
    boolean isLogarithmic;
  public:
    int enablePin, in1, in2, potiPin, potiVal, targetVal;

    Poti(int _enablePin, int _in1, int _in2, int _potiPin, boolean _isLogarithmic) {
      enablePin = _enablePin;
      in1 = _in1;
      in2 = _in2;
      potiPin = _potiPin;
      isLogarithmic = _isLogarithmic;
      setPinModes();
      potiVal = 1024 - analogRead(potiPin);
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
      //potiVal = analogRead(potiPin);
      potiVal = alpha * (1024 - analogRead(potiPin)) + (1 - alpha) * potiVal; //exponential smoothing
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

      if (status == 1 && communicationStarted) {
        if (potiVal > lowerDeviation(targetVal)) turnRight();
        if (potiVal < upperDeviation(targetVal)) turnLeft();
      }

    }
    boolean isSynced(int32_t _val) {
      //deviation for logarithmic scale!
      if ((potiVal >= lowerDeviation(_val)) && (potiVal <= upperDeviation(_val)))return true;
      //if ((potiVal >= _val - idleDeviation) && (potiVal <= _val + idleDeviation))return true;
      else return false;
    }

    boolean hasChanged(int32_t val_) {
      if ((potiVal > val_ + idleDeviation) || (potiVal < val_ - idleDeviation))return true;
      else return false;
    }

    float upperDeviation(float reference) {
      if (isLogarithmic) {
        //calculate logarithmic deviations based on reference value
        //return pow(10, log10(reference) + log10(deviation));
        return 1.05 * (reference + 1);
      } else {
        //linear
        return reference + 15;
      }

    }

    float lowerDeviation(float reference) {
      if (isLogarithmic) {
        //calculate logarithmic deviations based on reference value
        //return pow(10, log10(reference) - log10(deviation));
        return 0.95 * reference;
      } else {
        //linear
        return reference - 15;
      }
    }
};

Poti * input, *membrane, *volume;




#define buttonLED 11
#define button 12

#define ledA A3
#define ledB A4
#define ledC A5

boolean on = true;
boolean buttonPressed, buttonPressed_, buttonReleased;
boolean synced, changed;

//global values from incoming Serial Bundle
int32_t _on, _membrane, _volume, _inputState;

//device values from previous loop
int32_t on_, membrane_, volume_, inputState_, input_;


String inputString = "";         // a String to hold incoming data
boolean stringComplete = false;  // whether the string is complete

void setup() {
  input = new Poti(3, 2, 4, A0, true);
  membrane = new Poti(6, 5, 7, A1, false);
  volume = new Poti(9, 8, 10, A2, true);

  pinMode(ledA, OUTPUT);
  digitalWrite(ledA, HIGH);
  pinMode(ledB, OUTPUT);
  digitalWrite(ledB, HIGH);
  pinMode(ledC, OUTPUT);
  digitalWrite(ledC, HIGH);
  pinMode(buttonLED, OUTPUT);
  digitalWrite(buttonLED, HIGH);
  pinMode(button, INPUT_PULLUP);

  // initialize serial:
  Serial.begin(baud);

  // reserve 200 bytes for the inputString:
  inputString.reserve(200);


  status = idle;
}

void loop() {

  buttonPressed = digitalRead(button);

  if (digitalRead(button)) buttonPressed = true;

  input->read();
  membrane->read();
  volume->read();
  updateInputState();

  //update potis
  input->update();
  membrane->update();
  volume->update();

  // read the state of the pushbutton value:
  if (digitalRead(button)) buttonPressed = true;

  //also needs to listen for incoming bundles!
  //listenForIncomingBundles();
  listenForIncomingBundlesViaSerial();

  // read the state of the pushbutton value:
  if (digitalRead(button)) buttonPressed = true;


  digitalWrite(buttonLED, on);
  if (!on) {
    digitalWrite(ledA, LOW);
    digitalWrite(ledB, LOW);
    digitalWrite(ledC, LOW);
  }


  //at the end of each loop: update previous cycle values_
  update_();
}

void update_() {
  membrane_ = membrane->potiVal;
  volume_ = volume->potiVal;
  on_ = on;
  inputState_ = inputState;
  input_ = input->potiVal;
  buttonPressed_ = buttonPressed;
}

void sendValuesViaSerial() {
  Serial.print(status);
  Serial.print(",");
  Serial.print(synced);
  Serial.print(",");
  Serial.print(changed);
  Serial.print(",");
  Serial.print(on);
  Serial.print(",");
  Serial.print(inputState);
  Serial.print(",");
  Serial.print(input->potiVal);
  Serial.print(",");
  Serial.print(membrane->potiVal);
  Serial.print(",");
  Serial.println(volume->potiVal);
}


/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
  // read the state of the pushbutton value:
  if (digitalRead(button)) buttonPressed = true;


  while (Serial.available()) {
    // read the state of the pushbutton value:
    if (digitalRead(button)) buttonPressed = true;

    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
    // read the state of the pushbutton value:
    if (digitalRead(button)) buttonPressed = true;
  }
}

void listenForIncomingBundlesViaSerial() {
  // read the state of the pushbutton value:
  if (digitalRead(button)) buttonPressed = true;

  if (stringComplete) {
    //on,inputState,membrane,volume
    _on = Serial.parseInt();
    _inputState = Serial.parseInt();
    _membrane = Serial.parseInt();
    _volume = Serial.parseInt();

    // clear the string:
    inputString = "";
    stringComplete = false;
    compare();
    sendValuesViaSerial();
    communicationStarted = true;

    // read the state of the pushbutton value:
    if (digitalRead(button)) buttonPressed = true;
  }

}


void compare() {

  // read the state of the pushbutton value:
  if (digitalRead(button)) buttonPressed = true;
  if (buttonPressed && !buttonPressed_) {
    on = !on;
  }

  //check if local values are synced to global values
  synced = ((on == _on) && (inputState == _inputState) && (membrane->isSynced(_membrane)) && (volume->isSynced(_volume)));

  //check if local values have changed since last loop
  changed = ((on != on_) || (inputState != inputState_) || (membrane->hasChanged(membrane_)) || (volume->hasChanged(volume_)) || (input->hasChanged(input_)));



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
  //readButton();
  volume->targetVal = volume->potiVal;
  membrane->targetVal = membrane->potiVal;
  input->targetVal = input->potiVal;
}

void updateInputState() {

  // read the state of the pushbutton value:
  if (digitalRead(button)) buttonPressed = true;

  //1024-300  300-40  40-0
  if (input->potiVal >= 300 && on) {
    digitalWrite(ledA, LOW);
    digitalWrite(ledB, LOW);
    digitalWrite(ledC, HIGH);
    inputState = bluetooth;
  }
  else if ((input->potiVal >= 40) && (input->potiVal < 300) && on) {
    digitalWrite(ledA, LOW);
    digitalWrite(ledB, HIGH);
    digitalWrite(ledC, LOW);
    inputState = street;
  } else {
    if (on) {
      digitalWrite(ledA, HIGH);
      digitalWrite(ledB, LOW);
      digitalWrite(ledC, LOW);
      inputState = line;
    }
  }
}
