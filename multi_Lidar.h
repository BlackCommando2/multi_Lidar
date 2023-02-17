class multi_Lidar
{
  private:
    int rx_Pin1, tx_Pin1,rx_Pin2,tx_Pin2;
    int strength1,strength2; 
    float temprature1,temprature2,cm_to_inch=2.54;
    unsigned char check1,check2,check3,check4; 
    int i;
    unsigned char uart1[9],uart2[9]; 
    const int HEADER = 0x59; 
    int rec_debug_state1 = 0x01,rec_debug_state2 = 0x01,rec_debug_state3 = 0x01,rec_debug_state4 = 0x01;
    int previousDistanceCm1,previousDistanceInch1,previousDistanceCm2,previousDistanceInch2,serialPort=1;

  public:
    int objectDistanceCm1,objectDistanceInch1,objectDistanceCm2,objectDistanceInch2;
    multi_Lidar(){};

    void set_multi_Lidar1(int rx, int tx)
    {
      this->rx_Pin1 = rx;
      this->tx_Pin1 = tx;
      Serial.begin(115200);
      //Serial.println("Serial");
    }

    void set_multi_Lidar2(int rx, int tx)
    {
      this->rx_Pin2 = rx;
      this->tx_Pin2 = tx;
      Serial2.begin(115200);
      //Serial.println("Serial2");
    }

    void lidar1StartReadings()
    {
      if (Serial.available()) 
      {
        if (rec_debug_state1 == 0x01)
        { //the first byte
          uart1[0] = Serial.read();
          if (uart1[0] == 0x59)
          {
            check1 = uart1[0];
            rec_debug_state1 = 0x02;
          }
        }
        else if (rec_debug_state1 == 0x02)
        { //the second byte
          uart1[1] = Serial.read();
          if (uart1[1] == 0x59)
          {
            check1 += uart1[1];
            rec_debug_state1 = 0x03;
          }
          else {
            rec_debug_state1 = 0x01;
          }
        }
        else if (rec_debug_state1 == 0x03)
        {
          uart1[2] = Serial.read();
          check1 += uart1[2];
          rec_debug_state1 = 0x04;
        }
        else if (rec_debug_state1 == 0x04)
        {
          uart1[3] = Serial.read();
          check1 += uart1[3];
          rec_debug_state1 = 0x05;
        }
        else if (rec_debug_state1 == 0x05)
        {
          uart1[4] = Serial.read();
          check1 += uart1[4];
          rec_debug_state1 = 0x06;
        }
        else if (rec_debug_state1 == 0x06)
        {
          uart1[5] = Serial.read();
          check1 += uart1[5];
          rec_debug_state1 = 0x07;
        }
        else if (rec_debug_state1 == 0x07)
        {
          uart1[6] = Serial.read();
          check1 += uart1[6];
          rec_debug_state1 = 0x08;
        }
        else if (rec_debug_state1 == 0x08)
        {
          uart1[7] = Serial.read();
          check1 += uart1[7];
          rec_debug_state1 = 0x09;
        }
        else if (rec_debug_state1 == 0x09)
        {
          uart1[8] = Serial.read();
          if (uart1[8] == check1)
          {
            objectDistanceCm1 = uart1[2] + uart1[3] * 256; //the distance
            objectDistanceInch1=objectDistanceCm1/cm_to_inch;
            strength1 = uart1[4] + uart1[5] * 256; //the strength
            temprature1 = uart1[6] + uart1[7] * 256; //calculate chip temprature
            temprature1 = temprature1 / 8 - 256;
            //Serial.println("Lidar1: "+String(objectDistanceCm1));
           // Serial.println("Lidar1: "+String(objectDistanceCm1)+","+String(strength1));
            // s.println("Lidar1: "+String(objectDistanceCm1));
            
            while (Serial.available()) {
              Serial.read(); 
            }
            delay(50);
          }
          rec_debug_state1 = 0x01;
        }
      }
      else
      {
          //Serial.println("error in 1");
      }
    }

    void lidar2StartReadings()
    {
      if (Serial2.available()) //check if serial port has data input
      {
        if (rec_debug_state2 == 0x01)
        { //the first byte
          uart2[0] = Serial2.read();
          if (uart2[0] == 0x59)
          {
            check2 = uart2[0];
            rec_debug_state2 = 0x02;
          }
        }
        else if (rec_debug_state2 == 0x02)
        { //the second byte
          uart2[1] = Serial2.read();
          if (uart2[1] == 0x59)
          {
            check2 += uart2[1];
            rec_debug_state2 = 0x03;
          }
          else {
            rec_debug_state2 = 0x01;
          }
        }
        else if (rec_debug_state2 == 0x03)
        {
          uart2[2] = Serial2.read();
          check2 += uart2[2];
          rec_debug_state2 = 0x04;
        }
        else if (rec_debug_state2 == 0x04)
        {
          uart2[3] = Serial2.read();
          check2 += uart2[3];
          rec_debug_state2 = 0x05;
        }
        else if (rec_debug_state2 == 0x05)
        {
          uart2[4] = Serial2.read();
          check2 += uart2[4];
          rec_debug_state2 = 0x06;
        }
        else if (rec_debug_state2 == 0x06)
        {
          uart2[5] = Serial2.read();
          check2 += uart2[5];
          rec_debug_state2 = 0x07;
        }
        else if (rec_debug_state2 == 0x07)
        {
          uart2[6] = Serial2.read();
          check2 += uart2[6];
          rec_debug_state2 = 0x08;
        }
        else if (rec_debug_state2 == 0x08)
        {
          uart2[7] = Serial2.read();
          check2 += uart2[7];
          rec_debug_state2 = 0x09;
        }
        else if (rec_debug_state2 == 0x09)
        {
          uart2[8] = Serial2.read();
          if (uart2[8] == check2)
          {
            objectDistanceCm2 = uart2[2] + uart2[3] * 256; //the distance
            objectDistanceInch2=objectDistanceCm2/cm_to_inch;
            strength2 = uart2[4] + uart2[5] * 256; //the strength
            temprature2 = uart2[6] + uart2[7] * 256; //calculate chip temprature
            temprature2 = temprature2 / 8 - 256;
            //Serial.println("Lidar2: "+String(objectDistanceCm2)+","+String(strength2));
            
            while (Serial2.available()) {
              Serial2.read(); // This part is added becuase some previous packets are//there in the buffer so to clear serial buffer and get fresh data.
            }
            delay(50);
          }
          rec_debug_state2 = 0x01;
        }
      }
      else
      {
        //Serial.println("error in 2");
      }
    }

    int getReadingsCm1()
    {
      //Serial.println("Lidar1: "+String(objectDistanceCm1)+" Lidar2: "+String(objectDistanceCm2));
      return objectDistanceCm1;
    }

    int getReadingsInch1()
    {
      return objectDistanceInch1;
    }

    int getReadingsCmOnChange1(int change=2)
    {
      if (abs(previousDistanceCm1 - objectDistanceCm1) >= change)
      {
        previousDistanceCm1 = objectDistanceCm1;
        //Serial.println("1: "+String(objectDistanceCm1));
        return objectDistanceCm1;
        // Serial.print("objectDistance = ");
        // Serial.println(objectDistance + 5);
      }
      else
      {
          return previousDistanceCm1;
      }
    }

    int getReadingsInchOnChange1(int change=2)
    {
      if (abs(previousDistanceInch1 - objectDistanceInch1) >= change)
      {
        previousDistanceInch1 = objectDistanceInch1;
        return objectDistanceInch1;
        // Serial.print("objectDistance = ");
        // Serial.println(objectDistance + 5);
      }
    }

    int getReadingsCm2()
    {
      return objectDistanceCm2;
    }

    int getReadingsInch2()
    {
      return objectDistanceInch2;
    }

    int getReadingsCmOnChange2(int change=2)
    {
      if (abs(previousDistanceCm2 - objectDistanceCm2) >= change)
      {
        previousDistanceCm2 = objectDistanceCm2;
        //Serial.println("2: "+String(objectDistanceCm2));
        return objectDistanceCm2;
        // Serial.print("objectDistance = ");
        // Serial.println(objectDistance + 5);
      }
      else
      {
          return previousDistanceCm2;
      }
    }

    int getReadingsInchOnChange2(int change=2)
    {
      if (abs(previousDistanceInch2 - objectDistanceInch2) >= change)
      {
        previousDistanceInch2 = objectDistanceInch2;
        return objectDistanceInch1;
        // Serial.print("objectDistance = ");
        // Serial.println(objectDistance + 5);
      }
    }
};