// ADDED TO COPE WITH ENCODERS
// ---
// https://playground.arduino.cc/Main/RotaryEncoders#Example16
//
// 'threshold' is the Debounce Adjustment factor for the Rotary Encoder.
//
// The threshold value I'm using limits it to 100 half pulses a second
//

// My encoder has 12 pulses per 360deg rotation and the specs say
// it is rated at a maximum of 100rpm.


// This threshold will permit my encoder to reach 250rpm so if it was connected
// to a motor instead of a manually operated knob I
// might possibly need to adjust it to 25000. However, this threshold
// value is working perfectly for my situation

volatile unsigned long threshold = 100;  //<-- Test value // 500 (default value); // 10000; // Check the above text

int i;  // This is a generic counter
//  ------------------------------------------------ //
//  ------------------------------------------------ //

//  ------------------------------------------------ //
//    Working variables for the interrupt routines   //
//  ------------------------------------------------ //
volatile unsigned long int0time = 0;
volatile unsigned long int1time = 0;

volatile uint8_t int0signal = 0;  // Channel A's value of encoder
volatile uint8_t int1signal = 0;  // Channel B's value of encoder

volatile uint8_t int0history = 0;
volatile uint8_t int1history = 0;
//  ------------------------------------------------ //
//  ------------------------------------------------ //
// Broches PWM utilisées (attention: ceci doit correspondre à la configuration des cavaliers)
#define pwmMoteur1 10
#define pwmMoteur2 9
//  ------------------------------------------------ //
//  ------------------------------------------------ //

//  ------------------------------------------------ //
//  ------------------  ENCODER PINS --------------- //
int pin_A = 2;  // Output A of encoder // Yellow Wire (is A according to datasheet)
int pin_B = 3;  // Output B of encoder // White Wire (is B according to datasheet)

volatile long code = 0;  //pow(2,20); // Encoder's value (centered in 2^20);

const long Enc_Resolution = 8256;  // Check Motor documentation Or Matlab code (48*172)
//  ------------------------------------------------ //
//  ------------------------------------------------ //

//  ------------------------------------------------ //
//  ----------------- LOOP PARAMETERS -------------- //
//  ------------------------------------------------ //

int Boucle = 0;  // Type of loop (0 : open loop / 1 : closed loop)

//---- OPEN LOOP ----//
long Zero_Duty = 0;           //
long Duty_Cycle = Zero_Duty;  // PWM/Signal received in serial port
long PWM = Zero_Duty;         // PWM/signal to send to the motor

//--- CLOSED LOOP ---//
float Consigne = 0;  // In the case of speed control
float C = Consigne;  // Speed setpoint received in serial port

// --- Timer : FIXED UNIFORM SAMPLING --- //
const unsigned long Ts_us = 2000;              // 10 ms → 100 Hz (2000 micro -> 2ms 500Hz)
const unsigned long SimulationTime = 3000000;  // time of simulation (in microsecond);

unsigned long tinit = 0;  // Time's value (in microsecond) // Comes from micros
unsigned long last_time = 0;
unsigned long now;
unsigned long time_rel;

// tinit -> time at which the open / closed loop starts
// Time -> elapsed time  since the beginning of the open / closed loop

// --- CONTROLLER PART --- //
float Gain;           // Gain's value of the closed loop (propotional)
float Gain_Integral;  // Gain's value of the closed loop  (integral)
float Integral_Part = 0;
long Control = 0;        // PWM/signal to send to the motor
long Control_unSat = 0;  // PWM/signal to send to the motor before saturation
float Previous_Time = 0;
bool Clamp = false;
bool Condition1 = false;
bool Condition2 = false;
int8_t sign1;
int8_t sign2;


// ----------- SPEED COMPUTATION ------------ //
float dt = 1, dCode = 0;  // Time values used in order to compute speed
const int speed_comp_last = 1;
float codes4Speed[speed_comp_last + 1], times4Speed[speed_comp_last + 1];  // Length Should be equal to speed_comp_last+1

volatile float speed_measurement = 0, speed_measurement_Converted = 0;  // Computed speed
float output_speed = 0, alpha_filter = 0.9355;
// alpha ~ exp(-Ts/tau)

//  ------------------------------------------------ //
//  ------------------------------------------------ //
//  ------------------------------------------------ //
// ----        INTERRUPTS RELATED STUFF          ---- //
// 16000000 / 1024(prescaler) => period of 64 mu.sec
// 1 s corresponds to 15625 periods and 15625 = 5^6 = 5^3 5^3 = 125*125

long Num_Interrupt_counts = 125;
long Wanted_Size_TCNT2 = 125;
long Size_TCNT2 = 256;
long Interrupt_Count = 0;

//  ------------------------------------------------ //
//  ------------------------------------------------ //

// ------------------------------------------------------------------- //
//----------------------- FUNCTIONS PROTOTYPES ------------------------//
// ------------------------------------------------------------------ //
void Send_Data(long t, long u, float y, float C, float Integral);  // Function to send data in serial port
void Send_Motor(long Control);

// ----------------------------------------------------------------- //
//--------------------  Setup Routine (VOID) ------------------------//
// ----------------------------------------------------------------- //
void setup() {
  // --- cf. 3 Sigma Datasheet --- //
  // Définition des broches PWM en sortie
  pinMode(pwmMoteur1, OUTPUT);
  pinMode(pwmMoteur2, OUTPUT);

  // --- Encoder pins and variables Setup --- //
  pinMode(pin_A, INPUT);  // To configure input
  pinMode(pin_B, INPUT);

  digitalWrite(pin_A, HIGH);  //Enables Internal PullUP
  digitalWrite(pin_B, HIGH);  //Enables Internal PullUP
                              // https://www.arduino.cc/reference/en/language/functions/digital-io/digitalwrite/

  // --- Encoder Interrupts Setup --- //
  attachInterrupt(INT0, Tic_A, CHANGE);  // Encoder Tic on channel A // INT0 is the pin 2 interrupt
  attachInterrupt(INT1, Tic_B, CHANGE);  // Encoder Tic on channel B // INT0 is the pin 3 interrupt

  // --- Serial Com setup --- //
  Serial.begin(115200);                               // Define the baudrate (byte per second)
  while (Serial.available() > 0) { Serial.read(); };  // Clear the serial port
}

// ----------------------------------------------------------------- //
//------------------------    MAIN LOOP   ---------------------------//
// ----------------------------------------------------------------- //
void loop() {

  while (Serial.available() < 12)
    ;  // While there are less of 12 bytes in serial port, do nothing

  Interrupt_Count = 0;            // Reset the counter that allows STARTUP after 1s
  Boucle = (Serial.read() - 48);  // Read byte's value in ASCII and convert it
  switch (Boucle) {               // Begin of switch case

    case 0:  // Therefore open loop
      for (i = 1; i <= 3; i++) {
        Duty_Cycle += (Serial.read() - 48) * pow(10, 3 - i);  // Recover the PWM's Value centered in 500
      }
      Duty_Cycle -= 500;                                 // Center PWM's value in 0
                                                         //  Duty_Cycle = constrain(Duty_Cycle,Zero_Duty-255,Zero_Duty+255); // CONSTRAIN CONTROL WITHIN INTERVAL [-255, 255] DONE in SEND_Motor
      PWM = Zero_Duty;                                   // Reinitialize PWM's value to send to the motor
      while (Serial.available() > 0) { Serial.read(); }  // Flush Serial Port Buffer

      tinit = micros();  // Read time's value since the program's begining
      last_time = tinit;

      while ((last_time - tinit) < SimulationTime) {  // while the simulation is not terminated
        now = micros();                               // Measure Time until the next sample time

        if (now - last_time >= Ts_us) {
          last_time += Ts_us;
          for (i = 0; i <= speed_comp_last; i++) {
            codes4Speed[i] = codes4Speed[i + 1];
          }
          // Nouvelle Mesure
          codes4Speed[speed_comp_last] = code;  // code -> encoder value

          // Calcul de la vitesse Brute
          dCode = codes4Speed[speed_comp_last] - codes4Speed[0];  // dcode in Ticks
          dt = Ts_us * speed_comp_last;                           // microsec

          speed_measurement = dCode / dt;  // float(dCode) / float(dt); // speed (numerically) computed in Ticks/microsec

          // Calcul de la vitesse convertie
          speed_measurement_Converted = speed_measurement * 6 * pow(10, 7) / Enc_Resolution;  // Converts Ticks/microSecs -> RPM

          output_speed = alpha_filter * output_speed + (1 - alpha_filter) * speed_measurement_Converted;

          // --- Define relative Time --- //
          if (last_time - tinit > 1000000) {  // 1 seconde
            PWM = Duty_Cycle;
          }

          Send_Data(last_time - tinit, PWM, output_speed, 0.0, 0.0);  // Not SetPoint & No Integral

          // SEND PWM TO THE MOTOR //
          Send_Motor(PWM);
        }
      }
      break;
      /* ------------------- */
      /*   END OF OPEN LOOP  */
      /* ------------------- */
    case 1:  // Therefore closed loop
      for (i = 1; i <= 7; i++) {
        C += (Serial.read() - 48) * pow(10, 6 - i);  // Recover the Speed setpoint's Value  divided by 10 // in tr/min
      }
      C -= 500;  // Offset that was added in Matlab
      for (i = 1; i <= 5; i++) {
        Gain += (Serial.read() - 48) * pow(10, 3 - i);  // Recover the Gain's Value divided By 100
      }
      for (i = 1; i <= 6; i++) {
        Gain_Integral += (Serial.read() - 48) * pow(10, 2 - i);  // Recover the Gain's Value divided By 10000
      }
      /* NOTE -> FRAME should be 19 bytes long (1 (loop) + 7 (setpoint) + 5 (prop Gain) + 6 (integral gain)) */

      TCNT2 = Size_TCNT2 - Wanted_Size_TCNT2;
      tinit = micros();  // Read time's value since the program's begining

      last_time = micros();

      while ((last_time - tinit) < SimulationTime) {  // while the simulation is not terminated
        now = micros();
        if (now - last_time >= Ts_us) {
          last_time += Ts_us;

          for (i = 1; i <= speed_comp_last; i++) {
            codes4Speed[i - 1] = codes4Speed[i];
            times4Speed[i - 1] = times4Speed[i];
          }
          codes4Speed[speed_comp_last] = code;       // code -> encoder value
          times4Speed[speed_comp_last] = last_time;  // last_time  -> this loop's time

          dt = times4Speed[speed_comp_last] - times4Speed[0];     // dt in micro sec (10^-6)
          dCode = codes4Speed[speed_comp_last] - codes4Speed[0];  // dcode in Ticks

          speed_measurement = dCode / dt;  // float(dCode) / float(dt); // speed (numerically) computed in Ticks/microsec
          speed_measurement_Converted = speed_measurement * 6 * pow(10, 7) / Enc_Resolution;

          output_speed = alpha_filter * output_speed + (1 - alpha_filter) * speed_measurement_Converted;

          // --- Define relative Time --- //
          time_rel = last_time - tinit;

          // CHANGE THE SETPOINT
          if (time_rel >= 1000000) {  // 1 seconde
            Consigne = C;
          }

          // ---------- //
          // ANTI WINDUP //
          // ---------- //
          // SIGN FUNCTION (a>=0)-(a<=0)
          Condition1 = fabsf(Control - Control_unSat) > 1e-5;  // Check is Control saturates(ed)

          sign1 = ((Control > 0) - (Control < 0)); // SIGN of the CONTROL
          sign2 = ((Consigne - output_speed) > 0) - ((Consigne - output_speed) < 0); // SIGN of the ERROR

          Condition2 = (sign1 == sign2); // CHECKS if ERROR & CONTROL have SAME SIGN

          Clamp = Condition1 && Condition2; 

          if (!Clamp) { Integral_Part = Integral_Part + (Consigne - output_speed) * Ts_us * pow(10, -3); }
          //             Previous_Time = last_time ;

          Control = Zero_Duty + Gain * (Consigne - output_speed) + Gain_Integral * Integral_Part;  // Calculate control to send to motor
          Control_unSat = Control;
          Control = constrain(Control, Zero_Duty - 255, Zero_Duty + 255);  // CONSTRAIN CONTROL WITHIN INTERVAL [-255, 255]
          // SEND Control TO MOTOR
          Send_Motor(Control);

          // SEND Data to Serial Port
          Send_Data(time_rel, Control, output_speed, Consigne, Gain_Integral * Integral_Part);  // Send : Speed'value, time's value, PWM's value in the serial port  + Set Point Value + weighted Integral
          // Send_Data(time_rel, Control, output_speed, Consigne, Integral_Part);  
        }
      }
      break;
      /* --------------------- */
      /*   END OF CLOSED LOOP  */
      /* --------------------- */

    case 2:                    // reset
      C = 0;                   // In the case of Speed Control
      code = pow(2, 20);       // Reinitiate Encoder's value to zero (centered in 2^20)
      dCode = 0;               // In the case of speed control
      Duty_Cycle = Zero_Duty;  // Reinitiate PWM received in serial port
      PWM = Zero_Duty;         // Reinitiate PWM to send to the motor
      Send_Motor(PWM);
      break;
      /* --------------------- */
      /*     END OF RESET      */
      /* --------------------- */
  }

  C = 0;            //pow(2,20); // Reinitiate position setpoint received in serial port to zero (centered in 2^20)
  PWM = Zero_Duty;  // Reinitiate PWM received in serial port
  Gain = 0;         // Reinitiate PGain's value
  Send_Motor(PWM);
  for (i = 0; i <= speed_comp_last; i++) {
    codes4Speed[i] = 0;  // Note : could be done with memset...
    times4Speed[i] = 0;
  }
}


//--------------------------------------------------//
//----------------- EXTRA FUNCTIONS-----------------//


//-----------------Send data in serial port-----------------//
void Send_Data(long t, long u, float y, float C, float Integral) {
  // SimulationTime :  durée de la simulation
  // t : temps
  // u : entree
  // y : vitesse mesurée
  // C :  SetPoint
  if (t < SimulationTime) {
    // Start of message
    Serial.print("<");
    // Print Time
    Serial.print(t);
    Serial.print("/");

    // Print PWM (Centered)
    Serial.print(u + 500 - Zero_Duty);
    Serial.print("/");

    // Print Speed (Scaled)
    Serial.print(500 + y * 100);
    Serial.print("/");

    // Print SetPoint (Scaled)
    Serial.print(500 + C * 100);
    Serial.print("/");

    // Print Integral (Centered)
    Serial.print(Integral + 5000 - Zero_Duty);

    // End of message
    Serial.println(">");
  }
}
// ------------------------------------------------------------ //
//------------------  MOTOR ACTUATION   ------------------------//
// Not A function, THIS IS DONE DIRECTLY IN THE MAIN CODE
void Send_Motor(long Control) {
  Control = constrain(Control, -255, 255);
  // SEND PWM TO MOTOR
  if (Control >= 0) {
    analogWrite(pwmMoteur2, 255);
    analogWrite(pwmMoteur1, 255 - Control);
  } else {
    analogWrite(pwmMoteur1, 255);
    analogWrite(pwmMoteur2, 255 + Control);
  }
}


// ------------------------------------------------------------ //
//---------------------ENCODER INTERRUPTS---------------------//
void Tic_A() {
  if (micros() - int0time < threshold)
    return;
  int0history = int0signal;
  int0signal = bitRead(PIND, pin_A);
  if (int0history == int0signal)
    return;
  int0time = micros();
  if (int0signal == int1signal)
    code++;
  else
    code--;
}

void Tic_B() {
  if (micros() - int1time < threshold)
    return;
  int1history = int1signal;
  int1signal = bitRead(PIND, pin_B);
  if (int1history == int1signal)
    return;
  int1time = micros();
  if (int0signal == int1signal)
    code--;
  else
    code++;
}

//-----------------------------------------------------------------//
//------------------------  FRAME FORMAT  ------------------------//
// This is the format that should be sent to the arduino ///
// OPEN LOOP FORMAT


// CLOSED LOOP FORMAT
/* 19 bytes long (1 (loop) + 7 (setpoint) + 5 (prop Gain) + 6 (integral gain)) */
