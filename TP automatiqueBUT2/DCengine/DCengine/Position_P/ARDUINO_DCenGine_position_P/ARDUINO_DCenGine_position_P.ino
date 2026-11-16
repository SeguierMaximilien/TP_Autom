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
//
volatile unsigned long threshold = 100; // 10000;

int i; // This is a generic counter
//  ------------------------------------------------ //
//  ------------------------------------------------ //

//  ------------------------------------------------ //
//    Working variables for the interrupt routines   //
//  ------------------------------------------------ //
volatile unsigned long int0time = 0;
volatile unsigned long int1time = 0;

volatile uint8_t int0signal = 0; // Channel A's value of encoder
volatile uint8_t int1signal = 0; // Channel B's value of encoder

volatile uint8_t int0history = 0;
volatile uint8_t int1history = 0;
// -----------

// Broches PWM utilisées (attention: ceci doit correspondre à la configuration des cavaliers)
#define pwmMoteur1  10
#define pwmMoteur2  9

int pin_A = 2; // Output A of encoder // Yellow Wire (is A according to datasheet)
int pin_B = 3; // Output B of encoder // White Wire (is B according to datasheet)

volatile long code = 1e6L; // Encoder's value (centered in 1e6);

//  ------------------------------------------------ //
//  ----------------- LOOP PARAMETERS -------------- //
//  ------------------------------------------------ //
int Boucle = 0; // Type of loop (0 : open loop / 1 : closed loop)

//----open loop----//
long Zero_Duty = 0;           // See3Sigma Datasheet
long Duty_Cycle = Zero_Duty; // PWM/Signal received in serial port
long PWM = Zero_Duty;        // PWM/signal to send to the motor

//---closed loop---//
long Consigne = 1e6; // Case of position Control
long C = 0; // Position setpoint received in serial port
int compteur = 0;
float multiplicateur = 1.0;

// --- Timer : FIXED UNIFORM SAMPLING --- //
const unsigned long Ts_us = 2000; // 10 ms → 100 Hz (2000 micro -> 2ms 500Hz)
const unsigned long SimulationTime = 3000000; // time of simulation (in microsecond);

unsigned long tinit = 0; // Time's value (in microsecond) // Comes from micros
unsigned long last_time = 0;
unsigned long now;
unsigned long time_rel;

// tinit -> time at which the open / closed loop starts
// Time -> elapsed time  since the beginning of the open / closed loop


// --- CONTROLLER PART --- //
long Control = 0; // PWM/signal to send to the motor
float Gain;      // Gain's value of the closed loop


// ----------------------------------------------------------------- //
//-----------------------FUNCTION PROTOTYPES------------------------//
// ----------------------------------------------------------------- //
void Send_Data(long t, long u, long y, long C); // Function to send data in serial port
void Send_Motor(long Control);

// ----------------------------------------------------------------- //
//----------------------------Void Setup----------------------------//
// ----------------------------------------------------------------- //
void setup() {
 // --- cf. 3 Sigma Datasheet --- //
 // Définition des broches PWM en sortie
  pinMode(pwmMoteur1, OUTPUT);
  pinMode(pwmMoteur2, OUTPUT);

  // --- Encoder pins and variables Setup --- //
  pinMode(pin_A, INPUT); // To configure input
  pinMode(pin_B, INPUT);

  digitalWrite(pin_A, HIGH); //Enables Internal PullUP
  digitalWrite(pin_B, HIGH); //Enables Internal PullUP
  // https://www.arduino.cc/reference/en/language/functions/digital-io/digitalwrite/

  // --- Encoder Interrupts Setup --- //
  attachInterrupt(INT0, Tic_A, CHANGE); // Encoder Tic on channel A // INT0 is the pin 2 interrupt
  attachInterrupt(INT1, Tic_B, CHANGE); // Encoder Tic on channel B // INT0 is the pin 3 interrupt

  // --- Serial Com setup --- //
  Serial.begin(115200); // Define the baudrate (byte per second)
  while ( Serial.available () > 0) {
    Serial.read ();
  } ; // Clear the serial port
}

// ----------------------------------------------------------------- //
//----------------------------Void Loop-----------------------------//
// ----------------------------------------------------------------- //
void loop() {

  while (Serial.available () < 12); // While there are less than 12 bytes in serial port --> do nothing. Arduino is a slave

  Boucle = (Serial.read() - 48); // Read byte's value in ASCII and convert it
  switch (Boucle) { // Begin of switch case

    // ============== //
    //    OPEN LOOP   // 
    // ============== //
    case 0: // Therefore open loop
      for (i = 1; i <= 3; i++) {
        Duty_Cycle += (Serial.read() - 48) * pow(10, 3 - i); // Recover the PWM's Value centered in 500
      }
      Duty_Cycle -= 500; // Center PWM's value in 0
      Duty_Cycle = constrain(Duty_Cycle, Zero_Duty-255, Zero_Duty+ 255); // CONSTRAIN CONTROL WITHIN INTERVAL [-400, 400]
      PWM = Zero_Duty; // Reinitialize PWM's value to send to the motor

      tinit = micros(); // Read time's value since the program's begining
      last_time = tinit;
      
      
      while ((last_time - tinit)<SimulationTime) { // while the simulation is not terminated
        now = micros();
        if (now - last_time >= Ts_us){
            last_time += Ts_us;
             
            // --- Define relative Time --- //
            if (last_time - tinit> 1000000) {  // 1 seconde         
              PWM = Duty_Cycle;
            }  
        // SEND PWM TO THE MOTOR //
        Send_Motor(PWM);
        Send_Data(last_time - tinit,PWM,code,Consigne); // Send :  time's value, PWM's value, encoder'value in the serial port
        }}
      break;
    // ====================== //
    //   \END OPEN LOOP \END  // 
    // ====================== //


    // ================ //
    //    CLOSED LOOP   // 
    // ================ //
    case 1: // Therefore closed loop
    compteur = 0;
    multiplicateur = 1.0;

      for (i = 1; i <= 7; i++) {
        C += (Serial.read() - 48) * pow(10, 7 - i); // Recover the Position setpoint's Value
        // C stands for "consigne"
      }
      C = constrain(C, 0, 2e6); 
     // C -= pow(10, 6); // Centers the SetPoint around 0.
      for (i = 1; i <= 5; i++) {
        Gain += (Serial.read() - 48) * pow(10, 2 - i); // Recover the Gain's Value divided By 1000
      }
      tinit = micros();
      last_time = micros();

      while ((last_time - tinit)<SimulationTime){ // while the simulation is not terminated
        now = micros();

      if (now - last_time >= Ts_us){
            last_time += Ts_us;


            // CHANGE THE SETPOINT
            if ((last_time - tinit) >= 1000000){ //+compteur*500000) {  // 1 seconde
               Consigne = C; //1e6 +  (long)((float)(C-1e6) * (1.0f + (0.5f * (float)multiplicateur)));
                // multiplicateur = multiplicateur*-1.0; 
                // compteur+=1;
                }
            Control = Zero_Duty + Gain * (Consigne - code); // Calculate control to send to motor
            Control = constrain(Control, Zero_Duty - 255, Zero_Duty + 255); // CONSTRAIN CONTROL WITHIN INTERVAL [-255, 255]
            Send_Motor(Control);
            Send_Data(last_time - tinit,Control,code,Consigne); // Send : encoder'value, time's value, control's value in the serial port
      }}
      break;
    // ========================== //
    //    \END CLOSED LOOP \END   // 
    // ========================== //

    case 2: // reset
      Consigne = 1e6; // Reinitiate position setpoint to zero (centered in 2^20)
      code = 1e6; // Reinitiate Encoder's value to zero (centered in 2^20)
      // B = digitalRead(pin_B); // Read value of channel B
      // A = digitalRead(pin_A); // Read value of channel A
      Duty_Cycle = Zero_Duty; // Reinitiate PWM received in serial port
      PWM  = Zero_Duty; // Reinitiate PWM to send to the motor
      // SyRen.writeMicroseconds(PWM); // Send PWM to the motor
      Send_Motor(PWM);
      break;
      /* --------------------- */
      /*     END OF RESET      */
      /* --------------------- */
  }

  C = 0; // Reinitiate position setpoint received in serial port to zero (Not Centered => too large instead)

  Consigne = 1e6;
  code = 1e6;
  PWM = Zero_Duty; // Reinitiate PWM received in serial port
  Gain = 0; // Reinitiate PGain's value
  Send_Motor(PWM);
//  }
}

//--------------------------------------------------//
//----------------- EXTRA FUNCTIONS-----------------//


//-----------------Send data in serial port-----------------//
void Send_Data(long t, long u, long y, long C) {
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
    y = constrain(y, 0, 2e6); // Saturate Encoder's Value between 0 and 2e6 - just in case of. 
    Serial.print(y);
    Serial.print("/");

    // Print SetPoint
    Serial.print(C);
    
    // End of message
    Serial.print(">"); 
  }
}
// ------------------------------------------------------------ //
//------------------  MOTOR ACTUATION   ------------------------//
void Send_Motor(long Control){
    Control = constrain(Control,-255,255); 
// SEND PWM TO MOTOR
            if (Control >= 0){
              analogWrite(pwmMoteur2, 255);
              analogWrite(pwmMoteur1, 255 - Control);} 
            else{analogWrite(pwmMoteur1, 255);
                 analogWrite(pwmMoteur2, 255+Control);}
}

//---------------------Encoder readings---------------------//
void Tic_A() {
  if ( micros() - int0time < threshold )
    return;
  int0history = int0signal;
  int0signal = bitRead(PIND, pin_A);
  if ( int0history == int0signal )
    return;
  int0time = micros();
  if ( int0signal == int1signal )
    code++;
  else
    code--;
}

void Tic_B() {
  if ( micros() - int1time < threshold )
    return;
  int1history = int1signal;
  int1signal = bitRead(PIND, pin_B);
  if ( int1history == int1signal )
    return;
  int1time = micros();
  if ( int0signal == int1signal )
    code--;
  else
    code++;
}

