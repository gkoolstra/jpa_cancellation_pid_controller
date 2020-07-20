// ********************************************************
// * PID Basic Example
// * Reading analog input 0 to control analog PWM output 3
// ********************************************************/
#include <PID_v1.h>
#include <LiquidCrystal.h>
// Note: Before uploading the code, press the erase button on the Arduino Due.

//Define Variables we'll be connecting to
double Setpoint, Input, Output, Kp, Ki, Kd;

//Specify the links and initial tuning parameters
//Syntax: PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, Direction)
//Note: Keeping Ki at 0 results in "proportional droop" for finite gain systems
//Keep Kd = 0 because it can result in unstable behavior. Kp and Ki can be tuned in-situ.
PID myPID(&Input, &Output, &Setpoint, 2, 5, 0, DIRECT);

int time_delay = 25; // Time delay of the control loop in milliseconds
int screen_refresh_delay = 200; // Must be a multiple of the time_delay
int screen_update_frequency = int(screen_refresh_delay / time_delay);
unsigned long time_now = 0;
int PID_DAC = DAC1; // PID output will be applied to this DAC
int CTS_DAC = DAC0; // Second DAC on the arduino; will have constant output
int CTS_DAC_value = 0; // Voltage of the second dac
int PID_input_pin = 0; // Analog input pin number; will be used as input #0.
float DACvolts_max = 3.3; // Range of the DAC outputs
int DACrange_max = 4095; // Maximum number of bits used for the DAC (full range runs from 0 - 2^n_bits - 1)
char display_mode = 'V'; // 'V' or other. If 'V' display values are in volts, else units are in bits (0 to 4095)
int input_samples = 20; // number of averages for measuring the Input variable

// For serial communication
char action_char = 'A'; // for incoming serial data
char quant_char[] = "0000"; // for incoming serial data
bool use_serial_plotter = false; // plot diagnostics on the serial plotter screen. Should be turned off for communication w/ python.

// Voltage ramp parameters
int i = 0; // Counter for the voltage ramp.
bool currently_ramping = true; // If true the Arduino boots with ramp on the output channel
bool currently_manual_mode = false; // If true the Arduino boots with setting DAC0 to bit value 0. By monitoring AI0 you can minimize the offset.
float ramp_period = 4.0; // period in seconds
int ramp_min = 0; // minimum value of the ramp (you can constrain this to sweep over a smaller range)
int ramp_max = DACrange_max; // maximum value of the ramp (you can constrain this to sweep over a smaller range)

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(13, 12, 11, 10, 9, 8);

void setup()
{
  // Set the resolution of the Analog inputs to 12 bits (maximum for Arduino Due)
  analogReadResolution(12);
  // Set the resolution of the DAC outputs to 12 bits (maximum for Arduino Due)
  analogWriteResolution(12);
  // Set the analog reference to the default for arduino Due (3.3 V)
  analogReference(AR_DEFAULT);
  
  // Initialize the variables we're linked to
  Input = integrate_input(input_samples); //analogRead(PID_input_pin);
  Setpoint = 0;
  Kp = 0.01;
  Ki = 2.5;
  Kd = 0;
  
  lcd.begin(16, 2); // set up the LCD's number of columns and rows: 
  lcd.print("JPA Cancelation");

  Serial.begin(9600);
  // Wait for serial port to connect. Needed for native USB
  while (!Serial) {};

  //turn the PID on
  myPID.SetOutputLimits(0, DACrange_max);
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(time_delay);
}

void get_display_mode()
{
  Serial.println(display_mode);
}

void set_display_mode(char mode)
{
  display_mode = mode;
}

void set_CTS_DAC_value(int value)
{
  CTS_DAC_value = value;
}

void get_CTS_DAC_value()
{
  Serial.println(CTS_DAC_value);
}

void set_setpoint(int setpoint)
// Sets the setpoint of the PID loop. The PID will force the input to this value.
{
  Setpoint = int(setpoint); 
}

void get_setpoint(void)
{
  Serial.println(Setpoint);
}

void get_input(void)
{
  Serial.println(Input);
}

void get_output(void)
{
  Serial.println(Output);
}

void set_Kp(float P)
// Sets the proportional constant in the control block. Output = Kp * (Input - Setpoint)
{
  Kp = P;
  myPID.SetTunings(P, Ki, 0.0);
}

void get_Kp(void)
{
  Serial.println(myPID.GetKp());
}

void set_Ki(float I)
// Sets the integral control in the control block. Output = Ki * integral(Input - Setpoint)
{
  Ki = I;
  myPID.SetTunings(Kp, I, 0.0); 
}

void get_Ki(void)
{
  Serial.println(myPID.GetKi());
}

void set_Kd(float D)
{
  Kd = D;
  myPID.SetTunings(Kp, Ki, D); 
}

void get_Kd(void)
{
  Serial.println(myPID.GetKd());
}

void get_ramping()
{
  Serial.println(currently_ramping);
}

void get_manual_mode()
{
  Serial.println(currently_manual_mode);
}

int ramp_function(int ramp_min, int ramp_max, int index, float period_s)
{
  int period_pts = int(period_s / (0.001 * time_delay));
  int mod_i = index % period_pts;
  int ramp_out;
  if (mod_i < (0.5 * period_pts)) {
    ramp_out = ramp_min +  (ramp_max - ramp_min) * mod_i / (0.5 * period_pts);
  }
  else {
    ramp_out = ramp_max - (ramp_max - ramp_min) * (mod_i - 0.5 * period_pts) / (0.5 * period_pts);
  }
  return ramp_out;
}


void write_to_lcd(float input, float setpoint) 
{
  lcd.clear();
  lcd.setCursor(0,0); // Move the cursor to the top left
  if (currently_ramping)
  {
    lcd.print("Open loop ramp");
  }
  else if (currently_manual_mode)
  {
    lcd.print("Manual mode:");
  }
  else
  {
    lcd.print("Setpt: ");
    if (display_mode == 'V')
      {lcd.print(bit_to_volts(setpoint), 3);
      lcd.print('V');}
    else
      {lcd.print(int(setpoint));}
  }
  
  lcd.setCursor(0,1); // Move the cursor to the bottom left
  lcd.print("Input: ");
  if (display_mode == 'V')
    {lcd.print(bit_to_volts(input), 3);
     lcd.print('V');}
  else
    {lcd.print(int(input));}
}





void listen_to_serial()
// Reads serial commands sent by the computer and translates the commands into actions of the controller.
{
  if (Serial.available() > 0) 
  {
    int loop_counter = 0;
    lcd.setCursor(11,0); // Expect maximum received length = 5 (15,0) is upper right corner.
    while (Serial.available() > 0)
    {
      if (loop_counter == 0)
      {
        action_char = Serial.read(); // read the incoming character
      }
      else if (loop_counter > 4)
      {
        Serial.read(); // Empty the buffer, but neglect characters in a string that is longer than 4 characters
      }
      else
      {
        quant_char[loop_counter - 1] = Serial.read(); // read the incoming character
      }
      loop_counter += 1;
    }
    lcd.print(action_char);
    lcd.print(quant_char);
    translate_serial_input(action_char, quant_char);
  }
}


void translate_serial_input(char action, char quantity[])
{ // Allowed commands [R, G{____}, S{____}, P{____}, I{____}, M{____}]
  lcd.setCursor(0,0); // Cursor on the upper left
  if (action == 'S') // Changes the setpoint
  {
    long quantity_int = atol(quantity); // This must always be an integer (bit value)
    set_setpoint(constrain(quantity_int, 0, DACrange_max)); 
  }
  else if (action == 'R') // Toggles the ramping (does not require additional input)
  {
    currently_ramping = !currently_ramping;
  }
  else if (action == 'G')
  {
    use_serial_plotter = false;
    long quantity_int = atol(quantity);
    if (quantity_int == 0)
    {
      get_setpoint();
    }
    else if (quantity_int == 1)
    {
      get_input();
    }
    else if (quantity_int == 2)
    {
      get_output();
    }
    else if (quantity_int == 3)
    {
      get_Kp();
    }
    else if (quantity_int == 4)
    {
      get_Ki();
    }
    else if (quantity_int == 5)
    {
      get_Kd();
    }
    else if (quantity_int == 6)
    {
      get_display_mode();
    }
    else if (quantity_int == 7)
    {
      get_CTS_DAC_value();
    }
    else if (quantity_int == 8)
    {
      get_ramping();
    }
    else if (quantity_int == 9)
    {
      get_manual_mode();
    }
  }
  else if (action == 'P') // Sets the proportional control
  {
    float quantity_flt = atof(quantity);
    set_Kp(constrain(quantity_flt, 0, 1E15));
    lcd.clear();
    lcd.print("P = ");
    lcd.print(myPID.GetKp(), 2);
    delay(1000); // Display the newly set value of Kp on the screen for confirmation
  }
  else if (action == 'I') // Sets the integral control constant
  {
    float quantity_flt = atof(quantity);
    set_Ki(constrain(quantity_flt, 0, 1E15));
    lcd.clear();
    lcd.print("I = ");
    lcd.print(myPID.GetKi(), 2);
    delay(1000); // Display the newly set value of Ki on the screen for confirmation
  }
  else if (action == 'D') // Sets the integral control constant
  {
    float quantity_flt = atof(quantity);
    set_Kd(constrain(quantity_flt, 0, 1E15));
    lcd.clear();
    lcd.print("D = ");
    lcd.print(myPID.GetKd(), 2);
    delay(1000); // Display the newly set value of Ki on the screen for confirmation
  }
  else if (action == 'M') // Sets a manual voltage on the dac
  {
    currently_ramping = false; // turn off the ramping
    currently_manual_mode = true; // we are manually tuning the dac
    long quantity_int = atol(quantity);
    if (quantity_int > DACrange_max) // escape from manual tuning
    {
      currently_manual_mode = !currently_manual_mode; // toggle manual tuning of the dac output.
    }
    else
    {
      analogWrite(PID_DAC, constrain(quantity_int, 0, DACrange_max));
    }
    
  }
  else
  {
    lcd.print("?");
  }
}


float bit_to_volts(int value)
{
  if (display_mode == 'V')
  { 
    return value * DACvolts_max / float(DACrange_max); // map the bit number from 0 to 4095 to 0 to 3.3 V (DACvolts_max)
  }
  else
  {
    return value;
  }
}

int volts_to_bit(float value)
{
  return int(value / float(DACvolts_max) * DACrange_max); // map the bit number from 0 to 4095 to 0 to 3.3 V (DACvolts_max)
}

double integrate_input(int num_samples)
{
  int integrated_output = 0; // In theory the sampling rate should be 9.6 kHz so each measurement roughly takes 100 us.
  for (int s = 1; s <= num_samples; s++)
  {
     integrated_output += analogRead(PID_input_pin);
  }
  return float(integrated_output / num_samples);
}


void loop()
{ 
  time_now = millis();
  listen_to_serial(); // Check if something comes in on the serial port:
  Input = integrate_input(input_samples); // analogRead(PID_input_pin); // read the value from the sensor
  int ramp_value = ramp_function(ramp_min, ramp_max, i, ramp_period);
  
  if (currently_ramping) // Open loop operation
  {
    currently_manual_mode = false;
    // analog write values on DAC0 and DAC1 range from 0 to 4095 (DACrange_max), on PWM from 0 to 255.
    analogWrite(PID_DAC, constrain(ramp_value, 0, DACrange_max));

    if (use_serial_plotter)
    {
      Serial.println(ramp_value); // Blue
      Serial.print(",");
      Serial.println(Input); // Red
    }
    
  }
  else if (currently_manual_mode) // Open loop operation
  { // Toggle tuning of DAC0 output offset. This sets the DAC1 to "target" and monitors the output at Analog In 0
    currently_ramping = false;
    
    if (use_serial_plotter)
    {
      Serial.println(0.0); 
      Serial.print(",");
      Serial.println(Input); // Red, AI0 value (tune the potentiometer to match the target)
    }
  }
  else // Closed loop operation
  {
    myPID.Compute(); // Compute the Output of the PID controller based on the setpoint and input
    analogWrite(PID_DAC, constrain(Output, 0, DACrange_max)); // Make sure output is between 0 and 4095 and apply the calculated output to DAC0
    analogWrite(CTS_DAC, constrain(CTS_DAC_value, 0, DACrange_max));
    
    if (use_serial_plotter)
    {
      Serial.println(Input); // Blue
      Serial.print(",");
      Serial.println(Output); // Red
    }
  }
  
  // Write the value of Input to the LCD screen
  if (i%screen_update_frequency == 0) 
  {
    write_to_lcd(Input, Setpoint);
  }
  
  //delay(time_delay);
  while(millis() < time_now + time_delay)
    {
    //wait approx. [period] ms
    }
  i += 1;
}
