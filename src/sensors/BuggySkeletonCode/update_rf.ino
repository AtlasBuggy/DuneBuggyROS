#define CH1_PIN A2
#define CH2_PIN A3
#define CH3_PIN A4
#define CH4_PIN A5
#define CH5_PIN 6
#define CH6_PIN 7

int rf_vals[] = {0, 0, 0, 0, 0, 0};
int rf_errs[] = {0, 0, 0, 0, 0, 0};
int rf_pins[] = {CH1_PIN, CH2_PIN, CH3_PIN, CH4_PIN, CH5_PIN, CH6_PIN};
float rf_k = .3;
 
void init_rf() {
  // put your setup code here, to run once:
  //Serial.begin(115200);
  pinMode(CH6_PIN, INPUT);
  pinMode(CH5_PIN, INPUT);
  pinMode(CH1_PIN, INPUT); 
  pinMode(CH2_PIN, INPUT); 
  pinMode(CH3_PIN, INPUT); 
  pinMode(CH4_PIN, INPUT);
}

void update_rf() {
  // put your main code here, to run repeatedly:
  for(int i=0; i<=5; i++){
    int new_val = pulseIn(rf_pins[i], HIGH, 4);
    rf_errs[i]    = new_val - rf_vals[i];
    rf_vals[i]    = rf_vals[i] + rf_k*rf_errs[i];
  }
//  delay(1);
}

void write_rf_vals() {
  for(int i=0; i<=5; i++){
    Serial.print(rf_vals[i]);
    Serial.print('\t');
  }
//  Serial.print('\n');
}
