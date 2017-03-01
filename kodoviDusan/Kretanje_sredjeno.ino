//#include <TimerOne.h>

//Definicija pinova potrebnih za upravljanje motorom 1 - levi motor

//pinovi za zadavanje smera preko drajvera motora - H mosta
//AB=b00 iskljucivanje napajanja na motorima, ukoliko su se motori kretali zaustavice se usled trenja 
//AB=b10 "napred" smer 
//AB=b01 "nazad" smer
//AB=11 kocenje napajanje na krajevima motora ce biti tako spojeno da se motori aktivno koce

#define InA1            12                      // INA motor pin
#define InB1            13                      // INB motor pin 
#define PWM1            11                      // PWM motor pin, pin sa kojeg se zadaje vrednost popunjenosti signala za kontrolu motora

// enkoderski signali, program broji usponske ivice signala A dok signal B koristi za odredjivanje smera rotacije
#define encodPinA1      2                       // encoder A pin
#define encodPinB1      5                       // encoder B pin

//Definicijia potrebnih pinova za motor 2 - desni motor

#define InA2            7                       // INA motor pin
#define InB2            9                       // INB motor pin 
#define PWM2            10                      // PWM motor pin
#define encodPinA2      3                       // encoder A pin
#define encodPinB2      6                       // encoder B pin


#define LOOPTIME_POS         15                 // period pozivanja (u milisekundama) PD regulatora POZICIJE 
#define LOOPTIME_SPEED       5                  //period pozivanja (u milisekundama) PI regulatora BRZINE


//Parametri robota
float W2W_distance = 285;                        //rastojanje izmedju centara tockova (u milimetrima)
float EncoderPPR = 16;                           // broj impuplsa enkodera po jednom obrtaju
float GearRatio= 50;                             // ondos redukcije reduktora
float WheelRadius = 90;                          // poluprecnik tocka (u mm)


//Varijable regulacije brzine

boolean run = false;                             // Dozvola kretanja
boolean run1=false;
boolean run2=false;

float Ref_speed1 = 0;                            // Referenca translatorne brzine centra levog (motor 1) tocka (u m/s)
float Speed1 = 0;                                // Brzina 1 - Trenutna translatorna brzina centra levog tocka (motor 1) u (m/s)
float Ref_speed2 = 0;                            // Referenca translatorne brzine centra desnog (motor 2) tocka (u m/s)
float Speed2 = 0;                                // Brzina 2 - Trenutna translatorna brzine centra desnog (motor 2) tocka (u m/s)
float e_speed1=0;                                // Greska pracenja brzine 1 (u m/s)
float e_speed2=0;                                // Greska pracenja brzine 2 (u m/s)

float sum_e_speed1 = 0;                          //integral greske brzine 1
float sum_e_speed2 = 0;                          //integral greske brzine 2

float U_speed1 = 0;                              // Upravljacki signal motora 1. Vrednost 255 odgovara duty cicle-u PWMa prvog motora od 100% , negativni iznos označava suprotan smer
float U_speed2 = 0;                              // Upravljacki signal motora 2. Vrednost 255 odgovara duty cicle-u PWMa prvog motora od 100% , negativni iznos označava suprotan smer

float Max_speed = 0.7;                             // Maksimalna brzina 

int PWM_val1 = 0;                                // PWM signal za komandu prvom motoru (25% = 64; 50% = 127; 75% = 191; 100% = 255)
int PWM_val2 = 0;                                // PWM signal za komandu drugom motoru(25% = 64; 50% = 127; 75% = 191; 100% = 255)

//Parametri PI regulatora brzine
float Kp1_s = 0.9;                                // Proporcionalno pojacanje PI regulatora brzine prvog motora
float Ki1_s = 0.133;                              // Integralno pojacanje PI regulatora brzine prvog motora

float Kp2_s = 1.03;                              //Proporcionalno pojacanje PI regulatora brzine drugog motora
float Ki2_s = 0.133;                             // Integralno pojacanje PI regulatora brzine drugog motora


//Varijable regulacije pozicije


int Ref_pos1 = 0;                                // Referenca pozicije levog tocka (motor 1). Pozicija je zadata kao predjeni put (u mm) koji je potrebno da centar tocka predje
float U_pos1 = 0;                                // Upravljacki signal pozicionog PD regulatora 1
float e_pos1=0;                                  // Greska pracenja pozicije levog tocka (motor 1) (u mm)
float e_pos1_old = 0;                            // Stara greska pracenja pozicije 1 (iz prethodne iteracije)
volatile long Enc1count = 0;                     // Brojac impulsa levog (motor 1) enkodera 
long Enc1count_old = 0;                          // Stara vrednost brojaca impulsa 1 (iz prethodne iteracije)
float Position1=0;                               // Pozicija levog tocka (motor 1) - predjeni put centra tocka od pocetka programa (u mm)

int Ref_pos2 = 0;                                // Referenca pozicije desnog tocka (motor 2). Pozicija je zadata kao predjeni put (u mm) koji je potrebno da centar tocka predje 
float U_pos2 = 0;                                // Upravljacki signal pozicionog PD regulatora 2
float e_pos2=0;                                  // Greska pracenja pozicije desnog tocka (motor 2) (u mm)
float e_pos2_old = 0;                            // Stara greska pracenja pozicije 2 (iz prethodne iteracije)
volatile long Enc2count = 0;                     // Brojac impulsa desnog (motor 2) enkodera 
long Enc2count_old = 0;                          // Stara vrednost brojaca impulsa 2 (iz prethodne iteracije)
float Position2=0;                               // Pozicija levog tocka (motor 2) - predjeni put centra tocka od pocetka programa (u mm)

//Parametri PD regulatora pozicija

float Kp1 = 0.02;                                // Proporcionalno pojacanje regluatora pozicije levog tocka (motor 1)
float Kd1 = 0.05;                                // Diferencijalno pojacanje regluatora pozicije levog tocka (motor 1)
int diff1 = 0;                                   // Indikacija da li je u upotrebi diferencijalno dejstvo PD kontrolera 1 (1-jeste 0-nije )          


float Kp2 = 0.02;                                // Proporcionalno pojacanje regluatora pozicije desnog tocka (motor 2)
float Kd2 = 0.05;                                // Diferencijalno pojacanje regluatora pozicije desnog tocka (motor 2)
int diff2 = 0;                                   // Indikacija da li je u upotrebi diferencijalno dejstvo PD kontrolera 2 (1-jeste 0-nije ) 

// Vremenski parametri

boolean kraj= false;                              //Indikacija da li je potrebno zaustaviti program
long CurrentTime= 0;                              //Vreme od pocetka rada programa u milisekundama
long StartTime= 0;                                //Pocetno vreme (u milisekundama)
int counter =0;                                   // brojac, proverava da li je potrebno izvrsiti pozicioni PD 

void setup() {

  Serial.begin(9600);
  
  //pinovi za levi tocak (motor 1)
  
  pinMode(InA1, OUTPUT);                           //konfigurisanje pinova za kontrolu motora 1 kao izlazni
  pinMode(InB1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(encodPinA1, INPUT);                       // konfigurisanje pinova za enkoderske impulse kao ulazni                    
  pinMode(encodPinB1, INPUT); 
  digitalWrite(encodPinA1, HIGH);                   // ukljucivanje pull up otpornika na enkoderskim ulazima
  digitalWrite(encodPinB1, HIGH); 
   
  //pinovi za desni tocak (motor 2)
  
  pinMode(InA2, OUTPUT);                            //konfigurisanje pinova za kontrolu motora 1 kao izlazni
  pinMode(InB2, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(encodPinA2, INPUT);                       // konfigurisanje pinova za enkoderske impulse kao ulazni 
  pinMode(encodPinB2, INPUT); 
  digitalWrite(encodPinA2, HIGH);                      // ukljucivanje pull up otpornika na enkoderskim ulazima
  digitalWrite(encodPinB2, HIGH);
  
  

  attachInterrupt(0, Enc1_interrupt, RISING);        // interrupt 0 je vezan za pin 2, poziva se ISR rutina na usponsku ivicu signala na pinu 2 - enkoderski A kanal prvog enkodera
  attachInterrupt(1, Enc2_interrupt, RISING);        // interrupt 1 je vezan za pin 3, poziva se ISR rutina na usponsku ivicu signala na pinu 3 - enkoderski A kanal drugog enkodera
}//kraj setup-a


void loop() {

            StartTime=millis();                     //ocitavanje pocetnog vremena
            translatornoKretanje(250);       //kretanje napred 250mm
            delay(1000);
            Serial.println("Pauza");
            rotirajLevo(90);
            Serial.println("Pauza");
            delay(1000);
            rotirajDesno(90);
            while(1) {}
            }//kraj loop-a
            
            

// Brzinski regulator - PI            
void Speed_regulator(){
  
  if (run){                                                                 //ukoliko je dozvoljeno kretanje
  
  
    
      
    Speed1 = ((Enc1count - Enc1count_old)*90*3.14)/16/50/LOOPTIME_SPEED;    // estimacija brzine prvog tocka
    Speed2 = ((Enc2count - Enc2count_old)*90*3.14)/16/50/LOOPTIME_SPEED;    //estimacija bzine drugog tocka
    
    Enc1count_old=Enc1count;                                                //Pamcenje starih vrednosti enkoderskih brojaca
    Enc2count_old=Enc2count;  
    
    e_speed1 = Ref_speed1 - Speed1;                                        //Greska pracenja brzine
    e_speed2 = Ref_speed2 - Speed2;

    U_speed1= Kp1_s * e_speed1 + Ki1_s*(e_speed1+sum_e_speed1);              //PI zakon upravljanja
    U_speed2= Kp2_s * e_speed2 + Ki2_s*(e_speed2+sum_e_speed2);            
    
    if (U_speed1>255) U_speed1=255;                                          // zasicenje upravljackog signala i antiwindup. ukoliko je upravljanje u zasicenju
    else if (U_speed1<-255) U_speed1=-255;                                   // greska se ne sabira u integratoru
    else sum_e_speed1 += e_speed1;
    
    if(U_speed1>0) motorForward1(); else motorBackward1();                   // Na osnovu znaka upravljanja se odredjuje smer pobude motora
    PWM_val1 = constrain( abs(int(U_speed1)), 0, 255);                       // Na PWM izlaz se salje apsolutna vrednost upravljanja
        
    if (U_speed2>255) U_speed2=255;                                          // zasicenje upravljackog signala i antiwindup. ukoliko je upravljanje u zasicenju
    else if (U_speed2<-255) U_speed2=-255;                                   // greska se ne sabira u integratoru
    else sum_e_speed2 += e_speed2;
    
    if(U_speed2>0) motorForward2(); else motorBackward2();                  // Na osnovu znaka upravljanja se odredjuje smer pobude motora
    PWM_val2 = constrain( abs(int(U_speed2)), 0, 255);                      // Na PWM izlaz se salje apsolutna vrednost upravljanja 
    
    analogWrite(PWM1, abs(int(U_speed1)));                                   // Upis PWM signala na pinove
    analogWrite(PWM2, PWM_val2); 
    
    counter++;                                                               // inkrement brojaca perioda
    if (counter==3){                                                         // Na svaku trecu periodu se poziva Regulator pozicije
      Position_regulator();
      counter=0;
    }
  }
}

// Pozicioni kvazi PD regulator
void Position_regulator()   {            

    if (run){
  
    Position1 = Enc1count*90*3.14/16/50;      //estimacija pozicije: put koji je centar tocka presao se racuna na osnovu impulsa enkodera. za svaki umpuls tocak 
    Position2 = Enc2count*90*3.14/16/50;      //obrne 1/(rezolucija enkodera(16)* odnos redukcije(50) ) obrtaja. Za jedan obrtaj centar tocka predje pi*precnik tocka(90mm)
                                              // Iz ovoga sledi da svaki impuls predstavlja pomeraj od (precnik*pi)/(rezolucija *odnos redukcije) milimetara

    
  
    
    
    e_pos1 = Ref_pos1 - Position1;             //Greska pracenja pozicije
    e_pos2 = Ref_pos2 - Position2; 
    
//    Serial.print("Greska: ");
//    Serial.print(e_pos1);
//    Serial.print("  ");
//    Serial.println(e_pos2);
    
   
    if (abs(e_pos1)>200) diff1=1; else diff1=0;  //Ukoliko je greska velika ukljucuje se diferencijalno dejstvo da bi kocilo motor
    if ((abs(e_pos1) < 5)) {motor1Brake();run1=false;} //Ukoliko je greska pracenja mala smatra se da je motor stigao u zadatu referencu
    else{
        U_pos1 = constrain(Kp1 * e_pos1 + diff1 * Kd1*(e_pos1-e_pos1_old),-0.7,0.7);   // Zakon upravljanje PD regulatora. Upravljanje se odmah ogranicava na maksimalnu i minimalnu dozvoljenu brzinu    
        e_pos1_old=e_pos1;                       //pamti se trenutna greska kao stara  
        }
    
    Ref_speed1 = U_pos1;                          //Upravljanje pozicionog regulatora je referenca za brzinski regulator
    
    if (abs(e_pos2)>200) diff2 = 1;              //Ukoliko je greska velika ukljucuje se diferencijalno dejstvo da bi kocilo motor
    if ((abs(e_pos2) < 5)) {motor2Brake();run2=false;}   //Ukoliko je greska pracenja mala smatra se da je motor stigao u zadatu referencu i daje mu se komanda za kocenje
    else{
          U_pos2 = constrain(Kp2 * e_pos2 + diff2 * Kd2*(e_pos2-e_pos2_old),-0.7,0.7);  // Zakon upravljanje PD regulatora. Upravljanje se odmah ogranicava na maksimalnu i minimalnu dozvoljenu brzinu    
          e_pos2_old=e_pos2;                     //pamti se trenutna greska kao stara                                            
         }
      
    Ref_speed2 = U_pos2;                        //Upravljanje pozicionog regulatora je referenca za brzinski regulator
    
    run=run1||run2;                             //dozvola za kretanje je ukljucena ukoliko makar jedan od motora i dalje treba da se krece
    Serial.print("Pozicija: ");
    Serial.print(Position1);
    Serial.print("  ");
    Serial.print(Position2);
    Serial.print("  ");
    Serial.print(U_pos1);
    Serial.print("  ");
    Serial.print(U_pos2);
    Serial.print("  ");
    Serial.println(run);
  }
}

//Enkoder 1
void Enc1_interrupt()  {                           // prekidna rutina za enkoder 1, usponska ivica
  if(digitalRead(encodPinB1)==LOW)   Enc1count --; // ukoliko je pri pojavi usponske ivice signala A signal B na niskom nivou smer obrtanja je negativan
  if (digitalRead(encodPinB1)==HIGH) Enc1count ++; // ukoliko je pri pojavi usponske ivice signala A signal B na visokom nivou smer obrtanja je pozitivan  

}

void Enc2_interrupt()  {                           // prekidna rutina za enkoder 2, usponska ivica, enkoder je namesten obrnuto
  if(digitalRead(encodPinB2)==LOW)   Enc2count ++;// ukoliko je pri pojavi usponske ivice signala A signal B na niskom nivou smer obrtanja je pozitivan                
  if (digitalRead(encodPinB2)==HIGH) Enc2count --;// ukoliko je pri pojavi usponske ivice signala A signal B na visokom nivou smer obrtanja je negativan

}

// Funkcije za upravljanje smerom  

void motorForward1()  { 
  
   digitalWrite(InA1, LOW);
   digitalWrite(InB1, HIGH);
   
}
 
void motorBackward1()  {
 
  digitalWrite(InA1, HIGH);
  digitalWrite(InB1, LOW);
}

void motorForward2()  { 
    
   digitalWrite(InA2, LOW);
   digitalWrite(InB2, HIGH);

 }
 
void motorBackward2()  {
  
  digitalWrite(InA2, HIGH);
  digitalWrite(InB2, LOW);
  
}

void motorStop()  {
  analogWrite(PWM1, 0);
  digitalWrite(InA1, LOW);
  digitalWrite(InB1, LOW);
  
  analogWrite(PWM2, 0);
  digitalWrite(InA2, LOW);
  digitalWrite(InB2, LOW);

}

//Kocenje oba motora istovremeno
void motorBrake()  {
  
  analogWrite(PWM1, 0);
  digitalWrite(InA1, HIGH);
  digitalWrite(InB1, HIGH);
  
  analogWrite(PWM2, 0);
  digitalWrite(InA2, HIGH);
  digitalWrite(InB2, HIGH);

  run = false;                  //brisanje svih varijabli vezanih za regulaciju. Postavljanje na pocetno stanje 
  U_pos1=0;                     //Brisanje varijabli pozicione regulacije
  e_pos1=0;
  e_pos1_old=0;
  Ref_speed1=0;                  //Brisanje varijabli brzinske regulacije
  sum_e_speed1=0;
  
  U_pos2=0;                       //Brisanje varijabli pozicione regulacije
  e_pos2=0;                       
  e_pos2_old=0;
  Ref_speed2=0;                   //Brisanje varijabli brzinske regulacije
  sum_e_speed2=0;
  
  
}

//Kocenje motora 1
void motor1Brake()   {
  analogWrite(PWM1, 0);
  digitalWrite(InA1, HIGH);
  digitalWrite(InB1, HIGH);
  
                                //brisanje svih varijabli vezanih za regulaciju. Postavljanje na pocetno stanje 
  U_pos1=0;                     //Brisanje varijabli pozicione regulacije
  e_pos1=0;
  e_pos1_old=0;
  Ref_speed1=0;                  //Brisanje varijabli brzinske regulacije
  sum_e_speed1=0;
  
};

void motor2Brake()   {
  analogWrite(PWM2, 0);
  digitalWrite(InA2, HIGH);
  digitalWrite(InB2, HIGH);

  U_pos2=0;                       //Brisanje varijabli pozicione regulacije
  e_pos2=0;                       
  e_pos2_old=0;
  Ref_speed2=0;                   //Brisanje varijabli brzinske regulacije
  sum_e_speed2=0;
};

 void rotirajDesno(float ugao){

   run=true;                                                    //dozvola kretanja
   run1=true;
   run2=true;
   Ref_pos1= Ref_pos1 + W2W_distance*3.14*ugao/360;             //na osnovu potrebnog ugla racuna se koliki put treba da predju levi i desni tocak
   Ref_pos2= Ref_pos2 - W2W_distance*3.14*ugao/360;             //put je jednak luku koji opisuje yadati ugao. Ugao se prebacuje u raijane i mnozi sa razdaljinom od centra robota to docka
    
   while (run) {                                              // dokle god je dozvoljeno kretanje poziva se regulacija 
                if(millis()-StartTime>=85000){kraj=true; while(1);}     // ukoliko vreme trajanja procesa istekne pokrece se beskonacna petlja
                if(millis()>=(CurrentTime+LOOPTIME_SPEED)){                      // provera da li je protekla perioda regulacije
                                                    CurrentTime=millis();
                                                    Speed_regulator();    //Pozivanje brzinske regulacije
                                                    }  // kraj izvrsenja regulacije
        
                 }// kraj while petlje
 }// kraj rotirajDesno funkcije
 
 void rotirajLevo(float ugao){

   run=true;
   run1=true;
   run2=true;
   Ref_pos1= Ref_pos1 - W2W_distance*3.14*ugao/360;        //na osnovu potrebnog ugla racuna se koliki put treba da predju levi i desni tocak
   Ref_pos2= Ref_pos2 + W2W_distance*3.14*ugao/360;        //put je jednak luku koji opisuje yadati ugao. Ugao se prebacuje u raijane i mnozi sa razdaljinom od centra robota to docka
    
   while (run) {                                           // dokle god je dozvoljeno kretanje poziva se regulacija 
                if(millis()-StartTime>=85000){kraj=true; while(1);} // ukoliko vreme trajanja procesa istekne pokrece se beskonacna petlja
                if(millis()>=(CurrentTime+LOOPTIME_SPEED)){         // provera da li je protekla perioda regulacije
                                          CurrentTime=millis();
                                          Speed_regulator();        //Pozivanje brzinske regulacije
                                          }
                }
 }
 
 void translatornoKretanje(float duzina){
   
   run=true;        //dozvola kretanja
   run1=true;
   run2=true;
   Ref_pos1+=duzina;
   Ref_pos2+=duzina;
   
while (run) {
    if(millis()-StartTime>=850000){kraj=true; while(1);}
    
                if(millis()>=(CurrentTime+5)){
                                    CurrentTime=millis();
                                    Speed_regulator();
                                    }
              
      }
      //motorBrake();
      

                 }
