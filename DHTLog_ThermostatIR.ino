/*---------------------------------------------------
HTTP 1.1 Temperature & Humidity Webserver for ESP8266 
and ext value for Raspberry Thermostat

by Jpnos 2017  - free for everyone

 original developer :
by Stefan Thesen 05/2015 - free for anyone

Connect DHT21 / AMS2301 at GPIO2
---------------------------------------------------*/

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include "time_ntp.h"
#include "DHT.h"
#include <ArduinoOTA.h>
#include <ESP8266mDNS.h>
#include "RecIR.h"
#include "SendIR.h"
#include "irl.h"
#include "ssd1306.h"
////////////// START CONFIGURAZIONE ///////////

// WiFi connection
const char* ssid = "home";
const char* password = "marzia2009";

//enable screen !!!Attenzione se non si ha lo schermo mettere a zero
byte screen_on = 1;     //abilito la visualizzazione
byte irread_on = 1;     //abilito ricezione ir
byte irsend_on = 1;      //abilito trasmissione ir
byte dht22_on  = 1;      // abilito lettura  dht22

int RELE_ON = 0;       // selezionare il tipo di on se optoisolato = 0 se diretto = 1;
int RELE_OFF= 1;       // selezionare il contrario di RELE_ON

int dht22_pin = 0  ;      // Selezionare il pin dov e collegato il dht 22
byte dht22_media = 1  ;   // 0 lettura dht pin diretta -- 1 lettura Mediata 


// Static IP details...
int dhcp = 0 ;// 0 uso static ip ---- 1 uso dhcp
IPAddress ip(192, 168, 1, 40);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192,168,1,1);
//////////FINE CONFIGURAZIONE


// storage for Measurements; keep some mem free; allocate remainder
#define KEEP_MEM_FREE 9216
#define MEAS_SPAN_H 24
unsigned long ulMeasCount=0;          // values already measured
unsigned long ulNoMeasValues=0;       // size of array
unsigned long ulMeasDelta_ms;         // distance to next meas time
unsigned long ulNextMeas_ms;          // next meas time
unsigned long *pulTime;               // array for time points of measurements
float *pfTemp,*pfHum;                 // array for temperature and humidity measurements
unsigned long ulReqcount;             // how often has a valid page been requested
unsigned long ulReconncount;          // how often did we connect to WiFi
int irRead;                           // enable read ir data
int chekEnable = 0;                   // comando ricevuto
int RELEPIN = 14;                     // pin per RELE
float tempHist=0.2;                   // temp histeresis
float dhtTemp,dhtTemp0,dhtTemp1;      // dht read temperature
float dhtUm,dhtUm0,dhtUm1;	          // dht read umidita
float setTemp;                        // set Temp to check
unsigned long ulNextntp;              // next ntp check
unsigned long timezoneRead();         // read time form internet db
String stato= "All OFF";              //stato sistema


decode_results results; // Somewhere to store the results

// Create an instance of the server on Port 80
WiFiServer server(80);

//////////////////////////////
// DHT21 / AMS2301 pin
//////////////////////////////
#define DHTPIN dht22_pin

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11 
#define DHTTYPE DHT22   // DHT 22  (AM2302)
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

// init DHT; 3rd parameter = 16 works for ESP8266@80MHz
DHT dht(DHTPIN, DHTTYPE,16);  

// needed to avoid link error on ram check
extern "C" 
{
#include "user_interface.h"
}

/////////////////////
// the setup routine
/////////////////////
void setup() 
{
  // setup globals
  ulReqcount=0; 
  ulReconncount=0;
  irRead=0;
  
  irrecv.setUnknownThreshold(MIN_UNKNOWN_SIZE);
  irrecv.enableIRIn();  // Start the receiver  
  irsend.begin();  //start the sender
  
  pinMode(RELEPIN,OUTPUT);
  digitalWrite(RELEPIN,RELE_OFF);
  
  // start serial
  Serial.begin(115200);
  Serial.println("Esp8266 WI-FI Temp/Umidita Jpnos-2017 v1.0");
  if( screen_on ==1){
    initDisplay();
  }
  delay(100);
  
  // inital connect
  WiFi.mode(WIFI_STA);
  WiFiStart();
  //////// Inizializzo tabella per dati 
  
  uint32_t free=system_get_free_heap_size() - KEEP_MEM_FREE;
  ulNoMeasValues = free / (sizeof(float)*2+sizeof(unsigned long));  // humidity & temp --> 2 + time
  pulTime = new unsigned long[ulNoMeasValues];
  pfTemp = new float[ulNoMeasValues];
  pfHum = new float[ulNoMeasValues];
  
  if (pulTime==NULL || pfTemp==NULL || pfHum==NULL)
    {
      ulNoMeasValues=0;
      Serial.println("Errore in allocazione di memoria!");
    }
  else
    {
      Serial.print("Memoria Preparata per ");
      Serial.print(ulNoMeasValues);
      Serial.println(" data points.");
      float fMeasDelta_sec = MEAS_SPAN_H*3600./ulNoMeasValues;
      ulMeasDelta_ms = ( (unsigned long)(fMeasDelta_sec) ) * 1000;  // round to full sec
      Serial.print("La misura avviene ogni");
      Serial.print(ulMeasDelta_ms);
      Serial.println(" ms.");
      ulNextMeas_ms = millis()+ulMeasDelta_ms;
    }
  //////////////////////////////////
    // OTA UPDATE
    /////////////////////////////////
    // Port defaults to 8266
  ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname("DhtLog-esp8266");

  // No authentication by default
  //ArduinoOTA.setPassword((const char *)"5622");

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
    });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
    });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
  ArduinoOTA.begin();
  
  
  Serial.println("OTA Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  
}

///////////////////
// (re-)start WiFi
///////////////////
void WiFiStart()
{ 
  ulReconncount++;
  if ( screen_on == 1){
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0,25);
    display.print("Start WiFi");
    display.display();
  }
  // Connect to WiFi network
  if (dhcp == 0){
      WiFi.config(ip, dns, gateway, subnet);
  }
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) 
    { 
      delay(500);
      Serial.print(".");
    }
  Serial.println("");
  Serial.println("WiFi connessa ");
  
  // Start the server
  server.begin();
  Serial.println("Server inizializzato : ");

  // Print the IP address
  Serial.println(WiFi.localIP());
  if ( screen_on == 1){
    display.clearDisplay();
  }
  ntpacquire();
}
 void ntpacquire() 
  {
  ///////////////////////////////
  // connect to NTP and get time
  ///////////////////////////////
  if ( screen_on == 1){
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0,20);
    display.print("Load...");
    display.print("...Time");
    display.display();
  }
  ulSecs2000_timer=getNTPTimestamp();
  Serial.print("Ora Corrente da server NTP : " );
  Serial.println(epoch_to_string(ulSecs2000_timer).c_str());
  ulSecs2000_timer -= millis()/1000;  // keep distance to millis() counter
  ulNextntp=millis()+3600000;
  delay(1000);
  /////////////////////
  //timezoneRead()
  /////////////////////
  unsigned long timeInternet=timezoneRead();
  if (!timeInternet == 0)
    {
      ulSecs2000_timer=timeInternet;
      Serial.print("Ora Corrente da server Internet: " );
      Serial.println(epoch_to_string(ulSecs2000_timer).c_str());
      ulSecs2000_timer -= millis()/1000;  // keep distance to millis() counter
      ulNextntp=millis()+3600000;
    }
  if ( screen_on==1){
    display.clearDisplay();
  }
 }


////////////////////////////////////////////////////
// make google chart object table for measured data
////////////////////////////////////////////////////
unsigned long MakeList (WiFiClient *pclient, bool bStream)
{
  unsigned long ulLength=0;
  
  // he irrecv.setUnknownThreshold(MIN_UNKNOWN_SIZE);re we build a big list.
  // we cannot store this in a string as this will blow the memory   
  // thus we count first to get the number of bytes and later on 
  // we stream this out
  if (ulMeasCount>0) 
  { 
    unsigned long ulBegin;
    if (ulMeasCount>ulNoMeasValues)
    {
      ulBegin=ulMeasCount-ulNoMeasValues;
    }
    else
    {
      ulBegin=0;
    }
    
    String sTable="";
    for (unsigned long li=ulBegin;li<ulMeasCount;li++)
    {
      // result shall be ['18:24:08 - 21.5.2015',21.10,49.00],
      unsigned long ulIndex=li%ulNoMeasValues;
      sTable += "['";
      sTable += epoch_to_string(pulTime[ulIndex]).c_str();
      sTable += "',";
	  sTable += pfTemp[ulIndex];
      sTable += ",";
      sTable += pfHum[ulIndex];
      sTable += "],\n";

      // play out in chunks of 1k
      if(sTable.length()>1024)
      {
        if(bStream)
        {
          pclient->print(sTable);
          //pclient->write(sTable.c_str(),sTable.length());
        }
        ulLength+=sTable.length();
        sTable="";
      }
    }
    
    // remaining chunk
    if(bStream)
    {
      pclient->print(sTable);
      //pclient->write(sTable.c_str(),sTable.length());
    } 
    ulLength+=sTable.length();  
  }
  
  return(ulLength);
}


//////////////////////////
// create HTTP 1.1 header
//////////////////////////
String MakeHTTPHeader(unsigned long ulLength)
{
  String sHeader;
  
  sHeader  = F("HTTP/1.1 200 OK\r\nContent-Length: ");
  sHeader += ulLength;
  sHeader += F("\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n");
  
  return(sHeader);
}


////////////////////
// make html footer
////////////////////
String MakeHTTPFooter()
{
  String sResponse;
  
  sResponse  = F("<p align=\"center\"><FONT SIZE=-1><BR>Richieste ="); 
  sResponse += ulReqcount;
  sResponse += F(" - Count ="); 
  sResponse += ulReconncount;
  sResponse += F(" - RAM Libera =");
  sResponse += (uint32_t)system_get_free_heap_size();
  sResponse += F(" - Punti max =");
  sResponse += ulNoMeasValues;
  sResponse += F(" - Stato : ");
  sResponse += stato;
  sResponse += F("<BR>Jpnos 2017<BR></p></body></html>");
  
  return(sResponse);
}
void DecodingIr()
{
decode_results  results;        // Somewhere to store the results
// Check if the IR code has been received.
  if (irrecv.decode(&results)) {
    // Display a crude timestamp.
    uint32_t now = millis();
    Serial.printf("Timestamp : %06u.%03u\n", now / 1000, now % 1000);
    if (results.overflow){
      Serial.printf("WARNING: IR code is too big for buffer (>= %d). "
                    "This result shouldn't be trusted until this is resolved. "
                    "Edit & increase CAPTURE_BUFFER_SIZE.\n",
                    CAPTURE_BUFFER_SIZE);
    }
    // Display the library version the message was captured with.
    Serial.print("Library   : v");
    Serial.println(_IRREMOTEESP8266_VERSION_);
    Serial.println();

    // Output RAW timing info of the result.
    Serial.println(resultToTimingInfo(&results));
    yield();  // Feed the WDT (again)

    // Output the results as source code
    Serial.println(resultToSourceCode(&results));
    Serial.println("");  // Blank line between entries
    yield();  // Feed the WDT (again)
  }
}

void ReadDht(){
    dhtTemp0 = dht.readTemperature();
    dhtUm0 = dht.readHumidity();
      if (isnan(dhtTemp0) || isnan(dhtUm0)) 
      {
          Serial.println("Failed to read from DHT sensor!"); 
      }   
      else
      {
        if (dht22_media == 1)
        {
          if (ulMeasCount > 0){
            dhtTemp1 = (dhtTemp+dhtTemp0)/2;
            dhtTemp= dhtTemp1;
            dhtUm1= (dhtUm+dhtUm0)/2;
            dhtUm=dhtUm1;  
          }
          else 
          {
            dhtTemp=dhtTemp0;
            dhtUm=dhtUm0; 
          }
        }
        else{
          dhtTemp=dhtTemp0;
          dhtUm=dhtUm0;
        }
        pfHum[ulMeasCount%ulNoMeasValues] = dhtUm;
        pfTemp[ulMeasCount%ulNoMeasValues] = dhtTemp;
        pulTime[ulMeasCount%ulNoMeasValues] = millis()/1000+ulSecs2000_timer;
        Serial.print("Logging Temperature: "); 
        Serial.print(pfTemp[ulMeasCount%ulNoMeasValues]);
        Serial.print(" deg Celsius - Humidity: "); 
        Serial.print(pfHum[ulMeasCount%ulNoMeasValues]);
        Serial.print("% - Time: ");
        Serial.println(pulTime[ulMeasCount%ulNoMeasValues]);
        ulMeasCount++;
      }
}
void DisplayText() {
  if ( screen_on == 1){
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(0,5);
    display.print(epoch_to_date(millis()/1000+ulSecs2000_timer));
    display.setTextSize(2);
    display.setCursor(65,0);
    display.print(epoch_to_time(millis()/1000+ulSecs2000_timer));
    display.setCursor(0,27);
    display.print(pfTemp[(ulMeasCount-1)%ulNoMeasValues],1);
    display.print((char)247);
    display.setCursor(66,27);
    display.print(pfHum[(ulMeasCount-1)%ulNoMeasValues],1);
    display.print("%");
    display.setTextSize(1);
    display.setCursor(0,53);
    display.print(stato);
    display.display();
  }
}


/////////////
// main loop
/////////////
void loop() {
///////////Aggiorno display
if ( screen_on == 1){
  DisplayText();
  delay(5);// The repeating section of the code
//

  
}

///////////////////
  // do data logging
  ///////////////////
  if  (ulMeasCount==0){
      if (dht22_on == 1){
        ReadDht();
      }
      ulNextMeas_ms = millis()+ulMeasDelta_ms;
   }
   
  if (millis()>=ulNextMeas_ms) 
  {
   if (dht22_on == 1){
      ReadDht();
   }
   ulNextMeas_ms = millis()+ulMeasDelta_ms;
   //Serial.print(String(setTemp)+" - "+String(tempHist)+" - "+stato);
   switch (chekEnable)
      {
        case 0:
          stato="All OFF ";
          break;
        case 3 :
          if (setTemp >= pfTemp[(ulMeasCount-1)%ulNoMeasValues]+tempHist)
           {
            Serial.println("acceso rele :\n");
            digitalWrite ( RELEPIN,RELE_ON);
            stato="zona ON Set: "+String(setTemp,1);
            chekEnable = 3;
          }
          else if ( setTemp <= pfTemp[(ulMeasCount-1)%ulNoMeasValues])
            {
            Serial.println("spento rele: \n");
            digitalWrite ( RELEPIN , RELE_OFF) ;
            stato="zona OFF Set: "+String(setTemp,1);
            chekEnable = 100;
            }
          break;
         case 100 :
          if (setTemp >= pfTemp[(ulMeasCount-1)%ulNoMeasValues]+tempHist)
           {
            Serial.println("acceso rele :\n");
            digitalWrite ( RELEPIN,RELE_ON);
            stato="zona ON Set: "+String(setTemp,1);
            chekEnable = 3;
          }
          else if ( setTemp <= pfTemp[(ulMeasCount-1)%ulNoMeasValues])
            {
            Serial.println("spento rele: \n");
            digitalWrite ( RELEPIN , RELE_OFF) ;
            stato="zona OFF Set: "+String(setTemp,1);
            chekEnable = 100;
            }
            break;
          case 1 :
            stato="Cool ON Set: "+String(setTemp,1);
            break;
          case 2 :
            stato="Cool OFF ";
            break;
          case 99 :
          break;
          case 200:
            stato= "Rele Off ";
            break;
          case 201:
            stato = "Rele ON ";
            break;
      }
  }
  if (irRead == 1){
    if (irread_on ==1 ){
      DecodingIr();
    }
  }
	

  
  //////////////////////////////
  // check if WLAN is connected
  //////////////////////////////
  if (WiFi.status() != WL_CONNECTED)
  {
    WiFiStart();
  }

  ///////////////////////////////////
  //OTA 
  //////////////////////////////////
  ArduinoOTA.handle();
  
  ///////////////////////////////////
  // Check if a client has connected
  ///////////////////////////////////
  WiFiClient client = server.available();
  if (!client) 
  {
    return;
  }
  
  // Wait until the client sends some data
  Serial.println("new client");
  unsigned long ultimeout = millis()+250;
  while(!client.available() && (millis()<ultimeout) )
  {
    delay(1);
  }
  if(millis()>ultimeout) 
  { 
    Serial.println("client connection time-out!");
    return; 
  }
  
  /////////////////////////////////////
  // Read the first line of the request
  /////////////////////////////////////
  String sRequest = client.readStringUntil('\r');
  //Serial.println(sRequest);
  client.flush();
  
  // stop client, if request is empty
  if(sRequest=="")
  {
    Serial.println("empty request! - stopping client");
    client.stop();
    return;
  }
  
  // get path; end of path is either space or ?
  // Syntax is e.g. GET /?show=1234 HTTP/1.1
  String sPath="",sParam="", sCmd="";
  String sGetstart="GET ";
  int iStart,iEndSpace,iEndQuest;
  iStart = sRequest.indexOf(sGetstart);
  if (iStart>=0)
  {
    iStart+=+sGetstart.length();
    iEndSpace = sRequest.indexOf(" ",iStart);
    iEndQuest = sRequest.indexOf("?",iStart);
    
    // are there parameters?
    if(iEndSpace>0)
    {
      if(iEndQuest>0)
      {
        // there are parameters
        sPath  = sRequest.substring(iStart,iEndQuest);
        sParam = sRequest.substring(iEndQuest+1,iEndSpace);
        Serial.println(sParam);
      }
      else
      {
        // NO parameters
        sPath  = sRequest.substring(iStart,iEndSpace);
      }
    }
  }
  ///////////////////////////
  ///check NTP
  //////////////////////////
   if (millis()>=ulNextntp){
    ntpacquire();
  }  
  
  ///////////////////////////
  // format the html response
  ///////////////////////////
  String sResponse,sResponse2,sHeader;
  
  /////////////////////////////
  // format the html page for /
  ///////////////////////0//////
  if(sPath=="/") 
  {
    ulReqcount++;
    int iIndex= (ulMeasCount-1)%ulNoMeasValues;
    sResponse  = F("<html>\n<head>\n<title>Wi-FI Logger per Temperatura e Umidita</title>\n<script type=\"text/javascript\" src=\"https://www.google.com/jsapi?autoload={'modules':[{'name':'visualization','version':'1','packages':['gauge']}]}\"></script>\n<script type=\"text/javascript\">\nvar temp=");
    sResponse += pfTemp[iIndex];
    sResponse += F(",hum=");
    sResponse += pfHum[iIndex];
    sResponse += F(";\ngoogle.load('visualization', '1', {packages: ['gauge']});google.setOnLoadCallback(drawgaugetemp);google.setOnLoadCallback(drawgaugehum);\nvar gaugetempOptions = {min: -20, max: 50, yellowFrom: -20, yellowTo: 0,greenFrom: 0, greenTo: 30,redFrom: 30, redTo: 50, minorTicks: 10, majorTicks: ['-20','-10','0','10','20','30','40','50']};\n");
    sResponse += F("var gaugehumOptions = {min: 0, max: 100, yellowFrom: 0, yellowTo: 25, greenFrom: 25, greenTo: 75,redFrom: 75, redTo: 100, minorTicks: 10, majorTicks: ['0','10','20','30','40','50','60','70','80','90','100']};\nvar gaugetemp,gaugehum;\n\nfunction drawgaugetemp() {\ngaugetempData = new google.visualization.DataTable();\n");
    sResponse += F("gaugetempData.addColumn('number', 'C');\ngaugetempData.addRows(1);\ngaugetempData.setCell(0, 0, temp);\ngaugetemp = new google.visualization.Gauge(document.getElementById('gaugetemp_div'));\ngaugetemp.draw(gaugetempData, gaugetempOptions);\n}\n\n");
    sResponse += F("function drawgaugehum() {\ngaugehumData = new google.visualization.DataTable();\ngaugehumData.addColumn('number', '%');\ngaugehumData.addRows(1);\ngaugehumData.setCell(0, 0, hum);\ngaugehum = new google.visualization.Gauge(document.getElementById('gaugehum_div'));\ngaugehum.draw(gaugehumData, gaugehumOptions);\n}\n");
    sResponse += F("</script>\n</head>\n<body>\n<font color=\"#000000\"><body bgcolor=\"#d0d0f0\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=yes\">");
    sResponse += F("<p align=\"center\"><FONT SIZE=+3><FONT-WEIGHT=bold>WI-FI Logger per Temperatura e Umidita<BR><FONT SIZE=+1><FONT-WEIGHT=normal>con DHT22 su ESP8266 <BR><FONT SIZE=+1>Ultima Misura alle : ");
    sResponse += epoch_to_string(pulTime[iIndex]).c_str();
    sResponse += F(" <BR></p>\n<div style=\"width:400px; margin:0 auto\">\n<div id=\"gaugetemp_div\" style=\"float:left; width:200px; height: 200px\"></div> \n<div id=\"gaugehum_div\" style=\"float:left; width:200px; height: 200px\"></div>\n<div style=\"clear:both\"></div>\n</div>");
    
    sResponse2 +=F("<p style=\"text-align: center;\"><input name=\"grafico\" type=\"button\"  onclick=\"location.href= '/grafico'\"  value=\"Grafico\" />&nbsp; <input name=\"Irdecoder\" type=\"button\" value=\"IrDecoder\" onclick=\"location.href= '/irDecoder'\"/>&nbsp; <input name=\"IrSender\" type=\"button\" value=\"irSEnder\" onclick=\"location.href= '/irSender'\"/>&nbsp; <input name=\"IrSender\" type=\"button\" value=\"Zone\" onclick=\"location.href= '/zone'\"/></p>\r\n");

    sResponse2 += MakeHTTPFooter().c_str();
    
    // Send the response to the client 
    client.print(MakeHTTPHeader(sResponse.length()+sResponse2.length()).c_str());
    client.print(sResponse);
    client.print(sResponse2);
  }
  
  else if(sPath=="/grafico")
  ///////////////////////////////////
  // format the html page for /grafik
  ///////////////////////////////////
  {
    ulReqcount++;
    unsigned long ulSizeList = MakeList(&client,false); // get size of list first

    sResponse  = F("<html>\n<head>\n<title>WI-FI Logger per Temperatura e Umidita</title>\n<script type=\"text/javascript\" src=\"https://www.google.com/jsapi?autoload={'modules':[{'name':'visualization','version':'1','packages':['corechart']}]}\"></script>\n");
    sResponse += F("<script type=\"text/javascript\"> google.setOnLoadCallback(drawChart);\nfunction drawChart() {var data = google.visualization.arrayToDataTable([\n['Date / Time', 'Temperatura', 'Umidita'],\n");    
    // here the big list will follow later - but let us prepare the end first
      
    // part 2 of response - after the big list
    sResponse2  = F("]);\nvar options = {title: 'Letture',vAxes:{0:{viewWindowMode:'explicit',gridlines:{color:'black'},format:\"##.##\C\"},1: {gridlines:{color:'transparent'},format:\"##,##%\"},},series:{0:{targetAxisIndex:0},1:{targetAxisIndex:1},},curveType:'none',legend:{ position: 'bottom'}};");
    sResponse2 += F("var chart = new google.visualization.LineChart(document.getElementById('curve_chart'));chart.draw(data, options);}\n</script>\n</head>\n");
    sResponse2 += F("<body>\n<font color=\"#000000\"><body bgcolor=\"#d0d0f0\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=yes\"><p align=\"center\"><FONT SIZE=+3><FONT-WEIGHT=bold>WI-FI Logger per Temperatura e Umidita<BR><FONT SIZE=+1><FONT-WEIGHT=normal><a href=\"/\">Home Page</a><BR></p>");
    sResponse2 += F("<BR>\n<div id=\"curve_chart\" style=\"width: 600px; height: 400px; margin: 0 auto\"></div>");
    sResponse2 += MakeHTTPFooter().c_str();
    
    // Send the response to the client - delete strings after use to keep mem low
    client.print(MakeHTTPHeader(sResponse.length()+sResponse2.length()+ulSizeList).c_str()); 
    client.print(sResponse); sResponse="";
    MakeList(&client,true);
    client.print(sResponse2);
  }
  else if(sPath=="/dati")
  ///////////////////////////////////
  // format the html page for /dati ovvero passo dati in json
  ///////////////////////////////////
  {
    ulReqcount++;
    int iIndex= (ulMeasCount-1)%ulNoMeasValues;
    
    sResponse = "{";
    sResponse += "\"heap\": "+String(ESP.getFreeHeap());
    sResponse += ", \"S_temperature\": "+String(pfTemp[iIndex]);
    sResponse += ", \"S_humidity\": "+String(pfHum[iIndex]);
    sResponse += ", \"S_time\": \"" +epoch_to_string(pulTime[iIndex])+"\"";
    sResponse += ", \"S_control\": " + String(chekEnable);
    sResponse += ", \"S_setTemp\": \"" +String(setTemp)+"\"";
    sResponse += "}";
    Serial.print("Json: \n");
    Serial.print(sResponse);
    // Send the response to the client 
    client.print(MakeHTTPHeader(sResponse.length()).c_str());
    client.print(sResponse); 
    sResponse="";
    delay(2);
    client.stop();
  }
  else if(sPath=="/zone")
  ///////////////////////////////////
  // format the html page for /irDecoder 
  ///////////////////////////////////
  {
    ulReqcount++;
    sResponse = file4;
    // Send the response to the client 
    client.print(MakeHTTPHeader(sResponse.length()).c_str());
    client.print(sResponse); sResponse="";
  }
  else if(sPath=="/irDecoder")
  ///////////////////////////////////
  // format the html page for /irDecoder 
  ///////////////////////////////////
  {
    ulReqcount++;
    switch (sParam.toInt())
      {
     case 101:
          sResponse = file1;
          irRead = 0;
          stato = "All OFF";
          break;
     case 102:
          sResponse = file2;
          irRead = 1;
          stato = "Decoder ON";
          break;
      default: 
          sResponse = file1;
          irRead = 0;
          stato = "All OFF";
          break;  
      }
    // Send the response to the client 
    client.print(MakeHTTPHeader(sResponse.length()).c_str());
    client.print(sResponse); sResponse="";
  }
  
   ///////////////////////////////////
  // format the html page for Sender IR
  ///////////////////////////////////
  else if(sPath=="/irSender")
    {
    ulReqcount++;
    sResponse = file3;
     switch (sParam.toInt())
      {
        case 18:
         irsend.sendRaw(On18,iRlen,38);
         chekEnable = 1;
         setTemp= 18.0;
         stato="Cool ON Set: "+String(setTemp,1);
         
         break;
        case 19:
         irsend.sendRaw(On19,iRlen,38);
         chekEnable = 1;
         setTemp= 19.0;
         stato="Cool ON Set: "+String(setTemp,1);
         break;
        case 20:
         irsend.sendRaw(On20,iRlen,38);
         chekEnable = 1;
         setTemp= 20.0;
         stato="Cool ON Set: "+String(setTemp,1);
         break;
        case 21:
         irsend.sendRaw(On21,iRlen,38);
         chekEnable = 1;
         setTemp= 21.0;
         stato="Cool ON Set: "+String(setTemp,1);
         break;
        case 22:
         irsend.sendRaw(On22,iRlen,38);
         chekEnable = 1;
         setTemp= 22.0;
         stato="Cool ON Set: "+String(setTemp,1);
         break;
        case 23:
         irsend.sendRaw(On23,iRlen,38);
         chekEnable = 1;
         setTemp= 23.0;
         stato="Cool ON Set: "+String(setTemp,1);
         break;
        case 24:
         irsend.sendRaw(On24,iRlen,38);
         chekEnable = 1;
         setTemp= 24.0;
         stato="Cool ON Set: "+String(setTemp,1);
         break;
        case 25:
         irsend.sendRaw(On24,iRlen,38);
         chekEnable = 1;
         setTemp= 25.0;
         stato="Cool ON Set: "+String(setTemp,1);
         break;
        case 26:
         irsend.sendRaw(On24,iRlen,38);
         chekEnable = 1;
         setTemp= 26.0;
         stato="Cool ON Set: "+String(setTemp,1);
         break;
        case 99:
         irsend.sendRaw(Off,iRlen,38);
         chekEnable = 2;
         //setTemp= sParam.toFloat();
         stato="Cool OFF";
         break;
        default:
          break;
      
      }
      client.print(MakeHTTPHeader(sResponse.length()).c_str());
      client.print(sResponse); 
      sResponse="";
      digitalWrite ( RELEPIN , RELE_OFF);
      Serial.print("Setto Cool con setTemp: "+String(setTemp,1)+"\n");
      irRead=0;
      delay(2);
      client.stop();
  }
 
  else if(sPath=="/zoneON")
    {
     ulReqcount++;
     Serial.println("comando Zone");
     sResponse  = F("<html><head><meta http-equiv=\"refresh\" content=\"0; url=/zone\" /></head>");
      client.print(MakeHTTPHeader(sResponse.length()).c_str());
      client.print(sResponse); sResponse="";
      chekEnable = 3;
      setTemp = sParam.toFloat();
      ulNextMeas_ms = millis();
      //Serial.print("Setto Temperature to : "+String(setTemp)+"\n");
    }
    else if(sPath=="/zoneOFF")
    {
     ulReqcount++;
     Serial.println("comando Zone");
     sResponse  = F("<html><head><meta http-equiv=\"refresh\" content=\"0; url=/zone\" /></head>");
      client.print(MakeHTTPHeader(sResponse.length()).c_str());
      client.print(sResponse); sResponse="";
      chekEnable = 100;
      setTemp = 0;
      digitalWrite ( RELEPIN , RELE_OFF);
      ulNextMeas_ms = millis();
    }
   else if(sPath=="/clear")
    {
     ulReqcount++;
     Serial.println("comando Clear All");
     sResponse  = F("<html><head></head>");
     client.print(MakeHTTPHeader(sResponse.length()).c_str());
     client.print(sResponse); sResponse="";
     chekEnable = 0;
     setTemp = 0;
     digitalWrite ( RELEPIN , RELE_OFF);
     irsend.sendRaw(Off,iRlen,38);
     stato="All OFF ";
     delay(2);
     client.stop();
    }
    else if(sPath=="/releON")
    {
     ulReqcount++;
     Serial.println("comando Rele ON");
     sResponse  = F("<html><head></head>");
     client.print(MakeHTTPHeader(sResponse.length()).c_str());
     client.print(sResponse); sResponse="";
      chekEnable = 201;
      setTemp = 0;
      digitalWrite ( RELEPIN , RELE_ON);
      stato="Rele ON  ";
      delay(2);
      client.stop();
    }
    else if(sPath=="/releOFF")
    {
     ulReqcount++;
     Serial.println("comando Rele Off");
     sResponse  = F("<html><head></head>");
     client.print(MakeHTTPHeader(sResponse.length()).c_str());
     client.print(sResponse); sResponse="";
      chekEnable = 200;
      setTemp = 0;
      digitalWrite ( RELEPIN , RELE_OFF);
      stato="Rele OFF ";
      delay(2);
      client.stop();
    }
    else if(sPath=="/testLan")
    {
     ulReqcount++;
     Serial.println("Test Lan \n");
     sResponse  = F("<html><head></head>");
     client.print(MakeHTTPHeader(sResponse.length()).c_str());
     client.print(sResponse); sResponse="";
      delay(1);
      client.stop();
    }
  else
  ////////////////////////////
  // 404 for non-matching path
  ////////////////////////////
  {
    sResponse="<html><head><title>404 Not Found</title></head><body><h1>Not Found</h1><p>The requested URL was not found on this server.</p></body></html>";
    
    sHeader  = F("HTTP/1.1 404 Not found\r\nContent-Length: ");
    sHeader += sResponse.length();
    sHeader += F("\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n");
    
    // Send the response to the client
    client.print(sHeader);
    client.print(sResponse);
  }
  
  // and stop the client
  delay(2);
  client.stop();
  Serial.println("Client disconnected");
}

