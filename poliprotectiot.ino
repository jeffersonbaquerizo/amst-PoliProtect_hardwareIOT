#include <Arduino.h>
#if defined(ESP32)
#include <WiFi.h>
#include "soc/soc.h"           // Disable brownout problems
#include "soc/rtc_cntl_reg.h"  // Disable brownout problems
#include "driver/rtc_io.h"
#include <SPIFFS.h>
#include <FS.h>
#include <Firebase_ESP_Client.h>
#include "esp_camera.h"
#include "Base64.h"
#endif

// Provide the token generation process info.
#include <addons/TokenHelper.h>

// Provide the RTDB payload printing info and other helper functions.
#include <addons/RTDBHelper.h>

// GPS library
#include <TinyGPSPlus.h>

// library timer interruptions
#include <esp_timer.h>

// ==================================
//     Enter your WiFi credentials
// ==================================
const char* ssid = "NETLIFE_CORNEJO";
const char* password = "17242630";

// ======================================
//        Firebase Credentials
// ======================================
// Define the API Key
#define API_KEY "AIzaSyAJRSQnfMhIX8zuF4jvzugfQ2oWAqFq-W8"

// Define the RTDB URL
#define DATABASE_URL "poliprotect-6b93c-default-rtdb.firebaseio.com" //<databaseName>.firebaseio.com or <databaseName>.<region>.firebasedatabase.app
#define STORAGE_BUCKET_ID "poliprotect-6b93c.appspot.com" // Storage url

// Define the user Email and password that alreadey registerd or added in your project
#define USER_EMAIL "bus_5214648965@poliprotect.com"
#define USER_PASSWORD "5214648965"

// Define file paths of the photo
#define FILE_PHOTO "/data/photo.jpg"

// ==================
//    Camera pins
// ==================
#define PWDN_GPIO_NUM             32
#define RESET_GPIO_NUM           -1
#define XCLK_GPIO_NUM             0
#define SIOD_GPIO_NUM             26
#define SIOC_GPIO_NUM             27

#define Y9_GPIO_NUM               35
#define Y8_GPIO_NUM               34
#define Y7_GPIO_NUM               39
#define Y6_GPIO_NUM               36
#define Y5_GPIO_NUM               21
#define Y4_GPIO_NUM               19
#define Y3_GPIO_NUM               18
#define Y2_GPIO_NUM               5
#define VSYNC_GPIO_NUM            25
#define HREF_GPIO_NUM             23
#define PCLK_GPIO_NUM             22

// 4 for flash led or 33 for normal led
#define LED_GPIO_NUM              4
#define LED_LEDC_CHANNEL          2
#define CONFIG_LED_MAX_INTENSITY  255
int led_duty = 90;

// Leds that indicate any system states 
#define LED_ON                    12
#define LED_IS_CONNECTED          13
#define LED_ISNT_CONNECTED        15
#define LED_AUTH                  0

// ===========================
//     Device Information
// ===========================
// Device ID
String id = "5214648965";

// Device route
String ruta = "city_mall";

// Define Firebase Data object
FirebaseData fbdo;

// Define Firebase Auth
FirebaseAuth auth;

// Define Firebase Config
FirebaseConfig configFire;

// The TinyGPSPlus object
TinyGPSPlus gps;

// Define Camera Config
camera_config_t configCam;

// Variables related to the report
String idReporte = "20230813T202020";

// Optimization variables
unsigned long lastCaptureTime = 0;
unsigned long lastFirebaseTime = 0;
unsigned long lastADCTime = 0;

// GPS variables
float latitude = 0.0;
float longitude = 0.0;
int satelite = 0;

void pinConfig() {
  // Set led pins
  pinMode(LED_ON, OUTPUT);
  pinMode(LED_IS_CONNECTED, OUTPUT);
  pinMode(LED_ISNT_CONNECTED, OUTPUT);
  pinMode(LED_AUTH, OUTPUT);

  // Leds initial conditions
  digitalWrite(LED_ON, LOW);
  digitalWrite(LED_IS_CONNECTED, LOW);
  digitalWrite(LED_ISNT_CONNECTED, HIGH);
  digitalWrite(LED_AUTH, LOW);

  // Set the flash led parameters
  ledcSetup(LED_LEDC_CHANNEL, 5000, 8);
  ledcAttachPin(LED_GPIO_NUM, LED_LEDC_CHANNEL);
}

void wifiInit() {
  // Define the wifi credentials
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);
  
  // Message of connection
  Serial.print("WiFi connecting ");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
 
  // Indicate the connected red
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
}

void spiffsInit() {
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    SPIFFS.format();
    ESP.restart();
  }
  else {
    delay(500);
    Serial.println("SPIFFS mounted successfully");
    if (!SPIFFS.exists("/data")) {
        SPIFFS.mkdir("/data");
    }
  }
}

void cameraConfig() {
  // Set pines connected to the camera 
  configCam.ledc_channel = LEDC_CHANNEL_0;
  configCam.ledc_timer = LEDC_TIMER_0;
  configCam.pin_d0 = Y2_GPIO_NUM;
  configCam.pin_d1 = Y3_GPIO_NUM;
  configCam.pin_d2 = Y4_GPIO_NUM;
  configCam.pin_d3 = Y5_GPIO_NUM;
  configCam.pin_d4 = Y6_GPIO_NUM;
  configCam.pin_d5 = Y7_GPIO_NUM;
  configCam.pin_d6 = Y8_GPIO_NUM;
  configCam.pin_d7 = Y9_GPIO_NUM;
  configCam.pin_xclk = XCLK_GPIO_NUM;
  configCam.pin_pclk = PCLK_GPIO_NUM;
  configCam.pin_vsync = VSYNC_GPIO_NUM;
  configCam.pin_href = HREF_GPIO_NUM;
  configCam.pin_sccb_sda = SIOD_GPIO_NUM;
  configCam.pin_sccb_scl = SIOC_GPIO_NUM;
  configCam.pin_pwdn = PWDN_GPIO_NUM;
  configCam.pin_reset = RESET_GPIO_NUM;
  configCam.xclk_freq_hz = 20000000;
  configCam.frame_size = FRAMESIZE_VGA;
  configCam.pixel_format = PIXFORMAT_JPEG; // for streaming
  configCam.fb_location = CAMERA_FB_IN_PSRAM;
  configCam.jpeg_quality = 5;
  configCam.fb_count = 1;
  configCam.grab_mode = CAMERA_GRAB_LATEST;

  // Parameters for optimizing the photo quality
  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if(configCam.pixel_format == PIXFORMAT_JPEG){
    if(psramFound()){
      configCam.jpeg_quality = 10;
      configCam.fb_count = 2;
      configCam.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      configCam.frame_size = FRAMESIZE_SVGA;
      configCam.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    configCam.frame_size = FRAMESIZE_240X240;
  #if CONFIG_IDF_TARGET_ESP32S3
    configCam.fb_count = 2;
  #endif
  }
}

void cameraInit() {
  // camera init
  esp_err_t err = esp_camera_init(&configCam);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1); // flip it back
    s->set_brightness(s, 1); // up the brightness just a bit
    s->set_saturation(s, -2); // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  if(configCam.pixel_format == PIXFORMAT_JPEG){
    s->set_framesize(s, FRAMESIZE_QVGA);
  }
}

void firebaseInit() {
  // Indicate the Firebase version
  Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);

  // Assign the api key (required)
  configFire.api_key = API_KEY;

  // Assign the user sign in credentials
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;

  // Assign the RTDB URL (required)
  configFire.database_url = DATABASE_URL;

  // Assign the callback function for the long running token generation task
  configFire.token_status_callback = tokenStatusCallback; // see addons/TokenHelper.h

  Firebase.begin(&configFire, &auth);

  // Comment or pass false value when WiFi reconnection will control by your code or third party library
  Firebase.reconnectWiFi(true);

  Firebase.setDoubleDigits(5);
}

void setup() {
  // Set operation mode of the pins
  pinConfig();

  // Configuracion de la comunicacion serial
  Serial.begin(9600);
  Serial.setDebugOutput(true);
  Serial.println("");

  // Begin wifi connection
  wifiInit();

  // Build spiffs file format in the flash memory 
  spiffsInit();

  // ===================Investigar esta linea
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  // Set and initialize the camera
  cameraConfig();
  cameraInit();
  
  firebaseInit();

  // Inidicate the initialization and configuration of modules
  digitalWrite(LED_ON, HIGH);
}

void loop() {
  Firebase.RTDB.setString(&fbdo, F("Data"), F("Hola"));

  if(WiFi.status() != WL_CONNECTED){
    Serial.println("Se ha desconectado");
    digitalWrite(LED_ISNT_CONNECTED, HIGH);
    digitalWrite(LED_IS_CONNECTED, LOW);
  } else {
    Serial.println("Esta conectado");
    digitalWrite(LED_ISNT_CONNECTED, LOW);
    digitalWrite(LED_IS_CONNECTED, HIGH);

    if (Firebase.authenticated()) {
      Serial.println("Estás autenticado con Firebase");

      digitalWrite(LED_AUTH, HIGH);

      // Serial Communication
      if (Serial.available() > 0) {
        // Read the a line of message
        String inputData = Serial.readStringUntil('\n');
        inputData.trim(); // Remove leading/trailing whitespaces
        Serial.println("Fuera del if ");
  
        // There is message
        if (inputData.length() > 0) {
          Serial.println("Dentro del if ");

          if (gps.encode(Serial.read())){
            Serial.println("Dentro del 2 if ");

          }

          if(inputData == "Foto"){
            loadPhotoFirebase();

          }

          latitude = gps.location.lat();
          longitude = gps.location.lng();
          satelite = gps.satellites.value();

          Serial.print(gps.satellites.value(), 6);  
          Serial.print(F(","));
          Serial.print(gps.location.lat(), 6);
          Serial.print(F(","));
          Serial.println(gps.location.lng(), 6);

          Serial.println(inputData);

        }
      }

      unsigned long currentFirebaseTime = millis();

      // Check the necesary time for reading and updating data in the firebase
      if (currentFirebaseTime - lastFirebaseTime >= 1000){

        // Read and Update data in Firebase
        if (Firebase.RTDB.get(&fbdo, "/Data_app")) {
        
          Serial.println("Exite Data");

          // Define the paths for loading data in the firebase database
          String rutaDBLatstr = "/Data_app/listado_buses/"+ruta+"/bus_"+id+"/posicion/latitud";
          String rutaDBLngstr = "/Data_app/listado_buses/"+ruta+"/bus_"+id+"/posicion/longitud";
          String rutaDBSatstr = "/Data_app/listado_buses/"+ruta+"/bus_"+id+"/posicion/satelites";

          // Send data to the firebase database
          loadDataFloatFirebase(rutaDBLatstr, latitude);
          loadDataFloatFirebase(rutaDBLngstr, longitude);
          loadDataIntFirebase(rutaDBSatstr, satelite);

          // Paths for reading the data in the firebase database
          String rutaDBComandostr = "/Data_app/listado_buses/"+ruta+"/bus_"+id+"/comando";
          String rutaDBConnectionstr = "/Data_app/listado_buses/"+ruta+"/bus_"+id+"/conexion";

          char rutaDBComando[rutaDBComandostr.length() + 1];
          char rutaDBConnection[rutaDBConnectionstr.length() + 1];

          // Converts a string to a char array
          rutaDBComandostr.toCharArray(rutaDBComando, sizeof(rutaDBComando));
          rutaDBConnectionstr.toCharArray(rutaDBConnection, sizeof(rutaDBConnection));

          // Read command and connected in the firebase database
          String comando = Firebase.RTDB.getString(&fbdo, F(rutaDBComando)) ? fbdo.to<const char *>() : fbdo.errorReason().c_str();
          String connection = Firebase.RTDB.getString(&fbdo, F(rutaDBConnection)) ? fbdo.to<const char *>() : fbdo.errorReason().c_str();

          Serial.println("Obtuvo el Comando");
          Serial.println(comando);

          // Device actions because of commands
          if(comando.indexOf("foto") != -1){

            Serial.println("Entro en comando foto");

            // Separate the report id of command 
            String comandoParts[2];
            int numParts = 0;
            int lastIndex = 0;
            char delimiter = ',';

            for (int i = 0; i < comando.length(); i++) {
              if (comando.charAt(i) == delimiter) {
                comandoParts[numParts] = comando.substring(lastIndex, i);
                lastIndex = i + 1;
                numParts++;
              }
            }
            comandoParts[numParts] = comando.substring(lastIndex);

            idReporte = comandoParts[1];
            Serial.println("id del reporte: " + idReporte);

            // Load the photo in the Storage Database
            loadPhotoFirebase();

            // Indicate in the database that photo was sent
            Firebase.RTDB.setString(&fbdo, F(rutaDBComando), F("Done"));
          }

          // Answer of the message of connection check
          if(connection == "isConnected") {
            Firebase.RTDB.setString(&fbdo, F(rutaDBConnection), F("connected"));

          }
        } else {
          Serial.println("No exite base de datos");

        }
      }
    } else {
        Serial.println("No estás autenticado con Firebase");
        digitalWrite(LED_AUTH, LOW);

    }
  }
}

// Load the photo in the Firebase Storage Database
void loadPhotoFirebase(){
  unsigned long currentPhotoTime = millis();
  // Check the necesary time for taking other photo 
  if (currentPhotoTime - lastCaptureTime >= 1000){

    lastCaptureTime = currentPhotoTime;

    // Turn on flash led for capturing the photo
    enable_led(true);
    delay(200);
    Serial.println("Tomando foto");

    // Take the capture the photo
    camera_fb_t * photo = NULL;
    photo = esp_camera_fb_get();
    
    // Check the captured photo. If there is error, restart ESP32
    if(!photo) {
      enable_led(false); 
      Serial.println("Captura de la foto fallida");
      delay(200);
      ESP.restart();         
      return;

    }
    
    // Turn off flash led after capturing the photo
    delay(200);
    enable_led(false);         

    Serial.println("Foto tomada con exito");
    Serial.println("Tamaño de imagen: " + String(photo->len));

    // Create the file to store the image in the flash memory
    Serial.println("Se crea el archivo de almacenamiento de la imagen");
    File filePhoto = SPIFFS.open(FILE_PHOTO, FILE_WRITE);

    // Check if the file was created correctly
    if (!filePhoto) {
      Serial.println("Failed to open file in writing mode");

    }
    else {
      // Write the photo in the file
      filePhoto.write(photo->buf, photo->len); // payload (image), payload length
      Serial.print("La imagen fue almacenada en: ");
      Serial.print(FILE_PHOTO);
      Serial.print(" - Tamaño: ");
      Serial.print(filePhoto.size());
      Serial.println(" bytes");

    }

    // Close the file
    filePhoto.close();

    // Path for sending the photo in the Storage database
    String rutaDBSPhotostr = "/Reportes/"+id+"_"+idReporte+".jpg";
    char rutaDBSPhoto[rutaDBSPhotostr.length()+1]; 

    // Converts a string to a char array
    rutaDBSPhotostr.toCharArray(rutaDBSPhoto, sizeof(rutaDBSPhoto));

    // Load the photo file in the firebase Storage
    if (Firebase.Storage.upload(&fbdo, STORAGE_BUCKET_ID , FILE_PHOTO , mem_storage_type_flash, rutaDBSPhoto , "image/jpeg")){
      Serial.printf("\nDownload URL: %s\n", fbdo.downloadURL().c_str());

      String rutaDBFotostr = "/Data_app/listado_buses/"+ruta+"/bus_"+id+"/reportes/reporte_"+idReporte+"/foto";

      // Load the photo url in the RealTime Database
      loadDataStrFirebase(rutaDBFotostr, fbdo.downloadURL().c_str());
      Serial.println("Foto enviada exitosamente");

    }
    else{
      Serial.println(fbdo.errorReason());

    }

    // Free image buffer
    esp_camera_fb_return(photo); 
    Serial.println("Memoria limpia");

  }
}

// Enable the camera flash led
void enable_led(bool en){
    int duty = en ? led_duty : 0;
    if (en && (led_duty > CONFIG_LED_MAX_INTENSITY))
    {
        duty = CONFIG_LED_MAX_INTENSITY;
    }
    ledcWrite(LED_LEDC_CHANNEL, duty);
}

// Load String in the Firebase Database
void loadDataStrFirebase(String ruta, String data){
  char rutaDB[ruta.length() + 1];
  char dataDB[data.length() + 1];
  
  // Converts a string to a char array
  ruta.toCharArray(rutaDB, sizeof(rutaDB));
  data.toCharArray(dataDB, sizeof(dataDB));

  // Send the string to the Firebase Database
  Firebase.RTDB.setString(&fbdo, F(rutaDB), F(dataDB));

}

// Load Float in the Firebase Database
void loadDataFloatFirebase(String ruta, float data){
  char rutaDB[ruta.length() + 1];
  
  // Converts a string to a char array
  ruta.toCharArray(rutaDB, sizeof(rutaDB));

  // Send the float to the Firebase Database
  Firebase.RTDB.setFloat(&fbdo, F(rutaDB), data);

}

// Load Float in the Firebase Database
void loadDataIntFirebase(String ruta, int data){
  char rutaDB[ruta.length() + 1];
  
  // Converts a string to a char array
  ruta.toCharArray(rutaDB, sizeof(rutaDB));

  // Send the int to the Firebase Database
  Firebase.RTDB.setInt(&fbdo, F(rutaDB), data);

}