#include <WiFi.h>             
#include <ESP32WebServer.h>    //https://github.com/Pedroalbuquerque/ESP32WebServer download and place in your Libraries folder
#include <ESPmDNS.h>
#include <SerialFlash.h>      //https://github.com/PaulStoffregen/SerialFlash.git
#include "CSS.h" 
#include <SPI.h>

ESP32WebServer server(80);

#define servername "fileupload" 
const int FlashChipSelect = 5; // digital pin for flash chip CS pin

bool   SD_present = false;

void setup(void)
{  
  Serial.begin(115200);
  WiFi.softAP("esp32", "12345678"); 
  
  if (!MDNS.begin(servername)) 
  {          
    Serial.println(F("Error setting up MDNS responder!")); 
    ESP.restart(); 
  } 
  Serial.print(F("Initializing Flash Chip..."));
  
  if (!SerialFlash.begin(FlashChipSelect)) {
    Serial.println("Unable to access SPI Flash chip");
    SD_present = false; 
  }
  else
  {
    Serial.println(F("Flash chip initialised... file access enabled..."));
    SD_present = true; 
  }
  
  /*********  Server Commands  **********/
  server.on("/",         SD_dir);
  server.on("/upload",   File_Upload);
  server.on("/fupload",  HTTP_POST,[](){ server.send(200);}, handleFileUpload);

  server.begin();
  
  Serial.println("HTTP server started");
}

/*********  LOOP  **********/

void loop(void)
{
  server.handleClient(); //Listen for client connections
}

/*********  FUNCTIONS  **********/
void SD_dir()
{
  if (SD_present) 
  {
    if (server.args() > 0 ) //Arguments were received, ignored if there are not arguments
    { 
      Serial.println(server.arg(0));
  
      String Order = server.arg(0);
      Serial.println(Order);
      
      if (Order.indexOf("download_")>=0)
      {
        Order.remove(0,9);
        SD_file_download(Order);
        Serial.println(Order);
      }
  
      if ((server.arg(0)).indexOf("delete_")>=0)
      {
        Order.remove(0,7);
        SD_file_delete(Order);
        Serial.println(Order);
      }
    }

  if (SD_present) 
  { 
    SendHTML_Header();    
    webpage += F("<table align='center'>");
    webpage += F("<tr><th>Name/Type</th><th style='width:20%'>Type File/Dir</th><th>File Size</th></tr>");
    printDirectory("/",0);
    webpage += F("</table>");
    SendHTML_Content();
  }
  else 
    {
      SendHTML_Header();
      webpage += F("<h3>No Files Found</h3>");
    }
    append_page_footer();
    SendHTML_Content();
    SendHTML_Stop();   //Stop is needed because no content length was sent
  } else ReportSDNotPresent();
}

//Upload a file to the SD
void File_Upload()
{
  append_page_header();
  webpage += F("<h3>Select File to Upload</h3>"); 
  webpage += F("<FORM action='/fupload' method='post' enctype='multipart/form-data'>");
  webpage += F("<input class='buttons' style='width:25%' type='file' name='fupload' id = 'fupload' value=''>");
  webpage += F("<button class='buttons' style='width:10%' type='submit'>Upload File</button><br><br>");
  webpage += F("<a href='/'>[Back]</a><br><br>");
  append_page_footer();
  server.send(200, "text/html",webpage);
}

void printDirectory(const char * dirname, uint8_t levels)
{
  SerialFlash.opendir();
  int i = 0;
  while(1){
    char filename[64];
    uint32_t filesize;
    if (webpage.length() > 1000) {
      SendHTML_Content();
    }

    if (SerialFlash.readdir(filename, sizeof(filename), filesize)) {
   
      webpage += "<tr><td>"+String(filename)+"</td>";
      webpage += "<td>File</td>";

      webpage += "<td>filesize</td>";
      webpage += "<td>";
      webpage += F("<FORM action='/' method='post'>"); 
      webpage += F("<button type='submit' name='download'"); 
      webpage += F("' value='"); webpage +="download_"+String(filename); webpage +=F("'>Download</button>");
      webpage += "</td>";
      webpage += "<td>";
      webpage += F("<FORM action='/' method='post'>"); 
      webpage += F("<button type='submit' name='delete'"); 
      webpage += F("' value='"); webpage +="delete_"+String(filename); webpage +=F("'>Delete</button>");
      webpage += "</td>";
      webpage += "</tr>";

      Serial.print(F("  "));
      Serial.print(filename);
      Serial.print(F("  "));
      Serial.print(filesize);
      Serial.print(F(" bytes"));
      Serial.println();
  }
  else {
      break; // no more files
    }

  }
 
}

void SD_file_download(String filename)
{
  const char * Filename = filename.c_str();
  if (SD_present) 
  { 
    SerialFlashFile ff = SerialFlash.open(Filename);  
    if (ff) 
    {
      server.sendHeader("Content-Type", "text/text");
      server.sendHeader("Content-Disposition", "attachment; filename="+filename);
      server.sendHeader("Connection", "close");
      // server.streamFile(ff, "application/octet-stream");
      ff.close();
    } else ReportFileNotPresent("download"); 
  } else ReportSDNotPresent();
}
SerialFlashFile UploadFile;
//Upload a new file to the Filing system
void handleFileUpload()
{ 
  HTTPUpload& uploadfile = server.upload(); 
  String filename;
  const char * Filename = filename.c_str();
  unsigned long length;
   
                                          
  if(uploadfile.status == UPLOAD_FILE_START)
  {
    filename = uploadfile.filename;
    if(!filename.startsWith("/")) filename = "/"+filename;
    Serial.print("Upload File Name: "); Serial.println(filename);
    SerialFlash.remove(Filename);
    SerialFlash.create(Filename, length);
    UploadFile = SerialFlash.open(Filename);  
    filename = String();
    length = uploadfile.currentSize;
  }
  else if (uploadfile.status == UPLOAD_FILE_WRITE)
  {
      if (UploadFile) {
        Serial.print(F("  copying"));
        // copy data loop
        unsigned long count = 0;
        unsigned char dotcount = 9;
        while (count < length) {
          // char buf[256];
          unsigned int n;
          n = uploadfile.currentSize;
          UploadFile.write(uploadfile.buf, uploadfile.currentSize);
          count = count + n;
          Serial.print(".");
          if (++dotcount > 100) {
             Serial.println();
             dotcount = 0;
          }
        }
        UploadFile.close();
        if (dotcount > 0) Serial.println();
      else {
        Serial.println(F("  error opening freshly created file!"));
        }
      }
     else {
      Serial.println(F("  unable to create file"));
    }
  }

  else if (uploadfile.status == UPLOAD_FILE_END)
  {
    if(UploadFile)          
    {                                    
      UploadFile.close();   
      Serial.print("Upload Size: "); Serial.println(uploadfile.totalSize);
      webpage = "";
      append_page_header();
      webpage += F("<h3>File was successfully uploaded</h3>"); 
      webpage += F("<h2>Uploaded File Name: "); webpage += uploadfile.filename+"</h2>";
      webpage += F("<h2>File Size: "); webpage += file_size(uploadfile.totalSize) + "</h2><br><br>"; 
      webpage += F("<a href='/'>[Back]</a><br><br>");
      append_page_footer();
      server.send(200,"text/html",webpage);
    } 
    else
    {
      ReportCouldNotCreateFile("upload");
    }
  } 
}

void SD_file_delete(String filename) 
{ 
  const char * Filename = filename.c_str();
  if (SD_present) { 
    SendHTML_Header();
    SerialFlashFile dataFile = SerialFlash.open(Filename); 
    if (dataFile)
    { 
      if (SerialFlash.remove(Filename)) {
        Serial.println(F("File deleted successfully"));
        webpage += "<h3>File '"+filename+"' has been erased</h3>"; 
        webpage += F("<a href='/'>[Back]</a><br><br>");
      }
      else
      { 
        webpage += F("<h3>File was not deleted - error</h3>");
        webpage += F("<a href='/'>[Back]</a><br><br>");
      }
    } else ReportFileNotPresent("delete");
    append_page_footer(); 
    SendHTML_Content();
    SendHTML_Stop();
  } else ReportSDNotPresent();
} 

//SendHTML_Header
void SendHTML_Header()
{
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate"); 
  server.sendHeader("Pragma", "no-cache"); 
  server.sendHeader("Expires", "-1"); 
  server.setContentLength(CONTENT_LENGTH_UNKNOWN); 
  server.send(200, "text/html", ""); //Empty content inhibits Content-length header so we have to close the socket ourselves. 
  append_page_header();
  server.sendContent(webpage);
  webpage = "";
}

//SendHTML_Content
void SendHTML_Content()
{
  server.sendContent(webpage);
  webpage = "";
}

//SendHTML_Stop
void SendHTML_Stop()
{
  server.sendContent("");
  server.client().stop(); //Stop is needed because no content length was sent
}

//ReportSDNotPresent
void ReportSDNotPresent()
{
  SendHTML_Header();
  webpage += F("<h3>No flash chip present</h3>"); 
  webpage += F("<a href='/'>[Back]</a><br><br>");
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}

//ReportFileNotPresent
void ReportFileNotPresent(String target)
{
  SendHTML_Header();
  webpage += F("<h3>File does not exist</h3>"); 
  webpage += F("<a href='/"); webpage += target + "'>[Back]</a><br><br>";
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}

//ReportCouldNotCreateFile
void ReportCouldNotCreateFile(String target)
{
  SendHTML_Header();
  webpage += F("<h3>Could Not Create Uploaded File (write-protected?)</h3>"); 
  webpage += F("<a href='/"); webpage += target + "'>[Back]</a><br><br>";
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}

//File size conversion
String file_size(int bytes)
{
  String fsize = "";
  if (bytes < 1024)                 fsize = String(bytes)+" B";
  else if(bytes < (1024*1024))      fsize = String(bytes/1024.0,3)+" KB";
  else if(bytes < (1024*1024*1024)) fsize = String(bytes/1024.0/1024.0,3)+" MB";
  else                              fsize = String(bytes/1024.0/1024.0/1024.0,3)+" GB";
  return fsize;
}
