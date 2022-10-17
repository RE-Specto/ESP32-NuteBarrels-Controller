//#include "main.h"
#include "WebServer.h"
#include "Expanders.h"
#include "SystemState.h"
#include "Barrels.h"
#include "FlowSensor.h"
#include "PresureSensor.h"
// #include "WebServer.h"
#include "Filesystem.h"
// #include "Modem.h"
// #include "NTPClient.h"
// #include "FMSD.h"
#include "globals.h"

AsyncWebServer server(80);
DNSServer dns;

/*-------- WebServer Begin ----------*/
//https://github.com/me-no-dev/ESPAsyncWebServer

void ServerClass::begin()
{

    //WiFiManager Local intialization. Once its business is done, there is no need to keep it around
    AsyncWiFiManager wifiManager(&server, &dns);
    //wifiManager.resetSettings();
    wifiManager.setDebugOutput(false);
    wifiManager.setConfigPortalTimeout(180);  // 3 minutes
    wifiManager.autoConnect("AutoConnectAP"); // will stop here if no wifi connected
    LOG.printf("ESSID: %s IP: %s\r\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());

    // Route for root / web page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        LOG.printf("Requested: %s\r\n", request->url().c_str());
        request->redirect("/manual"); //untill implemented
        /*
        if(!SD.exists("/index.html")){
            SD.end();
            LOG.println("trying to restart SD");
            if(!SD.begin(22)){
            LOG.println("unable to read form SD card");
            request->send(200, "text/html", "<html><body><center><h1>check SD Card please</h1></center></body></html>");
            }
            }
        request->send(SD, "/index2.html", String(), false);
        */
    });

    server.on("/index", HTTP_GET, [](AsyncWebServerRequest *request) {
        LOG.printf("Requested: %s\r\n", request->url().c_str());
        request->send(*Filesys.disk, "/index.html", String(), false);
    });

    server.on(
        "/upload", HTTP_POST, [](AsyncWebServerRequest *request) {}, [](AsyncWebServerRequest *request, String filename, size_t index, byte *data, size_t len, bool final) {
            if(!index)
            {
                LOG.printf("\r\nUpload Started: %s\r\n", filename.c_str());
                // open the file on first call and store the file handle in the request object
                request->_tempFile = Filesys.disk->open("/"+filename, "w");
            }
            if(len) 
            {
                LOG.printf("received chunk [from %u to %u]\r\n", index, len);
                // stream the incoming chunk to the opened file
                request->_tempFile.write(data,len);
            }       
            if(final)
            {
                LOG.printf("\r\nUpload Ended: %s, %u Bytes\r\n", filename.c_str(), index+len);
                request->_tempFile.close();
                request->redirect("/list");
                //request->send(200, "text/plain", "File Uploaded !");
            } });

    server.on("/list", HTTP_GET, [](AsyncWebServerRequest *request) {
        LOG.printf("Requested: %s\r\n", request->url().c_str());
        AsyncResponseStream *response = request->beginResponseStream("text/html");
        File dir = Filesys.disk->open("/");
        File file = dir.openNextFile();
        response->print("<html><body style=\"transform: scale(1.5);transform-origin: 0 0;\"><h3>file system</h3><ul>");
        while (file)
        {
            response->print("<li>");
            response->printf("<button onclick=\"location=\'/del?f=%s\'\">Delete</button><span>\t</span>", file.name());
            response->printf("<a href=\"down?f=%s\"><b>%s</b></a> \t%u bytes \t %li timestamp</li>", file.name(), file.name(), file.size(), file.getLastWrite());
            file.close();
            file = dir.openNextFile();
        }
        dir.close();
        response->print("</ul><form method='POST' action='/upload' enctype='multipart/form-data'>");
        response->print("<input type='file' name='update'><input type='submit' value='Upload'></form>");
        response->print("<button onclick=\"location=\'/backup\'\">Backup all to SPIFFS</button><span> </span>");
        response->print("<button onclick=\"location=\'/restore\'\">Restore missing to SD</button><br><br>");
        response->print("<button onclick=\"location=location\">reload</button><span> </span>");
        response->print("<button onclick=\"location=\'/reset\'\">reset</button><br><br>");
        response->print("<button onclick=\"location=\'/manual\'\">manual controls</button><span> </span>");
        response->print("<button onclick=\"location=\'/settings\'\">calibration</button><span> </span>");
        response->print("<button onclick=\"location=\'/fmsd\'\">manual fmsd</button><br>");
        if (Filesys.isSD())
        {
            response->print("<div>filesystem is: SD Card</div>");
            response->print("<button onclick=\"location=\'/switchFS\'\">Switch to SPIFFS</button><br><br>");
        }
        else
        {
            response->print("<div>filesystem is: SPIFFS</div>");
            response->print("<button onclick=\"location=\'/switchFS\'\">Switch to SD Card</button><br><br>");
        }
        response->print("</body></html>");
        request->send(response);
    });

    server.on("/switchFS", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (Filesys.isSD())
        {
            LOG.println(F("switching to SPIFFS"));
            Filesys.useSD(false);
        }
        else
        { // IMPORTANT !! add check for filesystem availability
            LOG.println(F("switching to SD Card"));
            Filesys.useSD(true);
        }
        request->redirect("/list");
    });

    server.on("/del", HTTP_GET, [](AsyncWebServerRequest *request) {
        LOG.printf("Requested: %s\r\n", request->url().c_str());
        if (request->args() > 0)
        { // Arguments were received
            if (request->hasArg("f"))
            {
                const char *file = request->arg("f").c_str();
                LOG.printf("Deleting file %s ", file);
                Filesys.disk->remove(file) ? Serial.println("Successfully") : Serial.println("Failed");
            }
        }
        else
        {
            LOG.println("*server: del received no args");
        }
        request->redirect("/list");
    });

    server.on("/down", HTTP_GET, [](AsyncWebServerRequest *request) {
        LOG.printf("Requested: %s\r\n", request->url().c_str());
        if (request->args() > 0)
        { // Arguments were received
            if (request->hasArg("f"))
            {
                const char *file = request->arg("f").c_str();
                LOG.printf("Downloading file %s \r\n", file);
                request->send(*Filesys.disk, file, "text/plain");
            }
        }
        else
            request->send(200, "text/plain", "file not found");
    });

    server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(404);
        //request->send(*Filesys.disk, "/favicon.png", "image/png");
    });

    server.on("/manual", HTTP_GET, [](AsyncWebServerRequest *request) {
        LOG.printf("Requested: %s\r\n", request->url().c_str());
        AsyncResponseStream *response = request->beginResponseStream("text/html");
        response->print("<html><body style=\"transform: scale(1.5);transform-origin: 0 0;\"><h3>manual control</h3><ul>");
        response->print("<span>Relays</span>");
        for (byte i = 0; i < 8; i++)
        {
            response->print("<li>");
            response->printf("<button onclick=\"location=\'/man?f=%u&o=%u\'\">fill  %u %s </button><span> </span>", i, Expanders.FillingRelayGet(i) ? 0 : 1, i, Expanders.FillingRelayGet(i) ? "-ON" : "OFF");
            response->printf("<button onclick=\"location=\'/man?s=%u&o=%u\'\">store %u %s </button><span> </span>", i, Expanders.StoringRelayGet(i) ? 0 : 1, i, Expanders.StoringRelayGet(i) ? "-ON" : "OFF");
            response->printf("<button onclick=\"location=\'/man?d=%u&o=%u\'\">drain %u %s </button><span> </span>", i, Expanders.DrainingRelayGet(i) ? 0 : 1, i, Expanders.DrainingRelayGet(i) ? "-ON" : "OFF");
            response->print("</li>");
        }
        response->print("<span>RGB LED</span>");
        response->print("<li>");
        for (byte i = 0; i < 8; i++)
            response->printf("<button onclick=\"location=\'/man?rgb=%u\'\">RGB %u</button><span> </span>", i, i);
        response->print("</li>");
        response->print("<span>Flow Sensors</span>");
        response->printf("<li>Fs1: [%up] [%uL] [%umL/s] [%upulse/L]</li>",
                         Flow.Counted(FRESHWATER),
                         Flow.Counted(FRESHWATER) / Flow.Divider(FRESHWATER),
                         Flow.Get(FRESHWATER),
                         Flow.Divider(FRESHWATER));
        response->printf("<li>Fs2: [%up] [%uL] [%umL/s] [%upulse/L]</li>",
                         Flow.Counted(NUTRIENTS),
                         Flow.Counted(NUTRIENTS) / Flow.Divider(NUTRIENTS),
                         Flow.Get(NUTRIENTS),
                         Flow.Divider(NUTRIENTS));
        response->print("<span>Pressure Sensors</span>");
        response->print("<li>");
        response->printf("<button onclick=\"location=\'/pressure?n=1\'\">PS1: measure</button><span> </span>");
        response->printf("[%iUnits] [%u raw/Units] [%i correction]</li>",
                         //Pressure.measure(FRESHWATER),
                         Pressure.LastValue(FRESHWATER),
                         Pressure.Divider(FRESHWATER),
                         Pressure.Offset(FRESHWATER));
        response->print("<li>");
        response->printf("<button onclick=\"location=\'/pressure?n=2\'\">PS2: measure</button><span> </span>");
        response->printf("[%iUnits] [%u raw/Units] [%i correction]</li>",
                         //Pressure.measure(NUTRIENTS),
                         Pressure.LastValue(NUTRIENTS),
                         Pressure.Divider(NUTRIENTS),
                         Pressure.Offset(NUTRIENTS));
        response->print("<span>Ultrasonic Sensors</span>");

        for (byte i = 0; i < NUM_OF_BARRELS; i++)
        {
            response->print("<li>");
            response->printf("<button onclick=\"location=\'/sonic?n=%u\'\">Sonic %u: measure</button><span> </span>", i, i);
            response->printf("[%iL] [%imm] [%uml/mm] [%umm barrel] [problem:%u] [water:%u] [nutri:%u]", Barrels.SonicCalcLiters(i), Barrels.SonicLastMM(i), Barrels.SonicMLinMMGet(i), Barrels.SonicOffset(i), Barrels.Errors(i), Barrels.FreshGet(i), Barrels.NutriGet(i));
            response->print("</li>");
        }
        response->printf("<li>Sonic liters: [total %i] [usable %i] </li>", Barrels.SonicLitersTotal(), Barrels.SonicLitersUsable());

        response->print("</ul>");
        response->print("<button onclick=\"location=location\">reload</button><span> </span>");
        response->print("<button onclick=\"location=\'/reset\'\">reset</button><span> </span>");
        response->print("<button onclick=\"location=\'/man?start\'\">start</button><span> </span>");
        response->print("<button onclick=\"location=\'/man?stop\'\">stop</button><span> </span>");
        response->print("<button onclick=\"location=\'/list\'\">list filesystem</button><span> </span>");
        response->print("<button onclick=\"location=\'/settings\'\">calibration</button><span> </span>");
        response->print("<button onclick=\"location=\'/fmsd\'\">manual fmsd</button><br>");
        response->printf("<span>uptime: %lli seconds. system state:%u</span><br><br>", esp_timer_get_time() / 1000000, State.Get());
        response->print("</body></html>");
        request->send(response);
    });

    server.on("/fmsd", HTTP_GET, [](AsyncWebServerRequest *request) {
        LOG.printf("Requested: %s\r\n", request->url().c_str());
        byte status = 0; // for parameter out of range error
        if (request->hasArg("task"))
        {
            // converting arguments
            int task = request->arg("task").toInt();
            int ammo = request->arg("ammo").toInt();
            int src = request->arg("src").toInt();
            int dest = request->arg("dest").toInt();
            // printing message
            LOG.printf("[man]fmsd task received: task%i ammo%i src%i dest%i\r\n", task, ammo, src, dest);
            // checking input is valid
            if (src > -1 && src < NUM_OF_BARRELS && task > -1 && task < 6 && ammo > -1 && ammo < 32768 && dest > -1 && dest < NUM_OF_BARRELS)
            {
                // stopping auto
                State.Set(STOPPED_STATE);
                // running the task
                State.SetManual(task, src, dest, ammo);
                status = 1;
            }
            else
            {
                LOG.println("[E] parameter out of range");
                status = 2;
            }
        }
        AsyncResponseStream *response = request->beginResponseStream("text/html");
        response->print("<html><body style=\"transform: scale(1.5);transform-origin: 0 0;\"><h3>manual F.M.S.D.B.</h3><ul>");
        response->print("<form action=\"/fmsd\">");
        response->print("<li>fill-1 mix-2 store-3 drain-4 bypass-5</li>");
        response->printf("<li><input id=\"task\" name=\"task\" value=\"%u\"></li>", State.ManualTask());
        response->print("<li>how much</li>");
        response->printf("<li><input id=\"ammo\" name=\"ammo\" value=\"%u\"></li>", State.ManualAmmount());
        response->print("<li>from</li>");
        response->printf("<li><input id=\"src\" name=\"src\" value=\"%u\"></li>", State.ManualSource());
        response->print("<li>to</li>");
        response->printf("<li><input id=\"dest\" name=\"dest\" value=\"%u\"></li>", State.ManualDestination());
        switch (status)
        {
            case 0:
            response->print("<li>insert data and press Go</li>");
            break;
            case 1:
            response->print("<li>ok, data received</li>");
            break;
            case 2:
            response->print("<li>out of range data, try again!</li>");
            break;
        }
        response->print("<li><input type=\"submit\" value=\"Go\"><span> </span><button type=\"reset\" onclick=\"location=\'/fmsd?task=0\'\">cancel fmsd</button>");
        response->print("</form>");
        response->print("</ul>");
        response->print("<button onclick=\"location=\'/list\'\">list filesystem</button><span> </span>");
        response->print("<button onclick=\"location=\'/settings\'\">calibration</button><span> </span>");
        response->print("<button onclick=\"location=\'/manual\'\">manual controls</button><br>");
        response->printf("<span>uptime: %lli seconds. system state:%u</span><br><br>", esp_timer_get_time() / 1000000, State.Get());
        response->print("</body></html>");
        request->send(response);
    });

    server.on("/settings", HTTP_GET, [](AsyncWebServerRequest *request) {
        LOG.printf("Requested: %s\r\n", request->url().c_str());

        if (request->hasArg("Override"))
        {
            int Override = request->arg("Override").toInt();
            State.Override(Override);
        }

        if (request->hasArg("OverrideError"))
        {
            int OverrideError = request->arg("OverrideError").toInt();
            State.OverrideError(OverrideError);
        }

        if (request->hasArg("SetFillBarrel"))
        {
            int  SetFillBarrel = request->arg("SetFillBarrel").toInt();
            State.SetFillBarrel(SetFillBarrel);
        }

        if (request->hasArg("SetFillReq"))
        {
            int SetFillReq = request->arg("SetFillReq").toInt();
            State.SetFillReq(SetFillReq);
        }

        if (request->hasArg("SetMixReq"))
        {
            int SetMixReq = request->arg("SetMixReq").toInt();
            State.SetMixReq(SetMixReq);
        }

        if (request->hasArg("MixReset"))
        {
            State.MixReset();
        }

        if (request->hasArg("SetStoreBarrel"))
        {
            int SetStoreBarrel = request->arg("SetStoreBarrel").toInt();
            State.SetStoreBarrel(SetStoreBarrel);
        }

        if (request->hasArg("SetDrainReq"))
        {
            int SetDrainReq = request->arg("SetDrainReq").toInt();
            State.SetDrainReq(SetDrainReq);
        }

        if (request->hasArg("SetBypassReq"))
        {
            int SetBypassReq = request->arg("SetBypassReq").toInt();
            State.SetBypassReq(SetBypassReq);
        }

        if (request->hasArg("ErrorOverride"))
        {
            int ErrorOverride = request->arg("ErrorOverride").toInt();
            int ErrorOverrideData = request->arg("ErrorOverrideData").toInt();
            Barrels.ErrorOverride(ErrorOverride, ErrorOverrideData);
        }

        if (request->hasArg("ErrorReset"))
        {
            int ErrorReset = request->arg("ErrorReset").toInt();
            Barrels.Reset(ErrorReset);
        }

        if (request->hasArg("VolumeMaxSet"))
        {
            int VolumeMaxSet = request->arg("VolumeMaxSet").toInt();
            int VolumeMaxSetData = request->arg("VolumeMaxSetData").toInt();
            Barrels.VolumeMaxSet(VolumeMaxSet, VolumeMaxSetData);
        }

        if (request->hasArg("VolumeMinSet"))
        {
            int VolumeMinSet = request->arg("VolumeMinSet").toInt();
            int VolumeMinSetData = request->arg("VolumeMinSetData").toInt();
            Barrels.VolumeMinSet(VolumeMinSet, VolumeMinSetData);
        }

        if (request->hasArg("SonicOffsetSet"))
        {
            int SonicOffsetSet = request->arg("SonicOffsetSet").toInt();
            int SonicOffsetSetData = request->arg("SonicOffsetSetData").toInt();
            Barrels.SonicOffsetSet(SonicOffsetSet, SonicOffsetSetData);
        }

        if (request->hasArg("SonicMLinMMSet"))
        {
            int SonicMLinMMSet = request->arg("SonicMLinMMSet").toInt();
            int SonicMLinMMSetData = request->arg("SonicMLinMMSetData").toInt();
            Barrels.SonicMLinMMSet(SonicMLinMMSet, SonicMLinMMSetData);
        }

        if (request->hasArg("DividerSet"))
        {
            int DividerSet = request->arg("DividerSet").toInt();
            int DividerSetData = request->arg("DividerSetData").toInt();
            Flow.DividerSet(DividerSet, DividerSetData);
        }

        if (request->hasArg("FErrorReset"))
        {
            int FErrorReset = request->arg("FErrorReset").toInt();
            Flow.Reset(FErrorReset);
        }

        if (request->hasArg("FEnable"))
        {
            int FEnable = request->arg("FEnable").toInt();
            Flow.Enable(FEnable);
        }

        if (request->hasArg("FDisable"))
        {
            int FDisable = request->arg("FDisable").toInt();
            Flow.Disable(FDisable);
        }

        if (request->hasArg("PDividerSet"))
        {
            int PDividerSet = request->arg("PDividerSet").toInt();
            int PDividerSetData = request->arg("PDividerSetData").toInt();
            Pressure.DividerSet(PDividerSet, PDividerSetData);
        }

        if (request->hasArg("OffsetSet"))
        {
            int OffsetSet = request->arg("OffsetSet").toInt();
            int OffsetSetData = request->arg("OffsetSetData").toInt();
            Pressure.OffsetSet(OffsetSet, OffsetSetData);
        }

        if (request->hasArg("MinSet"))
        {
            int MinSet = request->arg("MinSet").toInt();
            int MinSetData = request->arg("MinSetData").toInt();
            Pressure.MinSet(MinSet, MinSetData);
        }

        if (request->hasArg("MaxSet"))
        {
            int MaxSet = request->arg("MaxSet").toInt();
            int MaxSetData = request->arg("MaxSetData").toInt();
            Pressure.MaxSet(MaxSet, MaxSetData);
        }

        if (request->hasArg("PErrorReset"))
        {
            int PErrorReset = request->arg("PErrorReset").toInt();
            Pressure.Reset(PErrorReset);
        }

        if (request->hasArg("Enable"))
        {
            int Enable = request->arg("Enable").toInt();
            Pressure.Enable(Enable);
        }

        if (request->hasArg("Disable"))
        {
            int Disable = request->arg("Disable").toInt();
            Pressure.Disable(Disable);
        }

        if (request->hasArg("DryBarrel"))
        {
            int DryBarrel = request->arg("DryBarrel").toInt();
            Barrels.SonicMeasure(DryBarrel);
            if(Barrels.Errors(DryBarrel))
            {
                LOG.printf("Dry point set: Barrel:%u at error state:%u, unable to measure dry point\r\n", DryBarrel, Barrels.Errors(DryBarrel));
            }
            else
            {
                LOG.printf("Setting Dry barrel:%u point, from:%u to:%u\r\n", DryBarrel, Barrels.SonicOffset(DryBarrel), Barrels.SonicLastMM(DryBarrel));
                Barrels.SonicOffsetSet(DryBarrel, Barrels.SonicLastMM(DryBarrel));
            }
        }

        if (request->hasArg("EmptyBarrel"))
        {
            int EmptyBarrel = request->arg("EmptyBarrel").toInt();
            Barrels.SonicMeasure(EmptyBarrel);
            if(Barrels.Errors(EmptyBarrel))
            {
                LOG.printf("Empty point set: Barrel:%u at error state:%u, unable to measure Empty point\r\n", EmptyBarrel, Barrels.Errors(EmptyBarrel));
            }
            else
            {
                LOG.printf("Setting empty barrel:%u point, from:%u to:%u\r\n", EmptyBarrel, Barrels.VolumeMin(EmptyBarrel), Barrels.SonicLastMM(EmptyBarrel));
                Barrels.VolumeMinSet(EmptyBarrel, Barrels.SonicLastMM(EmptyBarrel));
            }
        }

        if (request->hasArg("FullBarrel"))
        {
            int FullBarrel = request->arg("FullBarrel").toInt();
            Barrels.SonicMeasure(FullBarrel);
            if(Barrels.Errors(FullBarrel))
            {
                LOG.printf("Full point set: Barrel:%u at error state:%u, unable to measure Full point\r\n", FullBarrel, Barrels.Errors(FullBarrel));
            }
            else
            {
                LOG.printf("Setting Full barrel:%u point, from:%u to:%u\r\n", FullBarrel, Barrels.VolumeMax(FullBarrel), Barrels.SonicLastMM(FullBarrel));
                Barrels.VolumeMaxSet(FullBarrel, Barrels.SonicLastMM(FullBarrel));
            }
        }

        if (request->hasArg("Save100LitersMark"))
        {
            int Save100LitersMark = request->arg("Save100LitersMark").toInt();
            Barrels.SonicMeasure(Save100LitersMark);
            if(Barrels.Errors(Save100LitersMark))
            {
                LOG.printf("100L point set: Barrel:%u at error state:%u, unable to measure 100L point\r\n", Save100LitersMark, Barrels.Errors(Save100LitersMark));
            }
            else
            {
                uint16_t CoeffBefore = Barrels.SonicMLinMMGet(Save100LitersMark);
                Barrels.Save100LitersMark(Save100LitersMark);
                LOG.printf("Setting 100L barrel:%u point, from:%u to:%u\r\n", Save100LitersMark, CoeffBefore, Barrels.SonicMLinMMGet(Save100LitersMark));
            }         
        }

        AsyncResponseStream *response = request->beginResponseStream("text/html");
        response->print("<html><body style=\"transform: scale(1.5);transform-origin: 0 0;\"><h3>Settings and Calibrations</h3><style> form{margin:1;display:inline-block;} input:not([type]){width:63;} li{margin-top:4;}</style><ul>");

        response->print("<li>override system state</li>");
        response->print("<form action=\"/settings\">");
        response->printf("<input id=\"Override\" name=\"Override\" value=\"%u\"><span> </span>", State.Get());
        response->print("<input type=\"submit\" value=\"Go\"></form>");

        response->print("<li>override error state</li>");
        response->print("<form action=\"/settings\">");
        response->printf("<input id=\"OverrideError\" name=\"OverrideError\" value=\"%u\"><span> </span>", State.Errors());
        response->print("<input type=\"submit\" value=\"Go\"></form>");

        response->print("<li>barrel to fill and mix</li>");
        response->print("<form action=\"/settings\">");
        response->printf("<input id=\"SetFillBarrel\" name=\"SetFillBarrel\" value=\"%u\"><span> </span>", State.FillBarrel());
        response->print("<input type=\"submit\" value=\"Go\"></form>");

        response->print("<li>next barrel to store to</li>");
        response->print("<form action=\"/settings\">");
        response->printf("<input id=\"SetStoreBarrel\" name=\"SetStoreBarrel\" value=\"%u\"><span> </span>", State.StoreBarrel());
        response->print("<input type=\"submit\" value=\"Go\"></form>");

        response->print("<li>fill requirement</li>");
        response->print("<form action=\"/settings\">");
        response->printf("<input id=\"SetFillReq\" name=\"SetFillReq\" value=\"%u\"><span> </span>", State.FillRequirement());
        response->print("<input type=\"submit\" value=\"Go\"></form>");

        response->print("<li>mix requirement, mix timer reset</li>");
        response->print("<form action=\"/settings\">");
        response->printf("<input id=\"SetMixReq\" name=\"SetMixReq\" value=\"%u\"><span> </span>", State.MixRequirement());
        response->print("<input type=\"submit\" value=\"Go\"></form>");

        response->print("<form action=\"/settings\">");
        response->print("<input type=\"hidden\" id=\"MixReset\" name=\"MixReset\" value=\"1\">");
        response->printf("<input type=\"submit\" value=\"Time left:%u, Reset\"></form>", State.MixTimer());        

        response->print("<li>drain requirement</li>");
        response->print("<form action=\"/settings\">");
        response->printf("<input id=\"SetDrainReq\" name=\"SetDrainReq\" value=\"%u\"><span> </span>", State.DrainMore());
        response->print("<input type=\"submit\" value=\"Go\"></form>");

        response->print("<li>Bypass requirement</li>");
        response->print("<form action=\"/settings\">");
        response->printf("<input id=\"SetBypassReq\" name=\"SetBypassReq\" value=\"%u\"><span> </span>", State.BypassMore());
        response->print("<input type=\"submit\" value=\"Go\"></form>");

        response->print("<li>barrels: Errors override.. Dry barrel lenght in mm point, mililitrage to lenght ratio, Full barrel point in liters, . . . . . Empty barrel point in liters</li>");
        for (byte x=0;x<NUM_OF_BARRELS;x++)
        {
            response->printf("<span>Barrel %u </span>", x);

            response->print("<form action=\"/settings\">");
            response->printf("<input type=\"hidden\" id=\"ErrorOverride\" name=\"ErrorOverride\" value=\"%u\"><span> </span>", x);
            response->printf("<input id=\"ErrorOverrideData\" name=\"ErrorOverrideData\" value=\"%u\"><span> </span>", Barrels.Errors(x));
            response->print("<input type=\"submit\" value=\"Go\"></form>");

            response->print("<form action=\"/settings\">");
            response->printf("<input type=\"hidden\" id=\"DryBarrel\" name=\"DryBarrel\" value=\"%u\"><span> </span>", x);
            response->print("<input type=\"submit\" value=\"Set Dry Point\"></form>");

            response->print("<form action=\"/settings\">");
            response->printf("<input type=\"hidden\" id=\"SonicOffsetSet\" name=\"SonicOffsetSet\" value=\"%u\"><span> </span>", x);
            response->printf("<input id=\"SonicOffsetSetData\" name=\"SonicOffsetSetData\" value=\"%u\"><span> </span>", Barrels.SonicOffset(x));
            response->print("<input type=\"submit\" value=\"Go\"></form>");

            response->print("<form action=\"/settings\">");
            response->printf("<input type=\"hidden\" id=\"Save100LitersMark\" name=\"Save100LitersMark\" value=\"%u\"><span> </span>", x);
            response->print("<input type=\"submit\" value=\"Set 100L\"></form>");

            response->print("<form action=\"/settings\">");
            response->printf("<input type=\"hidden\" id=\"SonicMLinMMSet\" name=\"SonicMLinMMSet\" value=\"%u\"><span> </span>", x);
            response->printf("<input id=\"SonicMLinMMSetData\" name=\"SonicMLinMMSetData\" value=\"%u\"><span> </span>", Barrels.SonicMLinMMGet(x));
            response->print("<input type=\"submit\" value=\"Go\"></form>");

            response->print("<form action=\"/settings\">");
            response->printf("<input type=\"hidden\" id=\"FullBarrel\" name=\"FullBarrel\" value=\"%u\"><span> </span>", x);
            response->print("<input type=\"submit\" value=\"Set Full point\"></form>");

            response->print("<form action=\"/settings\">");
            response->printf("<input type=\"hidden\" id=\"VolumeMaxSet\" name=\"VolumeMaxSet\" value=\"%u\"><span> </span>", x);
            response->printf("<input id=\"VolumeMaxSetData\" name=\"VolumeMaxSetData\" value=\"%u\"><span> </span>", Barrels.VolumeMax(x));
            response->print("<input type=\"submit\" value=\"Go\"></form>");

            response->print("<form action=\"/settings\">");
            response->printf("<input type=\"hidden\" id=\"EmptyBarrel\" name=\"EmptyBarrel\" value=\"%u\"><span> </span>", x);
            response->print("<input type=\"submit\" value=\"Set Empty point\"></form>");

            response->print("<form action=\"/settings\">");
            response->printf("<input type=\"hidden\" id=\"VolumeMinSet\" name=\"VolumeMinSet\" value=\"%u\"><span> </span>", x);
            response->printf("<input id=\"VolumeMinSetData\" name=\"VolumeMinSetData\" value=\"%u\"><span> </span>", Barrels.VolumeMin(x));
            response->print("<input type=\"submit\" value=\"Go\"></form>");

            response->print("<br>");
        }

        response->print("<li>Sensors: . . flow Divider, . pressure Divider, pressure Offset, Min pressure, . Max pressure, . Pressure Sensors, Flow Sensors</li>");
        for (byte x=1;x<=2;x++)
        {
            response->printf("<span>%s </span>", x==1 ? "Freshwater" : "Nutrients .. ");

            response->print("<form action=\"/settings\">");
            response->printf("<input type=\"hidden\" id=\"DividerSet\" name=\"DividerSet\" value=\"%u\"><span> </span>", x);
            response->printf("<input id=\"DividerSetData\" name=\"DividerSetData\" value=\"%u\"><span> </span>", Flow.Divider(x));
            response->print("<input type=\"submit\" value=\"Go\"></form>");

            response->print("<form action=\"/settings\">");
            response->printf("<input type=\"hidden\" id=\"PDividerSet\" name=\"PDividerSet\" value=\"%u\"><span> </span>", x);
            response->printf("<input id=\"PDividerSetData\" name=\"PDividerSetData\" value=\"%u\"><span> </span>", Pressure.Divider(x));
            response->print("<input type=\"submit\" value=\"Go\"></form>");

            response->print("<form action=\"/settings\">");
            response->printf("<input type=\"hidden\" id=\"OffsetSet\" name=\"OffsetSet\" value=\"%u\"><span> </span>", x);
            response->printf("<input id=\"OffsetSetData\" name=\"OffsetSetData\" value=\"%u\"><span> </span>", Pressure.Offset(x));
            response->print("<input type=\"submit\" value=\"Go\"></form>");

            response->print("<form action=\"/settings\">");
            response->printf("<input type=\"hidden\" id=\"MinSet\" name=\"MinSet\" value=\"%u\"><span> </span>", x);
            response->printf("<input id=\"MinSetData\" name=\"MinSetData\" value=\"%u\"><span> </span>", Pressure.Min(x));
            response->print("<input type=\"submit\" value=\"Go\"></form>");

            response->print("<form action=\"/settings\">");
            response->printf("<input type=\"hidden\" id=\"MaxSet\" name=\"MaxSet\" value=\"%u\"><span> </span>", x);
            response->printf("<input id=\"MaxSetData\" name=\"MaxSetData\" value=\"%u\"><span> </span>", Pressure.Max(x));
            response->print("<input type=\"submit\" value=\"Go\"></form>");

            response->print("<form action=\"/settings\">");
            response->printf("<input type=\"hidden\" id=\"PErrorReset\" name=\"PErrorReset\" value=\"%u\"><span> </span>", x);
            response->print("<input type=\"submit\" value=\"Reset\"></form>");

            if (Pressure.Enabled(x))
            {
                response->print("<form action=\"/settings\">");
                response->printf("<input type=\"hidden\" id=\"Disable\" name=\"Disable\" value=\"%u\"><span> </span>", x);
                response->print("<input type=\"submit\" value=\"Disable\"></form>");
            }
            else
            {
                response->print("<form action=\"/settings\">");
                response->printf("<input type=\"hidden\" id=\"Enable\" name=\"Enable\" value=\"%u\"><span> </span>", x);
                response->print("<input type=\"submit\" value=\"Enable\"></form>");
            }

            response->print("<form action=\"/settings\">");
            response->printf("<input type=\"hidden\" id=\"FErrorReset\" name=\"FErrorReset\" value=\"%u\"><span> </span>", x);
            response->print("<input type=\"submit\" value=\"Reset\"></form>");

            if (Flow.Enabled(x))
            {
                response->print("<form action=\"/settings\">");
                response->printf("<input type=\"hidden\" id=\"FDisable\" name=\"FDisable\" value=\"%u\"><span> </span>", x);
                response->print("<input type=\"submit\" value=\"Disable\"></form>");
            }
            else
            {
                response->print("<form action=\"/settings\">");
                response->printf("<input type=\"hidden\" id=\"FEnable\" name=\"FEnable\" value=\"%u\"><span> </span>", x);
                response->print("<input type=\"submit\" value=\"Enable\"></form>");
            }
            response->print("<br>");
        }
        response->print("</ul>");
        response->print("<button onclick=\"location=\'settings\'\">reload</button><span> </span>");
        response->print("<button onclick=\"location=\'/reset\'\">reset</button><span> </span>");
        response->print("<button onclick=\"location=\'/list\'\">list filesystem</button><span> </span>");
        response->print("<button onclick=\"location=\'/manual\'\">manual controls</button><span> </span>");
        response->print("<button onclick=\"location=\'/fmsd\'\">manual fmsd</button><br>");
        response->printf("<span>uptime: %lli seconds. system state:%u</span><br><br>", esp_timer_get_time() / 1000000, State.Get());
        response->print("</body></html>");
        request->send(response);
    });


    server.on("/man", HTTP_GET, [](AsyncWebServerRequest *request) {
        LOG.printf("Requested: %s\r\n", request->url().c_str());
        if (request->args() > 0)
        { // Arguments were received
            if (request->hasArg("f"))
            {
                int f = request->arg("f").toInt();
                int o = request->arg("o").toInt();
                LOG.printf("setting FillingRelay %i to %i\r\n", f, o);
                Expanders.FillingRelay(f, o);
            }
            if (request->hasArg("s"))
            {
                int s = request->arg("s").toInt();
                int o = request->arg("o").toInt();
                LOG.printf("setting StoringRelay %i to %i\r\n", s, o);
                Expanders.StoringRelay(s, o);
            }
            if (request->hasArg("d"))
            {
                int d = request->arg("d").toInt();
                int o = request->arg("o").toInt();
                LOG.printf("setting DrainingRelay %i to %i\r\n", d, o);
                Expanders.DrainingRelay(d, o);
            }
            if (request->hasArg("rgb"))
            {
                int rgb = request->arg("rgb").toInt();
                LOG.printf("setting setRGBLED to %i\r\n", rgb);
                Expanders.setRGBLED(rgb);
            }
            if (request->hasArg("start"))
            {
                State.Unset(STOPPED_STATE);
                State.ResetManual();
            }
            if (request->hasArg("stop"))
            {
                State.Set(STOPPED_STATE);
                State.ResetManual();
            }
        }
        else
            request->send(200, "text/plain", "no args!");
        request->redirect("/manual");
    });

    server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request) {
        LOG.printf("Requested: %s\r\n", request->url().c_str());
        request->redirect("/manual");
        vTaskDelay(100 / portTICK_PERIOD_MS); // to prevent reset before redirect
        ESP.restart();
    });

    server.on("/backup", HTTP_GET, [](AsyncWebServerRequest *request) {
        LOG.printf("Requested: %s\r\n", request->url().c_str());
        //char buf [4];
        //sprintf (buf, "%03u", Filesys.Backup());
        //request->send(200, "text/html", buf);
        Filesys.Backup();
        request->redirect("/list");
    });

    server.on("/restore", HTTP_GET, [](AsyncWebServerRequest *request) {
        LOG.printf("Requested: %s\r\n", request->url().c_str());
        //char buf [4];
        //sprintf (buf, "%03u",  Filesys.Restore());
        //request->send(200, "text/html", buf);
        Filesys.Restore();
        request->redirect("/list");
    });

    server.on("/sonic", HTTP_GET, [](AsyncWebServerRequest *request) {
        LOG.printf("Requested: %s\r\n", request->url().c_str());
        if (request->args() > 0)
        { // Arguments were received
            if (request->hasArg("n"))
            {
                //char buf [6];
                //sprintf (buf, "%05u", barrels.SonicMeasure(request->arg("n").toInt()));
                //request->send(200, "text/html", buf);
                Barrels.SonicMeasure(request->arg("n").toInt(), 3, 500); // measure 3 times max 500 ms
                request->redirect("/manual");
            }
        }
        else
            request->send(200, "text/plain", "n parameter missing");
    });

    server.on("/pressure", HTTP_GET, [](AsyncWebServerRequest *request) {
        LOG.printf("Requested: %s\r\n", request->url().c_str());
        if (request->args() > 0)
        { // Arguments were received
            if (request->hasArg("n"))
            {
                Pressure.measure(request->arg("n").toInt());
                request->redirect("/manual");
            }
        }
        else
            request->send(200, "text/plain", "n parameter missing");
    });

    server.on("/mdns", HTTP_GET, [](AsyncWebServerRequest *request) {
        int mdns = mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
        char buf[6];
        sprintf(buf, "mdns:%u", mdns);
        request->send(200, "text/plain", buf);
    });

    server.onNotFound([](AsyncWebServerRequest *request) {
        LOG.printf("Requested: %s\r\n", request->url().c_str());
        AsyncResponseStream *response = request->beginResponseStream("text/html");
        response->addHeader("Server", "ESP Async Web Server");
        response->printf("<!DOCTYPE html><html><head><title>Webpage at %s</title></head><body>", request->url().c_str());

        response->print("<h2>Hello ");
        response->print(request->client()->remoteIP());
        response->print("</h2>");

        response->print("<h3>General</h3>");
        response->print("<ul>");
        response->printf("<li>Version: HTTP/1.%u</li>", request->version());
        response->printf("<li>Method: %s</li>", request->methodToString());
        response->printf("<li>URL: %s</li>", request->url().c_str());
        response->printf("<li>Host: %s</li>", request->host().c_str());
        response->printf("<li>ContentType: %s</li>", request->contentType().c_str());
        response->printf("<li>ContentLength: %u</li>", request->contentLength());
        response->printf("<li>Multipart: %s</li>", request->multipart() ? "true" : "false");
        response->print("</ul>");

        response->print("<h3>Headers</h3>");
        response->print("<ul>");
        int headers = request->headers();
        for (int i = 0; i < headers; i++)
        {
            AsyncWebHeader *h = request->getHeader(i);
            response->printf("<li>%s: %s</li>", h->name().c_str(), h->value().c_str());
        }
        response->print("</ul>");

        response->print("<h3>Parameters</h3>");
        response->print("<ul>");
        int params = request->params();
        for (int i = 0; i < params; i++)
        {
            AsyncWebParameter *p = request->getParam(i);
            if (p->isFile())
                response->printf("<li>FILE[%s]: %s, size: %u</li>", p->name().c_str(), p->value().c_str(), p->size());
            else if (p->isPost())
                response->printf("<li>POST[%s]: %s</li>", p->name().c_str(), p->value().c_str());
            else
                response->printf("<li>GET[%s]: %s</li>", p->name().c_str(), p->value().c_str());
        }
        response->print("</ul>");

        response->print("</body></html>");
        //send the response last
        request->send(response);
    });

    // Start server
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
    server.begin();
    #ifdef DEBUG_NET
    LOG.print(F("-Server init\r\n"));
    #endif
}
/*-------- WebServer END ----------*/
ServerClass WebServer;
