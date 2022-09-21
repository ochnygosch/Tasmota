#ifdef USE_RECEIVER_CTRL

#include <LinkedList.h>

#define XDRV_64   64

#if !defined(USE_RECEIVER_CTRL_MODEL_YAMAHA_2500)
#define USE_RECEIVER_CTRL_MODEL_YAMAHA_2500
#endif

#ifdef USE_RECEIVER_CTRL_MODEL_YAMAHA_2500
#define RECEIVER_CTRL_SERIAL_BAUDRATE 9600
#define RECEIVER_CTRL_LOGNAME	"RCV[Y2500]"
#endif


enum receiver_ctrl_serial_state_e : uint8_t {
    RECEIVER_CTRL_SERIAL_NOT_INIT=0,
    RECEIVER_CTRL_SERIAL_WAITING_FOR_CONFIG,
    RECEIVER_CTRL_SERIAL_INIT,
    RECEIVER_CTRL_SERIAL_TIMEOUT
};

enum receiver_ctrl_system_state_e : uint8_t {
    RECEIVER_CTRL_SYSTEM_STATE_OK=0,
    RECEIVER_CTRL_SYSTEM_STATE_BUSY,
    RECEIVER_CTRL_SYSTEM_STATE_STANDBY
};

enum receiver_ctrl_system_input_e: uint8_t {
    RECEIVER_CTRL_SYSTEM_INPUT_PHONO=0,
    RECEIVER_CTRL_SYSTEM_INPUT_CD,
    RECEIVER_CTRL_SYSTEM_INPUT_TUNER,
    RECEIVER_CTRL_SYSTEM_INPUT_CDR,
    RECEIVER_CTRL_SYSTEM_INPUT_MD_TAPE,
    RECEIVER_CTRL_SYSTEM_INPUT_DVD,
    RECEIVER_CTRL_SYSTEM_INPUT_DTV,
    RECEIVER_CTRL_SYSTEM_INPUT_CBL_SAT,
    RECEIVER_CTRL_SYSTEM_INPUT_SAT,
    RECEIVER_CTRL_SYSTEM_INPUT_VCR1,
    RECEIVER_CTRL_SYSTEM_INPUT_DVR_VCR2,
    RECEIVER_CTRL_SYSTEM_INPUT_DVR_VCR3,
    RECEIVER_CTRL_SYSTEM_INPUT_VAUX_DOCK,
    RECEIVER_CTRL_SYSTEM_INPUT_NET_USB,
    RECEIVER_CTRL_SYSTEM_INPUT_XM,
    RECEIVER_CTRL_SYSTEM_INPUT_MULTI_CH,
};

struct receiver_ctrl_softc_s {
    TasmotaSerial   *sc_serial;
    enum receiver_ctrl_serial_state_e   sc_serial_state;
    uint8_t init_retries;
    uint16_t timeout_to_do;
    LinkedList<uint8_t> *current_data;
    unsigned long last_packet;

    char model[6];
    char version;
    enum receiver_ctrl_system_state_e system_state;
    bool power_main;
    bool power_zone_2;
    bool power_zone_3;
    enum receiver_ctrl_system_input_e input_main;
} __packed;


static struct receiver_ctrl_softc_s *receiver_ctrl_sc = nullptr;

static void receiver_ctrl_update_mqtt(receiver_ctrl_softc_s *sc,bool send);
static void send_system_command(receiver_ctrl_softc_s *sc,uint8_t cmd0, uint8_t cmd1, uint8_t dat0, uint8_t dat1);
static void send_operation_command(receiver_ctrl_softc_s *sc,uint8_t cmd0, uint8_t cmd1, uint8_t cmd2, uint8_t cmd3);

static uint8_t char_to_num(uint8_t c) {
    if (c >= 0x30 && c <= 0x39) {
        return c - 0x30;
    }

    if (c >= 0x41 && c <= 0x46) {
        return c - 55;
    }

    return 0;
}

static uint8_t num_to_char(uint8_t n) {
    if (n < 10) {
        return n + 0x30;
    }

    if (n > 9 && n < 16) {
        return n - 10 + 0x41;
    }

    return 0;
}

static uint32_t ascii_to_num(LinkedList<uint8_t> *data,uint32_t start,uint32_t len) {
    uint32_t res = 0;

    for (int i = 0; i < len; i++) {
        uint32_t curr = char_to_num(data->get(start + i));
        if (i > 0) {
            res = res << 4;
        }
        res += curr;
    }

    return res;
}

static void receiver_ctrl_pre_init(void) {

    
    //AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Pre Init"));

    struct receiver_ctrl_softc_s *sc;
    int baudrate = RECEIVER_CTRL_SERIAL_BAUDRATE;

    if (!PinUsed(GPIO_RECEIVER_CTRL_TX) || !PinUsed(GPIO_RECEIVER_CTRL_RX)) {
        return ;
    }

    AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": pins configured"));

    sc = (struct receiver_ctrl_softc_s *)malloc(sizeof(*sc));
    if (sc == NULL) {
        AddLog(LOG_LEVEL_ERROR, PSTR(RECEIVER_CTRL_LOGNAME ": unable to allocate state"));
        return;
    }

    AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": State allocated"));
    memset(sc, 0, sizeof(*sc));

    sc->current_data = new LinkedList<uint8_t>();

    sc->sc_serial_state == RECEIVER_CTRL_SERIAL_NOT_INIT;

    AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Creating Serial"));
    sc->sc_serial = new TasmotaSerial(Pin(GPIO_RECEIVER_CTRL_RX), Pin(GPIO_RECEIVER_CTRL_TX), 2);

    if (!sc->sc_serial->begin(baudrate)) {
        AddLog(LOG_LEVEL_ERROR, PSTR(RECEIVER_CTRL_LOGNAME ": unable to begin serial (baudrate %d)"), baudrate);
        goto del;
    }

    AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Serial started"));

    if (sc->sc_serial->hardwareSerial()) {
        ClaimSerial();
        SetSerial(baudrate, TS_SERIAL_8N1);
    }

    receiver_ctrl_sc = sc;

    return; 
del:
    delete sc->sc_serial;
    delete sc->current_data;
free:
    free(sc);
}

static void receiver_ctrl_do_init(void) {
    if (receiver_ctrl_sc == nullptr) {
        // Pre Init not done
        return;
    }

    //AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Do init %d"), receiver_ctrl_sc->sc_serial_state);

    if (receiver_ctrl_sc->sc_serial_state == RECEIVER_CTRL_SERIAL_TIMEOUT) {
        // In init timeout
        return;
    }

    if (receiver_ctrl_sc->sc_serial_state == RECEIVER_CTRL_SERIAL_INIT) {
        // Init already done
        return;
    }

    if (receiver_ctrl_sc->init_retries > 4) {
        // Max retries reached... timeout
        AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Config max retries reached"));
        receiver_ctrl_sc->sc_serial_state = RECEIVER_CTRL_SERIAL_TIMEOUT;
        receiver_ctrl_sc->timeout_to_do = 10;
        receiver_ctrl_update_mqtt(receiver_ctrl_sc, true);
        return;
    }

    receiver_ctrl_sc->init_retries++;

    send_init_command();

    AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Send init command"));
    receiver_ctrl_sc->last_packet = millis();

    AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Recorded last packet"));

    receiver_ctrl_sc->sc_serial_state = RECEIVER_CTRL_SERIAL_WAITING_FOR_CONFIG;

    return;
}

static void send_init_command(void) {

    // TODO
    // Send init command 
    // Enter state waiting for config

    AddLog(LOG_LEVEL_INFO,PSTR(RECEIVER_CTRL_LOGNAME ": Sending init command"));

    receiver_ctrl_sc->sc_serial->write((uint8_t)0x11);
    receiver_ctrl_sc->sc_serial->flush();
    receiver_ctrl_sc->sc_serial->write((uint8_t)0x00);
    receiver_ctrl_sc->sc_serial->flush();
    receiver_ctrl_sc->sc_serial->write((uint8_t)0x00);
    receiver_ctrl_sc->sc_serial->flush();
    receiver_ctrl_sc->sc_serial->write((uint8_t)0x01);
    receiver_ctrl_sc->sc_serial->flush();
    receiver_ctrl_sc->sc_serial->write((uint8_t)0x03);
    receiver_ctrl_sc->sc_serial->flush();

    receiver_ctrl_sc->sc_serial_state = RECEIVER_CTRL_SERIAL_WAITING_FOR_CONFIG;
}

static void send_operation_command(receiver_ctrl_softc_s *sc,uint8_t cmd0, uint8_t cmd1, uint8_t cmd2, uint8_t cmd3) {
    sc->sc_serial->write(0x02);
    sc->sc_serial->write(0x30);
    sc->sc_serial->write(num_to_char(cmd0));
    sc->sc_serial->write(num_to_char(cmd1));
    sc->sc_serial->write(num_to_char(cmd2));
    sc->sc_serial->write(num_to_char(cmd3));
    sc->sc_serial->write(0x03);
    sc->sc_serial->flush();
}

static void send_system_command(receiver_ctrl_softc_s *sc,uint8_t cmd0, uint8_t cmd1, uint8_t dat0, uint8_t dat1) {
    sc->sc_serial->write(0x02);
    sc->sc_serial->write(0x32);
    sc->sc_serial->write(num_to_char(cmd0));
    sc->sc_serial->write(num_to_char(cmd1));
    sc->sc_serial->write(num_to_char(dat0));
    sc->sc_serial->write(num_to_char(dat1));
    sc->sc_serial->write(0x03);
    sc->sc_serial->flush();
}



static void handle_timeout(void) {

    if (receiver_ctrl_sc == nullptr) {
        // Pre init not done
        return;
    }

    if (receiver_ctrl_sc->sc_serial_state != RECEIVER_CTRL_SERIAL_TIMEOUT) {
        // Not in timeout
        return;
    }

    if (receiver_ctrl_sc->timeout_to_do < 1) {
        // Timeout done... retry init
        AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Timeout done retrying init"));
        receiver_ctrl_sc->init_retries = 0;
        receiver_ctrl_sc->sc_serial_state = RECEIVER_CTRL_SERIAL_NOT_INIT;
        return;
    }

    receiver_ctrl_sc->timeout_to_do--;

    return;
}

static void receiver_ctrl_loop(struct receiver_ctrl_softc_s *sc) {

    if (sc != NULL) {
        //AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": In loop"));
        if (sc->sc_serial->available()) {
            //AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Serial available"));
            while(sc->sc_serial->available()) {
                int data = sc->sc_serial->read();
                //AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Got serial data %02x"), data);
                sc->current_data->add(data);
            }

        } else if (sc->sc_serial_state == RECEIVER_CTRL_SERIAL_INIT) {
            //AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Got no serial data"));
            unsigned long now = millis() - sc->last_packet;

            if (now > 300000) {
                // Received last packet more than 300 secounds ago
                // Enter NOT_INIT state
                AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Got no packet for 300 seconds %d"), now);
                sc->sc_serial_state = RECEIVER_CTRL_SERIAL_NOT_INIT;
                sc->init_retries = 0;
            }
        } else {
            //AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Got no serial data2"));
        }
    }

    return;
}

static void parseConfiguration() {

    uint8_t start = receiver_ctrl_sc->current_data->shift();

    if (start != 0x12) {
        AddLog(LOG_LEVEL_INFO,PSTR(RECEIVER_CTRL_LOGNAME ": Parsing config paket"));
        // Invalid start do nothing
        return ;
    }

    receiver_ctrl_softc_s *r = receiver_ctrl_sc;

    r->model[0] = r->current_data->shift();
    r->model[1] = r->current_data->shift();
    r->model[2] = r->current_data->shift();
    r->model[3] = r->current_data->shift();
    r->model[4] = r->current_data->shift();
    r->model[5] = 0;

    r->version = r->current_data->shift();

    AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Got type %s"), r->model);

    uint32_t length = ascii_to_num(r->current_data,0,2);

    char len[3];
    len[0] = r->current_data->shift();
    len[1] = r->current_data->shift();
    len[2] = 0;

    AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Got length %d from %s"), length, len);

    int currDt = -1;

    if (length > 6) {
        currDt++;
        r->current_data->shift(); // DT0 Fixed Baud Rate
        currDt++;
        r->current_data->shift(); // DT1 Fixed Receive Buffer
        currDt++;
        r->current_data->shift(); // DT2 Fixed Receive Buffer
        currDt++;
        r->current_data->shift(); // DT3 Fixed Command Timeout
        currDt++;
        r->current_data->shift(); // DT4 Fixed Command Timeout
        currDt++;
        r->current_data->shift(); // DT5 Fixed Command Timeout
        currDt++;
        r->current_data->shift(); // DT6 Fixed Handshaking
    }

    if (length > 7) {
        currDt++;
        uint8_t dat = r->current_data->shift();
        dat = char_to_num(dat);
        AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": system %d"), dat);
        switch (dat) {
            case 0:
                r->system_state = RECEIVER_CTRL_SYSTEM_STATE_OK;
                break;
            case 1:
                r->system_state = RECEIVER_CTRL_SYSTEM_STATE_BUSY;
                break;
            case 2:
                r->system_state = RECEIVER_CTRL_SYSTEM_STATE_STANDBY;
                break;
        }
    }

    if (length > 8) {
        currDt++;
        uint8_t dat = r->current_data->shift();
        dat = char_to_num(dat);
        AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": power %d"), dat);
        switch (dat) {
            case 0:
                r->power_main = false;
                r->power_zone_2 = false;
                r->power_zone_3 = false;
                break;
            case 1:
                r->power_main = true;
                r->power_zone_2 = true;
                r->power_zone_3 = true;
                break;
            case 2:
                r->power_main = true;
                r->power_zone_2 = false;
                r->power_zone_3 = false;
                break;
            case 3:
                r->power_main = false;
                r->power_zone_2 = true;
                r->power_zone_3 = true;
                break;
            case 4:
                r->power_main = true;
                r->power_zone_2 = true;
                r->power_zone_3 = false;
                break;
            case 5:
                r->power_main = true;
                r->power_zone_2 = false;
                r->power_zone_3 = true;
                break;
            case 6:
                r->power_main = false;
                r->power_zone_2 = true;
                r->power_zone_3 = false;
                break;
            case 7:
                r->power_main = false;
                r->power_zone_2 = false;
                r->power_zone_3 = true;
                break;
        }
    }

    for (int i = currDt + 1; i < length; i++) {
        if (r->current_data->size()) {
            uint8_t dat = r->current_data->shift();
            AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Got DT%d: %02x"), i, dat);
        } else {
            AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": DT%d: none"), i);
        }
    }
    

    

    if (r->current_data->size()) {
    uint8_t curr = r->current_data->shift();

    AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Got %d additional bytes"), r->current_data->size());

    while (curr != 0x03 && r->current_data->size() > 0) {
        AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Discarding %02x"), curr);
        if (r->current_data->size() > 0) {
            //AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Not Empty"));
            curr = r->current_data->shift();
        } else {
            //AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Is Empty"));
        }
    }

    }
    AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Hier2"));

}

static void parseReport() {

    uint8_t start = receiver_ctrl_sc->current_data->shift();

    if (receiver_ctrl_sc->current_data->size() < 7) {
        AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Got too small Report"));
        return ;
    }

    

    if (start != 0x02) {
        // Ibvalid start
        return ;
    }

    uint8_t typ = receiver_ctrl_sc->current_data->shift();
    uint8_t grd = receiver_ctrl_sc->current_data->shift();

    uint32_t cmd = ascii_to_num(receiver_ctrl_sc->current_data,0,2);

    uint8_t cmd0 = receiver_ctrl_sc->current_data->shift();
    uint8_t cmd1 = receiver_ctrl_sc->current_data->shift();

    uint8_t dat0 = receiver_ctrl_sc->current_data->shift();
    uint8_t dat1 = receiver_ctrl_sc->current_data->shift();

    AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Report Typ %02x Grd %02x Cmd %02x Dat %02x %02x"), typ, grd, cmd, dat0, dat1);

    uint8_t end = receiver_ctrl_sc->current_data->shift();
}


static bool parsePaket() {
    //AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Parse Paket %d"), receiver_ctrl_sc->current_data->size());
    if (receiver_ctrl_sc->current_data->size()) {

        uint8_t curr_start = receiver_ctrl_sc->current_data->get(0);
        //AddLog(LOG_LEVEL_INFO,PSTR(RECEIVER_CTRL_LOGNAME ": Testing start %02x"), curr_start);
        while (curr_start != 0x12 && curr_start != 0x02 && receiver_ctrl_sc->current_data->size()) {
            //AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Testing start pre shift %d"), receiver_ctrl_sc->current_data->size());
            //AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Pre shift %d"), receiver_ctrl_sc->current_data->size());
            receiver_ctrl_sc->current_data->shift();
            //AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": After shift %d"), receiver_ctrl_sc->current_data->size());
        
            //AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Testing start got more %d"), receiver_ctrl_sc->current_data->size());
            if (receiver_ctrl_sc->current_data->size()) {
                curr_start = receiver_ctrl_sc->current_data->get(0);
                //AddLog(LOG_LEVEL_INFO,PSTR(RECEIVER_CTRL_LOGNAME ": Testing start again %02x"), curr_start);
            } else {
                //AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Testing start got no more"));
            }

        }

        if (!receiver_ctrl_sc->current_data->size()) {
            // No paket start found
            //AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Parse Paket got no start"));
            return false;
        }

        //AddLog(LOG_LEVEL_INFO,PSTR(RECEIVER_CTRL_LOGNAME ": Got paket with start %02x"), curr_start);

        bool end_found = false;
        for (int i = 0; i < receiver_ctrl_sc->current_data->size() && !end_found; i++) {
            if (receiver_ctrl_sc->current_data->get(i) == 0x03) {
                end_found = true;
            }
        }

        if (end_found) {
            //AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Got paket end"));
            switch (curr_start) {
                case 0x12:
                    parseConfiguration();
                    receiver_ctrl_sc->sc_serial_state = RECEIVER_CTRL_SERIAL_INIT;
                    receiver_ctrl_sc->last_packet = millis();
                    AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Got parseConfig return"));
                    //MqttShowSensor(false);
                    receiver_ctrl_update_mqtt(receiver_ctrl_sc, true);
                    break;
                case 0x02:
                    parseReport();
                    receiver_ctrl_sc->last_packet = millis();
                    receiver_ctrl_update_mqtt(receiver_ctrl_sc, true);
                    //MqttShowSensor(false);
                    break;
            }
        } else {
            AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Got no paket end"));
        }
    }

    //AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Return in parseReport"));
    return false;
}





void ReceiverJsonShow() {
    if (receiver_ctrl_sc == NULL) {
        ResponseAppend_P(PSTR(",\"Receiver\":{"));
        ResponseAppend_P(PSTR("\"Status\":-1"));
        ResponseJsonEnd();
    } else {
        ResponseAppend_P(PSTR(",\"Receiver\":{"));
        ResponseAppend_P(PSTR("\"Status\":1"));
        ResponseJsonEnd();
    }
}

static void receiver_ctrl_update_mqtt(receiver_ctrl_softc_s *sc,bool send) {
    Response_P(PSTR("{\"RCV\":{"));
    ResponseAppend_P(PSTR("\"SerialState\": %d"), sc->sc_serial_state);
    if (sc->sc_serial_state == RECEIVER_CTRL_SERIAL_INIT) {
        ResponseAppend_P(PSTR(",\"Model\":\"%s\""), sc->model);
        ResponseAppend_P(PSTR(",\"Version\":\"%c\""), sc->version);
        ResponseAppend_P(PSTR(",\"SystemState\": %d"), sc->system_state);
        ResponseAppend_P(PSTR(",\"PowerMain\": %d"), sc->power_main);
        ResponseAppend_P(PSTR(",\"PowerZone2\": %d"), sc->power_zone_2);
        ResponseAppend_P(PSTR(",\"PowerZone3\": %d"), sc->power_zone_3);
    }
    ResponseJsonEnd();
    ResponseJsonEnd();
    if (send) {
        MqttPublishPrefixTopicRulesProcess_P(RESULT_OR_STAT, PSTR("RCV"));
    }
}


bool receiver_ctrl_command(void) {

    uint8_t paramcount = 0;
    bool serviced = true;
    if (XdrvMailbox.data_len > 0) {
        paramcount = 1;
    } else {
        return false;
    }

    char argument[XdrvMailbox.data_len];
    for (uint32_t ca=0;ca<XdrvMailbox.data_len;ca++) {
        if ((' ' == XdrvMailbox.data[ca]) || ('=' == XdrvMailbox.data[ca])) { XdrvMailbox.data[ca] = ','; }
        if (',' == XdrvMailbox.data[ca]) { paramcount++; }
    }

    UpperCase(XdrvMailbox.data,XdrvMailbox.data);

    if (!strcmp(ArgV(argument, 1), "STATUS")) {
        AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Got status command"));
        receiver_ctrl_update_mqtt(receiver_ctrl_sc, false);
        return serviced;
    }

    if (!strcmp(ArgV(argument,1), "POWER")) {
        if (paramcount > 2) {
            if (!strcmp(ArgV(argument, 2), "1")) {
                if (!strcmp(ArgV(argument, 3), "ON")) {
                    AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Got main power on command"));
                } else {
                    AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Got main power off command"));
                }
            } else if (!strcmp(ArgV(argument, 2), "2")) {
                if (!strcmp(ArgV(argument, 3), "ON")) {
                    AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Got zone 2 power on command"));
                } else {
                    AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Got zone 2 power off command"));
                }
            } else if (!strcmp(ArgV(argument, 2), "3")) {
                if (!strcmp(ArgV(argument, 3), "ON")) {
                    AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Got zone 3 power on command"));
                } else {
                    AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Got zone 3 power off command"));   
                }
            } else {
                AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Got unknown power command"));
            }
            
            //send_operation_command(receiver_ctrl_sc, 0x7, 0xA, 0x1, 0xE);
        } else if (paramcount > 1) {
            if (!strcmp(ArgV(argument, 2), "ON")) {
                AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Got power on ALL command"));
                send_operation_command(receiver_ctrl_sc, 0x7, 0xA, 0x1, 0xE);
            } else {
                AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Got power off ALL command"));
                send_operation_command(receiver_ctrl_sc, 0x7, 0xA, 0x1, 0xD);
            }
        }
        receiver_ctrl_update_mqtt(receiver_ctrl_sc, false);
        return serviced;
    }

    return serviced;
}

bool Xdrv64(uint8_t function) {

    struct receiver_ctrl_softc_s *sc;

    sc = receiver_ctrl_sc;

    switch (function) {
        case FUNC_PRE_INIT:
            receiver_ctrl_pre_init();
            receiver_ctrl_do_init();
            return (false);
        case FUNC_JSON_APPEND:
            ReceiverJsonShow();
            return (false);
    }

    if (sc == nullptr) {
        return (false);
    }

    bool result = false;

    switch (function) {
        case FUNC_LOOP:
            //AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Every loop %d"), sc->sc_serial_state);
            receiver_ctrl_loop(sc);
            break;
        case FUNC_EVERY_250_MSECOND:
            //AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Every 250 msecond %d"), sc->sc_serial_state);
            parsePaket();
            break;
        case FUNC_EVERY_SECOND:
            //AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Every second %d"), sc->sc_serial_state);
            if (sc->sc_serial_state == RECEIVER_CTRL_SERIAL_TIMEOUT) {
                AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Handle Timeout"));
                handle_timeout();
            } else if (sc->sc_serial_state == RECEIVER_CTRL_SERIAL_NOT_INIT || sc->sc_serial_state == RECEIVER_CTRL_SERIAL_WAITING_FOR_CONFIG) {
                AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Handle do init"));
                receiver_ctrl_do_init();
            } else if (sc->sc_serial_state == RECEIVER_CTRL_SERIAL_INIT) {

            } else {
                AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Unknown state %d"), sc->sc_serial_state);
            }
            break;
        case FUNC_COMMAND_DRIVER:
            if (XDRV_64 == XdrvMailbox.index) {
                result = receiver_ctrl_command();
            }
            break;
    }

    return result;
}

#endif