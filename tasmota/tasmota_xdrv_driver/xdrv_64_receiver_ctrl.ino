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

struct receiver_ctrl_softc_s {
    TasmotaSerial   *sc_serial;
    enum receiver_ctrl_serial_state_e   sc_serial_state;
    uint8_t init_retries;
    uint16_t timeout_to_do;
    LinkedList<uint8_t> current_data;
    unsigned long last_packet;
} __packed;


static struct receiver_ctrl_softc_s *receiver_ctrl_sc = nullptr;

static int8_t char_to_num(uint8_t c) {
    if (c >= 0x30 && c <= 0x39) {
        return c - 0x30;
    }

    if (c >= 0x41 && c <= 0x46) {
        return c - 55;
    }

    return 0;
}

static uint32_t ascii_to_num(LinkedList<uint8_t> *data,uint32_t start,uint32_t len) {
    uint32_t res = 0;

    for (int i = 0; i < len; i++) {
        uint32_t curr = char_to_num(data->get(start + i));
        if (i > 0) {
            res << 4;
        }
        res += curr;
    }

    return res;
}

static void receiver_ctrl_pre_init(void) {

    
    AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Pre Init"));

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
free:
    free(sc);
}

static void receiver_ctrl_do_init(void) {
    if (receiver_ctrl_sc == nullptr) {
        // Pre Init not done
        return;
    }

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
        return;
    }

    receiver_ctrl_sc->init_retries++;

    send_init_command();

    AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Send init command"));
    receiver_ctrl_sc->last_packet = millis();

    AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Recorded last packet"));

    receiver_ctrl_sc->sc_serial_state = RECEIVER_CTRL_SERIAL_INIT;

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
            AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Serial available"));
            while(sc->sc_serial->available()) {
                int data = sc->sc_serial->read();
                AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Got serial data %02x"), data);
                sc->current_data.add(data);
            }

        } else if (sc->sc_serial_state == RECEIVER_CTRL_SERIAL_INIT) {
            AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Got no serial data"));
            unsigned long now = millis();

            if (sc->last_packet < now - 50000) {
                // Received last packet more than 5 secounds ago
                // Enter NOT_INIT state
                AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Got no packet for 50 seconds"));
                sc->sc_serial_state = RECEIVER_CTRL_SERIAL_NOT_INIT;
            }
        } else {
            //AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Got no serial data"));
        }
    }

    return;
}

static void parseConfiguration() {

    uint8_t start = receiver_ctrl_sc->current_data.shift();

    if (start != 0x12) {
        AddLog(LOG_LEVEL_INFO,PSTR(RECEIVER_CTRL_LOGNAME ": PArsing config paket"));
        // Invalid start do nothing
        return ;
    }

    char typ[5];

    typ[0] = receiver_ctrl_sc->current_data.shift();
    typ[1] = receiver_ctrl_sc->current_data.shift();
    typ[2] = receiver_ctrl_sc->current_data.shift();
    typ[3] = receiver_ctrl_sc->current_data.shift();
    typ[4] = receiver_ctrl_sc->current_data.shift();

    AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Got type"));
    AddLog(LOG_LEVEL_INFO, typ);
}

static void parseReport() {
    uint8_t start = receiver_ctrl_sc->current_data.shift();

    if (start != 0x02) {
        // Ibvalid start
        return ;
    }
}

static bool parsePaket() {

    if (receiver_ctrl_sc->current_data.size()) {

        uint8_t curr_start = receiver_ctrl_sc->current_data.get(0);
        while (curr_start != 0x12 && curr_start != 0x02 && receiver_ctrl_sc->current_data.size()) {

            receiver_ctrl_sc->current_data.shift();
            curr_start = receiver_ctrl_sc->current_data.get(0);

        }

        if (!receiver_ctrl_sc->current_data.size()) {
            // No paket start found
            AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Parse Paket got no start"));
            return false;
        }

        AddLog(LOG_LEVEL_INFO,PSTR(RECEIVER_CTRL_LOGNAME ": Got paket with start %02x"), curr_start);

        bool end_found = false;
        for (int i = 0; i < receiver_ctrl_sc->current_data.size() && !end_found; i++) {
            if (receiver_ctrl_sc->current_data.get(i) == 0x03) {
                end_found = true;
            }
        }

        if (end_found) {
            AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Got paket end"));
            switch (curr_start) {
                case 0x12:
                    parseConfiguration();
                    MqttShowSensor(false);
                    break;
                case 0x02:
                    parseReport();
                    MqttShowSensor(false);
                    break;
            }
        } else {
            AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Got no paket end"));
        }
    }

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

    switch (function) {
        case FUNC_LOOP:
            receiver_ctrl_loop(sc);
            break;
        case FUNC_EVERY_100_MSECOND:
            parsePaket();
            break;
        case FUNC_EVERY_SECOND:
            AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Every second"));
            if (sc->sc_serial_state == RECEIVER_CTRL_SERIAL_TIMEOUT) {
                AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Handle Timeout"));
                handle_timeout();
            } else if (sc->sc_serial_state == RECEIVER_CTRL_SERIAL_NOT_INIT || sc->sc_serial_state == RECEIVER_CTRL_SERIAL_WAITING_FOR_CONFIG) {
                AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Handle do init"));
                receiver_ctrl_do_init();
            } else {
                AddLog(LOG_LEVEL_INFO, PSTR(RECEIVER_CTRL_LOGNAME ": Unknown state"));
            }
            break;
    }

    return (false);
}

#endif