#ifdef USE_RECEIVER_CTRL

#define XDRV_63     63

#if !defined(USE_RECEIVER_CTRL_MODEL_YAMAHA_2500)
#define USE_RECEIVER_CTRL_MODEL_YAMAHA_2500
#endif

#ifdef USE_RECEIVER_CTRL_MODEL_YAMAHA_2500
#define RECEIVER_CTRL_SERIAL_BAUDRATE 9600
#define PROJECTOR_CTRL_LOGNAME	"RCV[Y2500]"
#endif

enum receiver_ctrl_serial_state_e : uint8_t {
    RECEIVER_CTRL_SERIAL_NOT_INIT=0,
    RECEIVER_CTRL_SERIAL_WAITING_FOR_CONFIG,
    RECEIVER_CTRL_SERIAL_INIT,
    RECEIVER_CTRL_SERIAL_TIMEOUT
};

struct receiver_ctrl_softc_s {
    TasmotaSerial   *sc_serial,
    enum receiver_ctrl_serial_state_e   sc_serial_state;
    uint8_t init_retries,
    uint16_t timeout_to_do
} __packed;

static struct receiver_ctrl_softc_s *receiver_ctrl_sc = nullptr;


static void rreceiver_ctrl_pre_init(void) {
    struct receiver_ctrl_softc_s *sc;
    int baudrate = RECEIVER_CTRL_SERIAL_BAUDRATE;

    if (!PinUsed(GPIO_RECEIVER_CTRL_TX) || !PinUsed(GPIO_RECEIVER_CTRL_RX)) {
        return ;
    }

    sc = (struct receiver_ctrl_softc_s *)malloc(sizeof(*sc));
    if (sc == NULL) {
        AddLog(LOG_LEVEL_ERROR, PSTR(RECEIVER_CTRL_LOGNAME ": unable to allocate state"));
        return;
    }

    memset(sc, 0, sizeof(*sc));

    sc->sc_serial = new TasmotaSerial(Pin(GPIO_RECEIVER_CTRL_RX), Pin(GPIO_RECEIVER_CTRL_TX), 2);

    if (!sc->sc_serial->begin(baudrate)) {
        AddLog(LOG_LEVEL_ERROR, PSTR(RECEIVER_CTRL_LOGNAME ": unable to begin serial (baudrate %d)"), baudrate);
        goto del;
    }

    if (sc->sc_serial_>hardwareSerial()) {
        ClaimSerial();
        SetSerial(baudrate, TS_SERIAL_8N1);
    }

    receiver_ctrl_sc = sc;

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
        receiver_ctrl_sc->timeout_to_do = 300;
        return;
    }

    receiver_ctrl_sc->init_retries++;

    send_init_command();

    return;
}

static void send_init_command(void) {

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
        receiver_ctrl_sc->init_retries = 0;
        receiver_ctrl_sc->sc_serial_state = RECEIVER_CTRL_SERIAL_NOT_INIT;
        return;
    }

    receiver_ctrl_sc->timeout_to_do--;

    return;
}

static void receiver_ctrl_loop(receiver_ctrl_soft_c *sc) {
    
}

bool Xdrv63(uint8_t function) {

    struct receiver_ctrl_soft_c *sc;

    sc = receiver_ctrl_sc;

    switch (function) {
        case FUNC_PRE_INIT:
            receiver_ctrl_pre_init();
            return (false);
    }

    if (sc == nullptr) {
        return (false);
    }

    switch (function) {
        case FUNC_LOOP:
            receiver_ctrl_loop(sc);
            break;
        case FUNC_EVERY_SECOND:
            if (sc->sc_serial_state == RECEIVER_CTRL_SERIAL_TIMEOUT) {
                handle_timeout();
            } else if (sc->sc_serial_state == RECEIVER_CTRL_SERIAL_NOT_INIT || sc->sc_serial_state == RECEIVER_CTRL_SERIAL_WAITING_FOR_CONFIG) {
                receiver_ctrl_do_init();
            } else {

            }
            break;
    }

    return (false);
}

#endif