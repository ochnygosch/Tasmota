#ifdef USE_PLAYER_CTRL

#include <LinkedList.h>

#define XDRV_67 67

#ifdef USE_PLAYER_CTRL_OPPO_BDP_103
#define PLAYER_CTRL_SERIAL_BAUDRATE 9600
#define PLAYER_CTRL_LOGNAME "PLYR[O103]"
#endif

enum player_ctrl_serial_state_e: uint8_t {
    PLAYER_CTRL_SERIAL_NOT_INIT=0,
    PLAYER_CTRL_SERIAL_WAIT_FOR_INIT,
    PLAYER_CTRL_SERIAL_INIT,
    PLAYER_CTRL_SERIAL_TIMEOUT
};

enum player_ctrl_power_state_e: uint8_t {
    PLAYER_CTRL_POWER_STATE_OFF=0,
    PLAYER_CTRL_POWER_STATE_ON
};

struct player_ctrl_softc_s {
    TasmotaSerial *serial;
    enum player_ctrl_serial_state_e serial_state;
    uint8_t init_retries;
    uint16_t timeout_to_do;
    LinkedList<uint8_t> *current_data;
    unsigned long last_paket;

    enum player_ctrl_power_state_e power_state;
} __packed;

static struct player_ctrl_softc_s *player_ctrl_state = nullptr;

static void player_ctrl_pre_init(void);
static void player_ctrl_do_init(struct player_ctrl_softc_s *st);
static void player_ctrl_json_append(void);
static void player_ctrl_update_mqtt(struct player_ctrl_softc_s *st, bool send);
static void player_ctrl_handle_timeout(struct player_ctrl_softc_s *st);
static void player_ctrl_parse_paket(struct player_ctrl_softc_s * st);
static void player_ctrl_loop(struct player_ctrl_softc_s *st);
static bool player_ctrl_command(void);
static void player_ctrl_send_init_command(struct player_ctrl_softc_s *st);
static void player_ctrl_send_power_command(bool on, struct player_ctrl_softc_s *st);
static void player_ctrl_send_eject_toggle_command(struct player_ctrl_softc_s *st);
static void player_ctrl_send_play_command(struct player_ctrl_softc_s *st);
static void player_ctrl_send_pause_command(struct player_ctrl_softc_s *st);
static void player_ctrl_send_std_command(byte cmd1, byte cmd2, byte cmd3, struct player_ctrl_softc_s *st);
static void player_ctrl_set_verbose_mode(struct player_ctrl_softc_s *st);
static bool player_ctrl_compare_cmd(char *cmd1, char *cmd2);

bool Xdrv67(uint8_t function) {
    struct player_ctrl_softc_s *st;

    st = player_ctrl_state;

    switch (function) {
        case FUNC_PRE_INIT:
            player_ctrl_pre_init();
            return false;
        case FUNC_JSON_APPEND:
            player_ctrl_json_append();
            return false;
    }

    if (st == nullptr) {
        return false;
    }

    bool result = false;

    switch (function) {
        case FUNC_LOOP:
            player_ctrl_loop(st);
            break;
        case FUNC_EVERY_100_MSECOND:
            player_ctrl_parse_paket(st);
            break;
        case FUNC_EVERY_SECOND:
            if (st->serial_state == PLAYER_CTRL_SERIAL_TIMEOUT) {
                player_ctrl_handle_timeout(st);
            } else if (st->serial_state == PLAYER_CTRL_SERIAL_NOT_INIT) {
                player_ctrl_do_init(st);
            }
            break;
        case FUNC_COMMAND_DRIVER:
            if (XDRV_67 == XdrvMailbox.index) {
                result = player_ctrl_command();
            }
    }

    return result;
}

static void player_ctrl_pre_init(void) {

    #ifdef DEBUG_PLAYER_CTRL
    AddLog(LOG_LEVEL_DEBUG, PSTR(PLAYER_CTRL_LOGNAME ": PreInit"));
    #endif

    struct player_ctrl_softc_s *st;

    if (!PinUsed(GPIO_PLAYER_CTRL_TX) || !PinUsed(GPIO_PLAYER_CTRL_RX)) {
        #ifdef DEBUG_PLAYER_CTRL
            AddLog(LOG_LEVEL_DEBUG, PSTR(PLAYER_CTRL_LOGNAME ": Pins not configured"));
        #endif
        return ;
    }

    #ifdef DEBUG_PLAYER_CTRL
        AddLog(LOG_LEVEL_DEBUG, PSTR(PLAYER_CTRL_LOGNAME ": Pins configured"));
    #endif

    st = (struct player_ctrl_softc_s *)malloc(sizeof(*st));
    if (st == nullptr) {
        AddLog(LOG_LEVEL_ERROR, PSTR(PLAYER_CTRL_LOGNAME ": Could not allocate player state"));
        return ;
    }

    #ifdef DEBUG_PLAYER_CTRL
        AddLog(LOG_LEVEL_DEBUG, PSTR(PLAYER_CTRL_LOGNAME ": State allocated"));
    #endif

    memset(st, 0, sizeof(*st));

    st->current_data = new LinkedList<uint8_t>();
    st->serial_state = PLAYER_CTRL_SERIAL_NOT_INIT;

    st->serial = new TasmotaSerial(Pin(GPIO_PLAYER_CTRL_RX), Pin(GPIO_PLAYER_CTRL_TX), 2);

    if (!st->serial->begin(PLAYER_CTRL_SERIAL_BAUDRATE)) {
        AddLog(LOG_LEVEL_ERROR, PSTR(PLAYER_CTRL_LOGNAME ": Could not start serial"));
        goto del;
    }

    #ifdef DEBUG_PLAYER_CTRL
        AddLog(LOG_LEVEL_DEBUG, PSTR(PLAYER_CTRL_LOGNAME ": Serial started"));
    #endif

    if (st->serial->hardwareSerial()) {
        ClaimSerial();
        SetSerial(PLAYER_CTRL_SERIAL_BAUDRATE, TS_SERIAL_8N1);
    }

    player_ctrl_state = st;
    return ;

del:
    delete st->serial;
    delete st->current_data;

    free(st);

    return ;
}

static void player_ctrl_do_init(struct player_ctrl_softc_s *st) {

    

    if (st == nullptr) {
        return ;
    }

    if (st->serial_state == PLAYER_CTRL_SERIAL_TIMEOUT) {
        return ;
    }

    if (st->serial_state == PLAYER_CTRL_SERIAL_INIT) {
        return ;
    }

    #ifdef DEBUG_PLAYER_CTRL
        AddLog(LOG_LEVEL_INFO, PSTR(PLAYER_CTRL_LOGNAME ": Do init"));
    #endif

    if (st->init_retries > 4) {
        #ifdef DEBUG_PLAYER_CTRL
            AddLog(LOG_LEVEL_DEBUG, PSTR(PLAYER_CTRL_LOGNAME ": Init Max retries reached"));
        #endif
        st->serial_state = PLAYER_CTRL_SERIAL_TIMEOUT;
        st->timeout_to_do = 60;
        player_ctrl_update_mqtt(st, true);
        return ;
    }

    st->init_retries++;

    #ifdef DEBUG_PLAYER_CTRL
        AddLog(LOG_LEVEL_DEBUG, PSTR(PLAYER_CTRL_LOGNAME ": Sending init"));
    #endif

    player_ctrl_send_init_command(st);

    st->last_paket = millis();

    st->serial_state = PLAYER_CTRL_SERIAL_WAIT_FOR_INIT;

    return ;
} 

static void player_ctrl_json_append(void) {

    #ifdef DEBUG_PLAYER_CTRL
        AddLog(LOG_LEVEL_DEBUG, PSTR(PLAYER_CTRL_LOGNAME ": Json Append"));
    #endif
    
    if (player_ctrl_state == nullptr) {
        ResponseAppend_P(PSTR(",\"Player\":{\"Status\":-1}"));
    } else {
        ResponseAppend_P(PSTR(",\"Player\":{\"Status\":1}"));
    }
}

static void player_ctrl_update_mqtt(struct player_ctrl_softc_s *st, bool send) {
    
    #ifdef DEBUG_PLAYER_CTRL
        AddLog(LOG_LEVEL_DEBUG, PSTR(PLAYER_CTRL_LOGNAME ": Update mqtt"));
    #endif

    Response_P(PSTR("{\"PLYR\":{"));

    ResponseAppend_P(PSTR("\"SerialState\": %d"), st->serial_state);
    if (st->serial_state == PLAYER_CTRL_SERIAL_INIT) {
        ResponseAppend_P(PSTR(",\"Power\": %d"), st->power_state);
    }

    ResponseJsonEnd();
    ResponseJsonEnd();

    if (send) {
        MqttPublishPrefixTopicRulesProcess_P(STAT, PSTR("PLYR"));
    }
}

static void player_ctrl_handle_timeout(struct player_ctrl_softc_s *st) {

    #ifdef DEBUG_PLAYER_CTRL
        AddLog(LOG_LEVEL_DEBUG, PSTR(PLAYER_CTRL_LOGNAME ": Handle timeout %d"), st->timeout_to_do);
    #endif

    if (st == nullptr) {
        // Pre init not done
        return ;
    }

    if (st->serial_state != PLAYER_CTRL_SERIAL_TIMEOUT) {
        // Not in timeout
        return ;
    }

    if (st->timeout_to_do < 1) {
        // Timeout done ... retry init
        #ifdef DEBUG_PLAYER_CTRL
            AddLog(LOG_LEVEL_DEBUG, PSTR(PLAYER_CTRL_LOGNAME ": Timeout done. Retry init"));
        #endif    
        st->init_retries = 0;
        st->serial_state = PLAYER_CTRL_SERIAL_NOT_INIT;
        return ;

    }

    st->timeout_to_do--;

    return ;
}

static void player_ctrl_parse_paket(struct player_ctrl_softc_s * st) {

    #ifdef DEBUG_PLAYER_CTRL
        AddLog(LOG_LEVEL_DEBUG, PSTR(PLAYER_CTRL_LOGNAME ": Parse Paket"));
    #endif

    if (st->current_data->size()) {

        uint8_t current_start = st->current_data->get(0);
        while (current_start != '@' && st->current_data->size()) {
            st->current_data->shift();

            if (st->current_data->size()) {
                current_start = st->current_data->get(0);
            }
        }

        if (!st->current_data->size()) {
            #ifdef DEBUG_PLAYER_CTRL
                AddLog(LOG_LEVEL_DEBUG, PSTR(PLAYER_CTRL_LOGNAME ": Got no start"));
            #endif
            return ;
        }

        bool end_found = false;

        for (int i = 0; i < st->current_data->size(); i++) {
            if (st->current_data->get(i) == 0x0D) {
                end_found = true;
            }
        }

        if (!end_found) {
            #ifdef DEBUG_PLAYER_CTRL
                AddLog(LOG_LEVEL_DEBUG, PSTR(PLAYER_CTRL_LOGNAME ": No end found"));
            #endif
            return ;
        }

        st->last_paket = millis();
        
        st->current_data->shift(); // Discard Start

        if (st->current_data->get(2) == ' ' || st->current_data->get(2) == 0x0D) {

            #ifdef DEBUG_PLAYER_CTRL
                AddLog(LOG_LEVEL_INFO, PSTR(PLAYER_CTRL_LOGNAME ": Got short response"));
            #endif

            // Short response
            bool ok = false;
            if (st->current_data->get(0) == 'O' && st->current_data->get(1) == 'K') {
                ok = true;
                #ifdef DEBUG_PLAYER_CTRL
                    AddLog(LOG_LEVEL_INFO, PSTR(PLAYER_CTRL_LOGNAME ": Got OK Response"));
                #endif
            } else {
                #ifdef DEBUG_PLAYER_CTRL
                    AddLog(LOG_LEVEL_INFO, PSTR(PLAYER_CTRL_LOGNAME ": Got ER response"));
                #endif
            }

            st->current_data->shift(); // O or E
            st->current_data->shift(); // K or R
            st->current_data->shift(); // SPACE

            if (st->serial_state == PLAYER_CTRL_SERIAL_WAIT_FOR_INIT) {
                if (st->current_data->get(0) == 'O' && st->current_data->get(1) == 'N') {
                    #ifdef DEBUG_PLAYER_CTRL
                        AddLog(LOG_LEVEL_INFO, PSTR(PLAYER_CTRL_LOGNAME ": Got ON"));
                    #endif
                    st->current_data->shift();
                    st->current_data->shift();
                    st->serial_state = PLAYER_CTRL_SERIAL_INIT;
                    st->power_state = PLAYER_CTRL_POWER_STATE_ON;
                } else if (
                    st->current_data->get(0) == 'O' && 
                    st->current_data->get(1) == 'F' &&
                    st->current_data->get(2) == 'F'
                ) {
                    #ifdef DEBUG_PLAYER_CTRL
                        AddLog(LOG_LEVEL_INFO, PSTR(PLAYER_CTRL_LOGNAME ": Got OFF"));
                    #endif
                    st->current_data->shift();
                    st->current_data->shift();
                    st->current_data->shift();
                    st->serial_state = PLAYER_CTRL_SERIAL_INIT;
                    st->power_state = PLAYER_CTRL_POWER_STATE_OFF;
                }

                // Set Verbose mode
                player_ctrl_set_verbose_mode(st);

            }
        } else {
            // Verbose response

            char cmd[4];
            cmd[0] = st->current_data->shift();
            cmd[1] = st->current_data->shift();
            cmd[2] = st->current_data->shift();
            cmd[3] = 0;

            #ifdef DEBUG_PLAYER_CTRL
                AddLog(LOG_LEVEL_INFO, PSTR(PLAYER_CTRL_LOGNAME ": Got verbose response: %s"), cmd);
            #endif

            st->current_data->shift(); // Discard SPACE


            // Short response
            bool ok = false;

            if (cmd[0] != 'U') {
                if (st->current_data->get(0) == 'O' && st->current_data->get(1) == 'K') {
                    ok = true;
                    #ifdef DEBUG_PLAYER_CTRL
                        AddLog(LOG_LEVEL_INFO, PSTR(PLAYER_CTRL_LOGNAME ": Got OK Response"));
                    #endif
                } else {
                    #ifdef DEBUG_PLAYER_CTRL
                        AddLog(LOG_LEVEL_INFO, PSTR(PLAYER_CTRL_LOGNAME ": Got ER response"));
                    #endif
                }

                st->current_data->shift(); // O or E
                st->current_data->shift(); // K or R
            }

            st->current_data->shift(); // SPACE

            if (!strncmp(cmd, "PON", 3)) {
                if (ok) {
                    st->power_state = PLAYER_CTRL_POWER_STATE_ON;
                }
            } else if (!strncmp(cmd, "POF", 3)) {
                if (ok) {
                    st->power_state = PLAYER_CTRL_POWER_STATE_OFF;
                }
            } else if (!strncmp(cmd, "QPW", 3)) {
                if (st->serial_state == PLAYER_CTRL_SERIAL_WAIT_FOR_INIT) {
                    st->serial_state = PLAYER_CTRL_SERIAL_INIT;
                }

                if (st->current_data->size() > 2) {
                    if (st->current_data->get(0) == 'O' && st->current_data->get(1) == 'N') {
                        st->power_state = PLAYER_CTRL_POWER_STATE_ON;
                    } else if (st->current_data->get(0) == 'O' && st->current_data->get(1) == 'F' && st->current_data->get(2) == 'F') {
                        st->power_state = PLAYER_CTRL_POWER_STATE_OFF;
                    }
                }
            } else if (!strncmp(cmd, "UPW", 3)) {
                if (st->current_data->size()) {
                    if (st->current_data->get(0) == '1') {
                        st->power_state = PLAYER_CTRL_POWER_STATE_ON;
                    } else if (st->current_data->get(1) == '0') {
                        st->power_state = PLAYER_CTRL_POWER_STATE_OFF;
                    }
                }
            }
        }

        while (st->current_data->size() && st->current_data->get(0) != 0x0D) {
            st->current_data->shift();
        }

        if (st->current_data->size()) {
            st->current_data->shift();
        } else {
            #ifdef DEBUG_PLAYER_CTRL
                AddLog(LOG_LEVEL_DEBUG, PSTR(PLAYER_CTRL_LOGNAME ": Got no end after parse"));
            #endif
        }

        st->last_paket = millis();

        player_ctrl_update_mqtt(st, true);
    } else {
        #ifdef DEBUG_PLAYER_CTRL
            AddLog(LOG_LEVEL_DEBUG, PSTR(PLAYER_CTRL_LOGNAME ": No data"));
        #endif
    }
}

static bool player_ctrl_command(void) {

    #ifdef DEBUG_PLAYER_CTRL
        AddLog(LOG_LEVEL_DEBUG, PSTR(PLAYER_CTRL_LOGNAME ": Command"));
    #endif

    struct player_ctrl_softc_s *st = player_ctrl_state;
    uint8_t paramcount = 0;
    bool serviced = true;
    if (XdrvMailbox.data_len > 0) {
        paramcount = 1;
    } else {
        return false;
    }

    char argument[XdrvMailbox.data_len];
    for (uint32_t ca = 0; ca < XdrvMailbox.data_len; ca++) {
        if ((' ' == XdrvMailbox.data[ca]) || ('=' == XdrvMailbox.data[ca])) { XdrvMailbox.data[ca] = ','; }
        if (',' == XdrvMailbox.data[ca]) { paramcount++; }
    }

    UpperCase(XdrvMailbox.data, XdrvMailbox.data);

    if (!strcmp(ArgV(argument, 1), "POWER")) {
        #ifdef DEBUG_PLAYER_CTRL
            AddLog(LOG_LEVEL_DEBUG, PSTR(PLAYER_CTRL_LOGNAME ": Got power command"));
        #endif
        if (paramcount > 1) {
            if (!strcmp(ArgV(argument, 2), "ON")) {
                player_ctrl_send_power_command(true, st);
            } else {
                player_ctrl_send_power_command(false, st);
            }
        }
    } else if (!strcmp(ArgV(argument,1), "EJECT")) {
        #ifdef DEBUG_PLAYER_CTRL
            AddLog(LOG_LEVEL_INFO, PSTR(PLAYER_CTRL_LOGNAME ": Got eject command"));
        #endif
        if (paramcount > 1) {
        } else {
            player_ctrl_send_eject_toggle_command(st);
        }
    } else if (!strcmp(ArgV(argument,1), "PLAY")) {
        #ifdef DEBUG_PLAYER_CTRL
            AddLog(LOG_LEVEL_INFO, PSTR(PLAYER_CTRL_LOGNAME ": Got play command"));
        #endif
        if (paramcount > 1) {
        } else {
            player_ctrl_send_play_command(st);
        }
    } else if (!strcmp(ArgV(argument,1), "PAUSE")) {
        #ifdef DEBUG_PLAYER_CTRL
            AddLog(LOG_LEVEL_INFO, PSTR(PLAYER_CTRL_LOGNAME ": Got Pause command"));
        #endif
        if (paramcount > 1) {
        } else {
            player_ctrl_send_pause_command(st);
        }
    } else if (!strcmp(ArgV(argument, 1), "TOP_MENU")) {
        #ifdef DEBUG_PLAYER_CTRL
            AddLog(LOG_LEVEL_INFO, PSTR(PLAYER_CTRL_LOGNAME ": Got Top Menu command"));
        #endif
        if (paramcount > 1) {

        } else {
            player_ctrl_send_std_command('T','T','L', st);
        }
    } else if (!strcmp(ArgV(argument, 1), "POP_UP_MENU")) {
        #ifdef DEBUG_PLAYER_CTRL
            AddLog(LOG_LEVEL_INFO, PSTR(PLAYER_CTRL_LOGNAME ": Got PopUp Menu command"));
        #endif
        if (paramcount > 1) {

        } else {
            player_ctrl_send_std_command('M','N','U', st);
        }
    } else if (!strcmp(ArgV(argument, 1), "UP")) {
        #ifdef DEBUG_PLAYER_CTRL
            AddLog(LOG_LEVEL_INFO, PSTR(PLAYER_CTRL_LOGNAME ": Got Up command"));
        #endif
        if (paramcount > 1) {

        } else {
            player_ctrl_send_std_command('N','U','P', st);
        }
    } else if (!strcmp(ArgV(argument, 1), "LEFT")) {
        #ifdef DEBUG_PLAYER_CTRL
            AddLog(LOG_LEVEL_INFO, PSTR(PLAYER_CTRL_LOGNAME ": Got Left command"));
        #endif
        if (paramcount > 1) {

        } else {
            player_ctrl_send_std_command('N','L','T', st);
        }
    } else if (!strcmp(ArgV(argument, 1), "RIGHT")) {
        #ifdef DEBUG_PLAYER_CTRL
            AddLog(LOG_LEVEL_INFO, PSTR(PLAYER_CTRL_LOGNAME ": Got Right command"));
        #endif
        if (paramcount > 1) {

        } else {
            player_ctrl_send_std_command('N','R','T', st);
        }
    } else if (!strcmp(ArgV(argument, 1), "DOWN")) {
        #ifdef DEBUG_PLAYER_CTRL
            AddLog(LOG_LEVEL_INFO, PSTR(PLAYER_CTRL_LOGNAME ": Got Down command"));
        #endif
        if (paramcount > 1) {

        } else {
            player_ctrl_send_std_command('N','D','N', st);
        }
    } else if (!strcmp(ArgV(argument, 1), "ENTER")) {
        #ifdef DEBUG_PLAYER_CTRL
            AddLog(LOG_LEVEL_INFO, PSTR(PLAYER_CTRL_LOGNAME ": Got Enter command"));
        #endif
        if (paramcount > 1) {

        } else {
            player_ctrl_send_std_command('S','E','L', st);
        }
    } else if (!strcmp(ArgV(argument, 1), "RETURN")) {
        #ifdef DEBUG_PLAYER_CTRL
            AddLog(LOG_LEVEL_INFO, PSTR(PLAYER_CTRL_LOGNAME ": Got Return command"));
        #endif
        if (paramcount > 1) {

        } else {
            player_ctrl_send_std_command('R','E','T', st);
        }
    } else if (!strcmp(ArgV(argument, 1), "AUDIO")) {
        #ifdef DEBUG_PLAYER_CTRL
            AddLog(LOG_LEVEL_INFO, PSTR(PLAYER_CTRL_LOGNAME ": Got Audio command"));
        #endif
        if (paramcount > 1) {

        } else {
            player_ctrl_send_std_command('A','U','D', st);
        }
    } else if (!strcmp(ArgV(argument, 1), "SUBTITLE")) {
        #ifdef DEBUG_PLAYER_CTRL
            AddLog(LOG_LEVEL_INFO, PSTR(PLAYER_CTRL_LOGNAME ": Got Subtitle command"));
        #endif
        if (paramcount > 1) {

        } else {
            player_ctrl_send_std_command('S','U','B', st);
        }
    } else if (!strcmp(ArgV(argument, 1), "ANGLE")) {
        #ifdef DEBUG_PLAYER_CTRL
            AddLog(LOG_LEVEL_INFO, PSTR(PLAYER_CTRL_LOGNAME ": Got Angle command"));
        #endif
        if (paramcount > 1) {

        } else {
            player_ctrl_send_std_command('A','N','G', st);
        }
    } else if (!strcmp(ArgV(argument, 1), "ZOOM")) {
        #ifdef DEBUG_PLAYER_CTRL
            AddLog(LOG_LEVEL_INFO, PSTR(PLAYER_CTRL_LOGNAME ": Got Zoom command"));
        #endif
        if (paramcount > 1) {

        } else {
            player_ctrl_send_std_command('Z','O','M', st);
        }
    } else if (!strcmp(ArgV(argument, 1), "RESOLUTION")) {
        #ifdef DEBUG_PLAYER_CTRL
            AddLog(LOG_LEVEL_INFO, PSTR(PLAYER_CTRL_LOGNAME ": Got Resolution command"));
        #endif
        if (paramcount > 1) {

        } else {
            player_ctrl_send_std_command('H','D','M', st);
        }
    } else if (!strcmp(ArgV(argument, 1), "OPTION")) {
        #ifdef DEBUG_PLAYER_CTRL
            AddLog(LOG_LEVEL_INFO, PSTR(PLAYER_CTRL_LOGNAME ": Got Option command"));
        #endif
        if (paramcount > 1) {

        } else {
            player_ctrl_send_std_command('O','P','T', st);
        }
    } else if (!strcmp(ArgV(argument, 1), "3D")) {
        #ifdef DEBUG_PLAYER_CTRL
            AddLog(LOG_LEVEL_INFO, PSTR(PLAYER_CTRL_LOGNAME ": Got 3D command"));
        #endif
        if (paramcount > 1) {

        } else {
            player_ctrl_send_std_command('M','3','D', st);
        }
    } else if (!strcmp(ArgV(argument, 1), "HOME")) {
        #ifdef DEBUG_PLAYER_CTRL
            AddLog(LOG_LEVEL_INFO, PSTR(PLAYER_CTRL_LOGNAME ": Got Home command"));
        #endif
        if (paramcount > 1) {

        } else {
            player_ctrl_send_std_command('H','O','M', st);
        }
    }

    player_ctrl_update_mqtt(st, false);

    return serviced;
}

static void player_ctrl_send_init_command(struct player_ctrl_softc_s *st) {

    #ifdef DEBUG_PLAYER_CTRL
        AddLog(LOG_LEVEL_DEBUG, PSTR(PLAYER_CTRL_LOGNAME ": Sending init command"));
    #endif


    st->serial->write('#');

    st->serial->write('Q');
    st->serial->write('P');
    st->serial->write('W');

    st->serial->write(0x0D);
    st->serial->flush();

    #ifdef DEBUG_PLAYER_CTRL
        AddLog(LOG_LEVEL_DEBUG, PSTR(PLAYER_CTRL_LOGNAME ": Sending init done"));
    #endif
}

static void player_ctrl_send_power_command(bool on, struct player_ctrl_softc_s *st) {
    #ifdef DEBUG_PLAYER_CTRL
        AddLog(LOG_LEVEL_DEBUG, PSTR(PLAYER_CTRL_LOGNAME ": Sending power command"));
    #endif
    
    st->serial->write('#');

    st->serial->write('P');
    st->serial->write('O');

    if (on) {
        st->serial->write('N');
    } else {
        st->serial->write('F');
    }

    st->serial->write(0x0D);
    st->serial->flush();
}

static void player_ctrl_send_eject_toggle_command(struct player_ctrl_softc_s *st) {
    #ifdef DEBUG_PLAYER_CTRL
        AddLog(LOG_LEVEL_DEBUG, PSTR(PLAYER_CTRL_LOGNAME ": Sending eject toggle command"));
    #endif

    st->serial->write('#');

    st->serial->write('E');
    st->serial->write('J');
    st->serial->write('T');

    st->serial->write(0x0D);
    st->serial->flush();
}

static void player_ctrl_send_play_command(struct player_ctrl_softc_s *st) {
    #ifdef DEBUG_PLAYER_CTRL
        AddLog(LOG_LEVEL_DEBUG, PSTR(PLAYER_CTRL_LOGNAME ": Sending eject toggle command"));
    #endif

    st->serial->write('#');

    st->serial->write('P');
    st->serial->write('L');
    st->serial->write('A');

    st->serial->write(0x0D);
    st->serial->flush();
}

static void player_ctrl_send_std_command(byte cmd1, byte cmd2, byte cmd3, struct player_ctrl_softc_s *st) {
    #ifdef DEBUG_PLAYER_CTRL
        AddLog(LOG_LEVEL_DEBUG, PSTR(PLAYER_CTRL_LOGNAME ": Sending std command " cmd1 cmd2 cmd3));
    #endif

    st->serial->write('#');
    st->serial->write(cmd1);
    st->serial->write(cmd2);
    st->serial->write(cmd3);

    st->serial->write(0x0D);
    st->serial->flush();
}

static void player_ctrl_send_pause_command(struct player_ctrl_softc_s *st) {
    #ifdef DEBUG_PLAYER_CTRL
        AddLog(LOG_LEVEL_DEBUG, PSTR(PLAYER_CTRL_LOGNAME ": Sending eject toggle command"));
    #endif

    st->serial->write('#');

    st->serial->write('P');
    st->serial->write('A');
    st->serial->write('U');

    st->serial->write(0x0D);
    st->serial->flush();
}

static void player_ctrl_set_verbose_mode(struct player_ctrl_softc_s *st) {
    #ifdef DEBUG_PLAYER_CTRL
        AddLog(LOG_LEVEL_DEBUG, PSTR(PLAYER_CTRL_LOGNAME ": Sending set verbose mode"));
    #endif

    st->serial->write('#');

    st->serial->write('S');
    st->serial->write('V');
    st->serial->write('M');
    st->serial->write(' ');
    st->serial->write('3');

    st->serial->write(0x0D);
    st->serial->flush();
}


static bool player_ctrl_compare_cmd(char *cmd1, char *cmd2) {
    return cmd1[0] == cmd2[0] && cmd1[1] == cmd2[1] && cmd1[2] == cmd2[2];
}

static void player_ctrl_loop(struct player_ctrl_softc_s *st) {

    if (st == nullptr) {
        return ;
    }

    if (st->serial->available()) {
        while (st->serial->available()) {
            st->current_data->add(st->serial->read());
        }
    } else if (st->serial_state == PLAYER_CTRL_SERIAL_INIT) {
        unsigned long now = millis();
        now = now - st->last_paket;

        if (now > 300000) {
            #ifdef DEBUG_PLAYER_CTRL
                AddLog(LOG_LEVEL_DEBUG, PSTR(PLAYER_CTRL_LOGNAME ": Timeout"));
            #endif
            st->serial_state = PLAYER_CTRL_SERIAL_NOT_INIT;
            st->init_retries = 0;
        }
    } else if (st->serial_state == PLAYER_CTRL_SERIAL_WAIT_FOR_INIT) {
        unsigned long now = millis();
        now = now - st->last_paket;

        if (now > 2000) {
            #ifdef DEBUG_PLAYER_CTRL
                AddLog(LOG_LEVEL_DEBUG, PSTR(PLAYER_CTRL_LOGNAME ": Timeout waiting for init response"));
            #endif
            player_ctrl_do_init(st);
        }
    }

    return ;
}

#endif