#ifdef USE_PROJECTOR_SIMPLE_CTRL

#include <LinkedList.h>

#define XDRV_65 65

#ifdef USE_PROJECTOR_SIMPLE_CTRL_SONY_VPL_HW30
#define PROJECTOR_SIMPLE_CTRL_SERIAL_BAUDRATE 38400
#define PROJECTOR_SIMPLE_CTRL_SERIAL_CONFIG SERIAL_8E1
#define PROJECTOR_SIMPLE_CTRL_LOGNAME "PRJ[VWL30]"
#endif

enum projector_simple_ctrl_serial_state_e: uint8_t {
    PROJECTOR_SIMPLE_CTRL_SERIAL_STATE_NOT_INIT=0,
    PROJECTOR_SIMPLE_CTRL_SERIAL_STATE_CONNECTED
};

enum projector_simple_ctrl_cmd_e: uint8_t {
    PROJECTOR_SIMPLE_CTRL_CMD_GET_POWER=0,
    PROJECTOR_SIMPLE_CTRL_CMD_GET_INPUT,
    PROJECTOR_SIMPLE_CTRL_CMD_GET_3D,
    PROJECTOR_SIMPLE_CTRL_CMD_SET_POWER_ON,
    PROJECTOR_SIMPLE_CTRL_CMD_SET_POWER_OFF
};

enum projector_simple_ctrl_cmd_state_e: uint8_t {
    PROJECTOR_SIMPLE_CTRL_CMD_STATE_PENDING=0,
    PROJECTOR_SIMPLE_CTRL_CMD_STATE_SEND,
    PROJECTOR_SIMPLE_CTRL_CMD_STATE_DONE
};

enum projector_simple_ctrl_power_state_e: uint8_t {
    PROJECTOR_SIMPLE_CTRL_POWER_STATE_UNKNOWN=0,
    PROJECTOR_SIMPLE_CTRL_POWER_STATE_STANDBY,
    PROJECTOR_SIMPLE_CTRL_POWER_STATE_ON
};

enum projector_simple_ctrl_input_e: uint8_t {
    PROJECTOR_SIMPLE_CTRL_INPUT_HDMI_1 = 0,
    PROJECTOR_SIMPLE_CTRL_INPUT_HDMI_2,
    PROJECTOR_SIMPLE_CTRL_INPUT_VGA
};

struct projector_simple_ctrl_cmd_s {
    enum projector_simple_ctrl_cmd_e cmd;
    enum projector_simple_ctrl_cmd_state_e state;
    unsigned long send_time;
};

struct projector_simple_ctrl_state_s {
    TasmotaSerial *serial_port;
    enum projector_simple_ctrl_serial_state_e serial_state;

    LinkedList<struct projector_simple_ctrl_cmd_s *> *commands;

    LinkedList<uint8_t> *current_data;

    enum projector_simple_ctrl_power_state_e power_state;
    enum projector_simple_ctrl_input_e input_state;
} __packed;;

static struct projector_simple_ctrl_state_s *projector_simple_ctrl_state = nullptr;

static void projector_simple_ctrl_pre_init(void);
static void projector_simple_ctrl_loop(struct projector_simple_ctrl_state_s *st);
static void projector_simple_ctrl_send_command(struct projector_simple_ctrl_cmd_s *cmd, struct projector_simple_ctrl_state_s *st);
static bool projector_simple_ctrl_tokenize_command(struct projector_simple_ctrl_state_s *st);
static void projector_simple_ctrl_parse_command(uint8_t item0, uint8_t item1, uint8_t type, uint8_t data0, uint8_t data1, uint8_t checksum, struct projector_simple_ctrl_state_s *st);
static void projector_simple_ctrl_do_update(struct projector_simple_ctrl_state_s *st);
static void projector_simple_ctrl_update_mqtt(struct projector_simple_ctrl_state_s *st, bool send);
static void projector_simple_ctrl_json_append();
static uint8_t projector_simple_ctrl_calc_checksum(uint8_t Byte1, uint8_t Byte2, uint8_t Byte3, uint8_t Byte4, uint8_t Byte5);
static bool projector_simple_ctrl_command(void);

static void projector_simple_ctrl_pre_init(void) {

#ifdef DEBUG_PROJECTOR_SIMPLE_CTRL
    AddLog(LOG_LEVEL_DEBUG, PSTR(PROJECTOR_SIMPLE_CTRL_LOGNAME ": Pre Init"));
#endif

    struct projector_simple_ctrl_state_s *st;

    if (!PinUsed(GPIO_PROJECTOR_SIMPLE_CTRL_TX) || !PinUsed(GPIO_PROJECTOR_SIMPLE_CTRL_RX)) {
        #ifdef DEBUG_PROJECTOR_SIMPLE_CTRL
            AddLog(LOG_LEVEL_DEBUG, PSTR(PROJECTOR_SIMPLE_CTRL_LOGNAME ": No pins configured"));
        #endif
        return ;
    }

    #ifdef DEBUG_PROJECTOR_SIMPLE_CTRL
    AddLog(LOG_LEVEL_DEBUG, PSTR(PROJECTOR_SIMPLE_CTRL_LOGNAME ": pins configured TX: %d RX: %d"), Pin(GPIO_PROJECTOR_SIMPLE_CTRL_TX), Pin(GPIO_PROJECTOR_SIMPLE_CTRL_RX));
    #endif

    st = (struct projector_simple_ctrl_state_s *)malloc(sizeof(*st));
    if (st == NULL) {
        AddLog(LOG_LEVEL_ERROR, PSTR(PROJECTOR_SIMPLE_CTRL_LOGNAME ": unable to allocate state"));
        return ;
    }

    memset(st, 0, sizeof(*st));

    st->current_data = new LinkedList<uint8_t>();
    st->commands = new LinkedList<projector_simple_ctrl_cmd_s *>();
    st->serial_state = PROJECTOR_SIMPLE_CTRL_SERIAL_STATE_NOT_INIT;
    st->power_state = PROJECTOR_SIMPLE_CTRL_POWER_STATE_UNKNOWN;
    
    #ifdef DEBUG_PROJECTOR_SIMPLE_CTRL
    AddLog(LOG_LEVEL_DEBUG, PSTR(PROJECTOR_SIMPLE_CTRL_LOGNAME ": Creating serial"));
    #endif

    st->serial_port = new TasmotaSerial(Pin(GPIO_PROJECTOR_SIMPLE_CTRL_RX), Pin(GPIO_PROJECTOR_SIMPLE_CTRL_TX), PROJECTOR_SIMPLE_CTRL_SERIAL_CONFIG);

    if (!st->serial_port->begin(PROJECTOR_SIMPLE_CTRL_SERIAL_BAUDRATE)) {
        AddLog(LOG_LEVEL_ERROR, PSTR(PROJECTOR_SIMPLE_CTRL_LOGNAME ": unable to begin serial (baudrate %d)"), PROJECTOR_SIMPLE_CTRL_SERIAL_BAUDRATE);
        goto del;
    }

    if (st->serial_port->hardwareSerial()) {
        ClaimSerial();
        SetSerial(PROJECTOR_SIMPLE_CTRL_SERIAL_BAUDRATE,PROJECTOR_SIMPLE_CTRL_SERIAL_CONFIG);
    }

    projector_simple_ctrl_state = st;

    return ;

del:
    delete st->current_data;
    delete st->commands;
    delete st->serial_port;
    free(st);
}

static void projector_simple_ctrl_loop(struct projector_simple_ctrl_state_s *st) {
    if (st != NULL) {
        if (st->serial_port->available()) {
            while (st->serial_port->available()) {
                int data = st->serial_port->read();
                #ifdef DEBUG_PROJECTOR_SIMPLE_CTRL
                    AddLog(LOG_LEVEL_DEBUG, PSTR(PROJECTOR_SIMPLE_CTRL_LOGNAME ": Got serial data %02x"), data);
                #endif
                st->current_data->add(data);
            }
        } else if (st->serial_state == PROJECTOR_SIMPLE_CTRL_SERIAL_STATE_CONNECTED) {
           
        } else {

        }

        if (st->commands->size()) {
            struct projector_simple_ctrl_cmd_s *cmd = st->commands->get(0);

            if (cmd->state == PROJECTOR_SIMPLE_CTRL_CMD_STATE_PENDING) {
                // SEND COMMAND
                #ifdef DEBUG_PROJECTOR_SIMPLE_CTRL
                    AddLog(LOG_LEVEL_DEBUG, PSTR(PROJECTOR_SIMPLE_CTRL_LOGNAME ": Sending command %d"), cmd->cmd);
                #endif
                projector_simple_ctrl_send_command(cmd, st);

            } else if (cmd->state == PROJECTOR_SIMPLE_CTRL_CMD_STATE_SEND) {
                unsigned long time_since_send = millis() - cmd->send_time;
                if (time_since_send > 1000) {
                    #ifdef DEBUG_PROJECTOR_SIMPLE_CTRL
                        AddLog(LOG_LEVEL_DEBUG, PSTR(PROJECTOR_SIMPLE_CTRL_LOGNAME ": Command %d timed out... discarding"), cmd->cmd);
                    #endif
                    st->serial_state = PROJECTOR_SIMPLE_CTRL_SERIAL_STATE_NOT_INIT;
                    st->power_state = PROJECTOR_SIMPLE_CTRL_POWER_STATE_UNKNOWN;
                    st->current_data->clear();
                    cmd = st->commands->pop();
                    free(cmd);
                    projector_simple_ctrl_update_mqtt(st,true);
                }
            }
        }
    }
}

static void projector_simple_ctrl_send_command(struct projector_simple_ctrl_cmd_s *cmd, struct projector_simple_ctrl_state_s *st) {

    bool cmd_send = true;
    uint8_t to_send[8];

    uint8_t num_sends = 1;

    to_send[0] = 0xA9;
    to_send[4] = 0x00;
    to_send[5] = 0x00;
    to_send[7] = 0x9A;

    switch (cmd->cmd) {

        case PROJECTOR_SIMPLE_CTRL_CMD_GET_POWER:
            to_send[1] = 0x01;
            to_send[2] = 0x02;
            to_send[3] = 0x01;
            break;
        case PROJECTOR_SIMPLE_CTRL_CMD_SET_POWER_ON:
            to_send[1] = 0x17;
            to_send[2] = 0x2E;
            to_send[3] = 0x00;
            num_sends = 2;
            break;
        case PROJECTOR_SIMPLE_CTRL_CMD_SET_POWER_OFF:
            to_send[1] = 0x17;
            to_send[2] = 0x2F;
            to_send[3] = 0x00;
            break;
        default:
            AddLog(LOG_LEVEL_INFO, PSTR(PROJECTOR_SIMPLE_CTRL_LOGNAME ": Unknown command %d"), cmd->cmd);
            st->commands->shift();
            cmd_send = false;
            break;
    }

    if (cmd_send) {

        to_send[6] = projector_simple_ctrl_calc_checksum(to_send[1], to_send[2], to_send[3], to_send[4], to_send[5]);

        #ifdef DEBUG_PROJECTOR_SIMPLE_CTRL
            AddLog(LOG_LEVEL_DEBUG, PSTR(PROJECTOR_SIMPLE_CTRL_LOGNAME ": Sending command %02x"),cmd->cmd);
        #endif

        for (uint8_t j = 0; j < num_sends; j++) {
            for (uint8_t i = 0; i < 8; i++) {
                st->serial_port->write(to_send[i]);
            }
        }

        st->serial_port->flush();

        cmd->state = PROJECTOR_SIMPLE_CTRL_CMD_STATE_SEND;
        cmd->send_time = millis();
    }
}

static bool projector_simple_ctrl_tokenize_command(struct projector_simple_ctrl_state_s *st) {
    if (!st->current_data->size()) {
        return false;
    }

    #ifdef DEBUG_PROJECTOR_SIMPLE_CTRL
        AddLog(LOG_LEVEL_DEBUG, PSTR(PROJECTOR_SIMPLE_CTRL_LOGNAME ": Parsing commdn"));
    #endif

    while (st->current_data->size() && st->current_data->get(0) == 0xA9) {
        st->current_data->shift();
    }

    #ifdef DEBUG_PROJECTOR_SIMPLE_CTRL
        AddLog(LOG_LEVEL_DEBUG, PSTR(PROJECTOR_SIMPLE_CTRL_LOGNAME ": Got paket start"));
        for (int i = 0; i < st->current_data->size(); i++) {
            AddLog(LOG_LEVEL_DEBUG, PSTR(PROJECTOR_SIMPLE_CTRL_LOGNAME ": Ser: %d %02x"), i, st->current_data->get(i));
        }
    #endif

    if (st->current_data->size() < 8) {
        AddLog(LOG_LEVEL_DEBUG, PSTR(PROJECTOR_SIMPLE_CTRL_LOGNAME ": Recveived Paket not complete"));
        return false;
    }

    // Start Byte
    st->current_data->shift();
    uint8_t Byte1 = st->current_data->shift();
    uint8_t Byte2 = st->current_data->shift();

    uint8_t type = st->current_data->shift();

    uint8_t data0 = st->current_data->shift();
    uint8_t data1 = st->current_data->shift();

    uint8_t checksum = st->current_data->shift();

    // End byte
    st->current_data->shift();

    if (st->commands->size()) {
        struct projector_simple_ctrl_cmd_s *cmd = st->commands->get(0);

        if (cmd->state == PROJECTOR_SIMPLE_CTRL_CMD_STATE_SEND || cmd->state == PROJECTOR_SIMPLE_CTRL_CMD_STATE_DONE) {
            st->commands->shift();

            #ifdef DEBUG_PROJECTOR_SIMPLE_CTRL
                AddLog(LOG_LEVEL_DEBUG, PSTR(PROJECTOR_SIMPLE_CTRL_LOGNAME ": Removed finished command"));
            #endif
            free(cmd);

        }
    }

    #ifdef DEBUG_PROJECTOR_SIMPLE_CTRL
            AddLog(LOG_LEVEL_DEBUG, PSTR(PROJECTOR_SIMPLE_CTRL_LOGNAME ": Tokenized: B1: %02x B2: %02x, Type: %02x, Data1: %02x, Data2: %02x"), Byte1, Byte2, type, data0, data1);
    #endif

    projector_simple_ctrl_parse_command(
        Byte1, Byte2,
        type,
        data0, data1,
        checksum,
        st
    );

    projector_simple_ctrl_update_mqtt(st, true);


    return true;
}


static void projector_simple_ctrl_do_update(struct projector_simple_ctrl_state_s *st) {

    #ifdef DEBUG_PROJECTOR_SIMPLE_CTRL
        AddLog(LOG_LEVEL_DEBUG, PSTR(PROJECTOR_SIMPLE_CTRL_LOGNAME ": Do Update called"));
    #endif

    bool pwr_req_found = false;
    for (int i = 0; i < st->commands->size(); i++) {
        struct projector_simple_ctrl_cmd_s *cmd = st->commands->get(i);

        if (cmd->cmd == PROJECTOR_SIMPLE_CTRL_CMD_GET_POWER) {
            pwr_req_found = true;
        }
    }

    if (!pwr_req_found) {
        #ifdef DEBUG_PROJECTOR_SIMPLE_CTRL
            AddLog(LOG_LEVEL_DEBUG, PSTR(PROJECTOR_SIMPLE_CTRL_LOGNAME ": No PWR Cmd found. Adding"));
        #endif
        struct projector_simple_ctrl_cmd_s *cmd;
        cmd = (struct projector_simple_ctrl_cmd_s *)malloc(sizeof(*cmd));
        if (cmd != NULL) {
            cmd->cmd = PROJECTOR_SIMPLE_CTRL_CMD_GET_POWER;
            cmd->state = PROJECTOR_SIMPLE_CTRL_CMD_STATE_PENDING;
            st->commands->add(cmd);
        } else {
            AddLog(LOG_LEVEL_INFO, PSTR(PROJECTOR_SIMPLE_CTRL_LOGNAME ": Could not alloc cmd struct"));
        }
    } else {
        #ifdef DEBUG_PROJECTOR_SIMPLE_CTRL
            AddLog(LOG_LEVEL_DEBUG, PSTR(PROJECTOR_SIMPLE_CTRL_LOGNAME ": PWR Cmd found. Not Adding"));
        #endif
    }
}

static void projector_simple_ctrl_parse_command(uint8_t item0, uint8_t item1, uint8_t type, uint8_t data0, uint8_t data1, uint8_t checksum, struct projector_simple_ctrl_state_s *st) {
    if (item0 == 0x01 && item1 == 0x02) {
        #ifdef DEBUG_PROJECTOR_SIMPLE_CTRL
            AddLog(LOG_LEVEL_DEBUG, PSTR(PROJECTOR_SIMPLE_CTRL_LOGNAME ": Got Status Power %02x%02x"), data0, data1);
        #endif

        switch (data1) {
            case 0x00:
            case 0x08:
                st->power_state = PROJECTOR_SIMPLE_CTRL_POWER_STATE_STANDBY;
                break;
            default:
                st->power_state = PROJECTOR_SIMPLE_CTRL_POWER_STATE_ON;
                break;
        }
    }
}

static void projector_simple_ctrl_update_mqtt(struct projector_simple_ctrl_state_s *st, bool send) {
    ResponseAppend_P(PSTR("{\"PRJ\":{"));
    ResponseAppend_P(PSTR("\"SerialState\": %d"), st->serial_state);
    ResponseAppend_P(PSTR(",\"Power\": %d"), st->power_state);
    if (st->power_state == PROJECTOR_SIMPLE_CTRL_POWER_STATE_ON) {
        ResponseAppend_P(PSTR(",\"Input\": %d"), st->input_state);
    }
    ResponseJsonEnd();
    ResponseJsonEnd();

    if (send) {
        MqttPublishPrefixTopicRulesProcess_P(RESULT_OR_STAT, PSTR("PRJ"));
    }
}

static void projector_simple_ctrl_json_append() {
    ResponseAppend_P(PSTR("\"Projector\":{"));
    if (projector_simple_ctrl_state == NULL) {
        ResponseAppend_P(PSTR("\"Status\": -1"));
    } else {
        ResponseAppend_P(PSTR("\"Status\": 1"));
    }
    ResponseJsonEnd();
}

static uint8_t projector_simple_ctrl_calc_checksum(uint8_t Byte1, uint8_t Byte2, uint8_t Byte3, uint8_t Byte4, uint8_t Byte5) {
    return Byte1 | Byte2 | Byte3 | Byte4 | Byte5;
}

static bool projector_simple_ctrl_command(void) {

    uint8_t paramcount = 0;
    bool serviced = true;
    struct projector_simple_ctrl_state_s *st = projector_simple_ctrl_state;

    struct projector_simple_ctrl_cmd_s *cmd = nullptr;

    if (XdrvMailbox.data_len > 0) {
        paramcount = 1;
    } else {
        return false;
    }

    char argument[XdrvMailbox.data_len];
    for (uint32_t ca = 0; ca < XdrvMailbox.data_len; ca++) {
        if ((' ' == XdrvMailbox.data[ca]) || ('=' == XdrvMailbox.data[ca])) {
            XdrvMailbox.data[ca] = ',';
        }

        if (',' == XdrvMailbox.data[ca]) {
            paramcount++;
        } 
    }

    UpperCase(XdrvMailbox.data, XdrvMailbox.data);

    if (!strcmp(ArgV(argument, 1), "STATUS")) {
        #ifdef DEBUG_PROJECTOR_SIMPLE_CTRL
            AddLog(LOG_LEVEL_INFO, PSTR(PROJECTOR_SIMPLE_CTRL_LOGNAME ": Got status command"));
        #endif
        projector_simple_ctrl_update_mqtt(st, false);
        return serviced;
    }

    if (!strcmp(ArgV(argument, 1), "POWER")) {
        AddLog(LOG_LEVEL_INFO, PSTR(PROJECTOR_SIMPLE_CTRL_LOGNAME ": Got power command"));
        if (paramcount > 1) {
            #ifdef DEBUG_PROJECTOR_SIMPLE_CTRL
                AddLog(LOG_LEVEL_DEBUG, PSTR(PROJECTOR_SIMPLE_CTRL_LOGNAME ": Before mallow"));
            #endif
            cmd = (struct projector_simple_ctrl_cmd_s *)malloc(sizeof(*cmd));
            #ifdef DEBUG_PROJECTOR_SIMPLE_CTRL
                AddLog(LOG_LEVEL_DEBUG, PSTR(PROJECTOR_SIMPLE_CTRL_LOGNAME ": After malloc"));
            #endif
            if (cmd != nullptr) {
                cmd->state = PROJECTOR_SIMPLE_CTRL_CMD_STATE_PENDING;
                if (!strcmp(ArgV(argument, 2), "ON")) {
                    #ifdef DEBUG_PROJECTOR_SIMPLE_CTRL
                        AddLog(LOG_LEVEL_INFO, PSTR(PROJECTOR_SIMPLE_CTRL_LOGNAME ": Got Power On command"));
                    #endif
                    cmd->cmd = PROJECTOR_SIMPLE_CTRL_CMD_SET_POWER_ON;
                } else {
                    #ifdef DEBUG_PROJECTOR_SIMPLE_CTRL
                        AddLog(LOG_LEVEL_INFO, PSTR(PROJECTOR_SIMPLE_CTRL_LOGNAME ": Got Power OFF command"));
                    #endif
                    cmd->cmd = PROJECTOR_SIMPLE_CTRL_CMD_SET_POWER_OFF;
                }
                st->commands->add(cmd);
            } else {
                AddLog(LOG_LEVEL_ERROR, PSTR(PROJECTOR_SIMPLE_CTRL_LOGNAME ": Could not allocate cmd"));
            }
        }
        
    }
    return serviced;
}

bool Xdrv65(uint8_t function) {

    struct projector_simple_ctrl_state_s *st;

    st = projector_simple_ctrl_state;

    switch (function) {
        case FUNC_PRE_INIT:
            projector_simple_ctrl_pre_init();
            return (false);
        case FUNC_JSON_APPEND:
            projector_simple_ctrl_json_append();
            return (false);
    }

    if (st == nullptr) {
        return (false);
    }

    bool result = false;

    switch (function) {
        case FUNC_LOOP:
            projector_simple_ctrl_loop(st);
            break;
        case FUNC_EVERY_100_MSECOND:
            while (projector_simple_ctrl_tokenize_command(st)) {

            }
            break;
        case FUNC_EVERY_SECOND:
            projector_simple_ctrl_do_update(st);
            break;
        case FUNC_COMMAND_DRIVER:
            if (XDRV_65 == XdrvMailbox.index) {
                result = projector_simple_ctrl_command();
            }
            break;
    }

    return result;

}

#endif