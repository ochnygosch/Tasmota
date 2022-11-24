#ifdef USE_HDMI_SWITCH_CTRL

#define XDRV_66 66

#define HDMI_SWITCH_CTRL_LOGNAME "HDMI"

struct hdmi_switch_ctrl_state_s {
    uint8_t requested_input;
    uint8_t current_input;

    uint8_t in_toggle;
    uint8_t toggle_cooldown;

    int pin_switch;
    int pin_input_1;
    int pin_input_2;
    int pin_input_3;
};

struct hdmi_switch_ctrl_state_s *hdmi_switch_ctrl_sc;

static void hdmi_switch_ctrl_pre_init(void);
static void hdmi_switch_ctrl_loop(struct hdmi_switch_ctrl_state_s *st);
static bool hdmi_switch_ctrl_command();
static void hdmi_switch_ctrl_update_mqtt(struct hdmi_switch_ctrl_state_s *st,bool send);

static void hdmi_switch_ctrl_pre_init(void) {

    #ifdef DEBUG_HDMI_SWITCH_CTRL
        AddLog(LOG_LEVEL_DEBUG,PSTR(HDMI_SWITCH_CTRL_LOGNAME ": Pre Init"));
    #endif

    if (
        !PinUsed(GPIO_HDMI_SWITCH_CTRL_SWITCH) ||
        !PinUsed(GPIO_HDMI_SWITCH_CTRL_INPUT_1) ||
        !PinUsed(GPIO_HDMI_SWITCH_CTRL_INPUT_2) ||
        !PinUsed(GPIO_HDMI_SWITCH_CTRL_INPUT_3)
    ) {
        #ifdef DEBUG_HDMI_SWITCH_CTRL
            AddLog(LOG_LEVEL_DEBUG, PSTR(HDMI_SWITCH_CTRL_LOGNAME ": No pins configured"));
        #endif

        return;
    }

    #ifdef DEBUG_HDMI_SWITCH_CTRL
        AddLog(LOG_LEVEL_DEBUG, PSTR(HDMI_SWITCH_CTRL_LOGNAME ": Pins configured"));
    #endif

    struct hdmi_switch_ctrl_state_s *st;

    st = (struct hdmi_switch_ctrl_state_s *)malloc(sizeof(*st));

    if (st == nullptr) {
        AddLog(LOG_LEVEL_ERROR, PSTR(HDMI_SWITCH_CTRL_LOGNAME ": unable to malloc state struct"));
        return ;
    }

    memset(st, 0, sizeof(*st));

    st->pin_switch = Pin(GPIO_HDMI_SWITCH_CTRL_SWITCH);
    st->pin_input_1 = Pin(GPIO_HDMI_SWITCH_CTRL_INPUT_1);
    st->pin_input_2 = Pin(GPIO_HDMI_SWITCH_CTRL_INPUT_2);
    st->pin_input_3 = Pin(GPIO_HDMI_SWITCH_CTRL_INPUT_3);
    
    #ifdef DEBUG_HDMI_SWITCH_CTRL
        AddLog(LOG_LEVEL_DEBUG, PSTR(HDMI_SWITCH_CTRL_LOGNAME ": Pins Switch: %d, Input 1: %d, Input 2: %d, Input 3: %d"), st->pin_switch, st->pin_input_1, st->pin_input_2, st->pin_input_3);
    #endif

    pinMode(st->pin_switch, OUTPUT);
    pinMode(st->pin_input_1, INPUT);
    pinMode(st->pin_input_2, INPUT);
    pinMode(st->pin_input_3, INPUT);

    digitalWrite(st->pin_switch, HIGH);

    hdmi_switch_ctrl_sc = st;
}

static void hdmi_switch_ctrl_loop(struct hdmi_switch_ctrl_state_s *st) {

    if (st == nullptr) {
        return ;
    }

    if (st->in_toggle) {
        if (st->in_toggle == 1) {
            #ifdef DEBUG_HDMI_SWITCH_CTRL
                AddLog(LOG_LEVEL_DEBUG, PSTR(HDMI_SWITCH_CTRL_LOGNAME ": Finishing toggle"));
            #endif
            digitalWrite(st->pin_switch, HIGH);
            st->toggle_cooldown = 5;
        }
        st->in_toggle--;
    }

    int val_input_1 = digitalRead(st->pin_input_1);
    int val_input_2 = digitalRead(st->pin_input_2);
    int val_input_3 = digitalRead(st->pin_input_3);


    #ifdef DEBUG_HDMI_SWITCH_CTRL
        //AddLog(LOG_LEVEL_DEBUG, PSTR(HDMI_SWITCH_CTRL_LOGNAME ": Read Input 1: %d, Input 2: %d, Input 3: %d"), val_input_1, val_input_2, val_input_3);
    #endif

    uint8_t new_input = 0;
    if (val_input_1 == LOW) {
        new_input = 1;
    } else if (val_input_2 == LOW) {
        new_input = 2;
    } else if (val_input_3 == LOW) {
        new_input = 3;
    }

    if (new_input != st->current_input) {
        st->current_input = new_input;

        hdmi_switch_ctrl_update_mqtt(st, true);
    }

    #ifdef DEBUG_HDMI_SWITCH_CTRL
        //AddLog(LOG_LEVEL_DEBUG, PSTR(HDMI_SWITCH_CTRL_LOGNAME ": Current Input: %d"), st->current_input);
    #endif

    if (st->requested_input > 0) {
        

        if (st->requested_input != st->current_input && st->in_toggle == 0 && st->toggle_cooldown == 0) {
            #ifdef DEBUG_HDMI_SWITCH_CTRL
                AddLog(LOG_LEVEL_DEBUG, PSTR(HDMI_SWITCH_CTRL_LOGNAME ": Requested Input: %d"), st->requested_input);
            #endif
            digitalWrite(st->pin_switch, LOW);
            st->in_toggle = 5;
        } else if (st->toggle_cooldown) {
            st->toggle_cooldown--;
        }
    }
}


static bool hdmi_switch_ctrl_command() {
    struct hdmi_switch_ctrl_state_s *st = hdmi_switch_ctrl_sc;
    uint8_t paramcount = 0;
    bool serviced = true;

    if (XdrvMailbox.data_len > 0) {
        paramcount = 1;
    } else {
        return false;
    }

    char argument[XdrvMailbox.data_len];
    for (uint32_t ca=0; ca < XdrvMailbox.data_len; ca++) {
        if ((' ' == XdrvMailbox.data[ca]) || ('=' == XdrvMailbox.data[ca])) { XdrvMailbox.data[ca] = ','; }
        if (',' == XdrvMailbox.data[ca]) { paramcount++; }
    }

    UpperCase(XdrvMailbox.data,XdrvMailbox.data);

    if (!strcmp(ArgV(argument, 1), "INPUT")) {
        if (paramcount == 2) {
            #ifdef DEBUG_HDMI_SWITCH_CTRL
                AddLog(LOG_LEVEL_DEBUG, PSTR(HDMI_SWITCH_CTRL_LOGNAME ": Got input command"));
            #endif

            if (!strcmp(ArgV(argument, 2), "1")) {
                st->requested_input = 1;
            } else if (!strcmp(ArgV(argument, 2), "2")) {
                st->requested_input = 2;
            } else if (!strcmp(ArgV(argument, 2), "3")) {
                st->requested_input = 3;
            } else if (!strcmp(ArgV(argument, 2), "0")) {
                st->requested_input = 0;
            } else {
                st->requested_input = 0;
            }

            #ifdef DEBUG_HDMI_SWITCH_CTRL
                AddLog(LOG_LEVEL_DEBUG, PSTR(HDMI_SWITCH_CTRL_LOGNAME ": input %d requested"), st->requested_input);
            #endif

            hdmi_switch_ctrl_update_mqtt(st, false);
            return serviced;
        }
    }

    return false;
}

static void hdmi_switch_ctrl_update_mqtt(struct hdmi_switch_ctrl_state_s *st,bool send) {
    ResponseAppend_P(PSTR("\"HDMI\"{"));
    ResponseAppend_P(PSTR("\"IN\":%d"), st->current_input);
    ResponseAppend_P(PSTR("\"REQ\":%d"), st->requested_input);
    ResponseJsonEnd();

    if (send) {
        MqttPublishPrefixTopicRulesProcess_P(RESULT_OR_STAT, PSTR("HDMI"));
    }
}

bool Xdrv66(uint8_t function) {

    struct hdmi_switch_ctrl_state_s *st;

    st = hdmi_switch_ctrl_sc;

    switch (function) {
        case FUNC_PRE_INIT:
            hdmi_switch_ctrl_pre_init();
            return false;
    }

    if (st == nullptr) {
        return false;
    }

    bool result = false;

    switch (function) {
        case FUNC_LOOP:
            hdmi_switch_ctrl_loop(st);
            break;
        case FUNC_COMMAND_DRIVER:
            if (XDRV_66 == XdrvMailbox.index) {
                result = hdmi_switch_ctrl_command();
            }
            break;
    }
    return result;
}

#endif