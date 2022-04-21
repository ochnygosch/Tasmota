#ifdef USE_RECEIVER_CTRL

#define XDRV_63 63

enum receiver_ctrl_dev_state_pwr_e : uint8_t {
	RECEIVER_CTRL_DEV_STATE_UNKNOWN=0,
	RECEIVER_CTRL_DEV_STATE_PWR_ON,
	RECEIVER_CTRL_DEV_STATE_PWR_OFF
};

enum receiver_ctrl_dev_state_input_e : uint8_t {
	RECEIVER_CTRL_DEV_STATE_UNKNOW=0,
	RECEIVER_CTRL_DEV_STATE_CD,
	RECEIVER_CTRL_DEV_STATE_DVD,
	RECEIVER_CTRL_DEV_STATE_VAUX
};

#include "xdrv_63_receiver_ctrl.h"

struct receiver_ctrl_softc_s {
	TasmotaSerial	*sc_serial;
	uint8_t		sc_dev_volume;
	enum receiver_ctrl_dev_state_pwr_e	sc_dev_state_pwr;
	enum receiver_ctrl_dev_state_input_e	sc_dev_state_input;
} __packed;

static struct receiver_ctrl_softc_s *receiver_ctrl_sc = nullptr;




#endif
