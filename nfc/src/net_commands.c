#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <x_axis.h>
#include "debug.h"
#include "FreeRTOS.h"

#include "rema.h"
#include "net_commands.h"
#include "parson.h"
#include "json_wp.h"
#include "settings.h"
#include "temperature_ds18b20.h"
#include "relay.h"

#define PROTOCOL_VERSION  	"JSON_1.0"

extern int count_z;

extern struct mot_pap x_axis;
extern struct mot_pap y_axis;
extern struct mot_pap z_axis;

typedef struct {
	char *cmd_name;
	JSON_Value* (*cmd_function)(JSON_Value const *pars);
} cmd_entry;

QueueHandle_t get_queue(const char *axis) {
	QueueHandle_t queue = NULL;

	switch (*axis) {
	case 'x':
	case 'X':
		queue = x_axis.queue;
		break;
	case 'y':
	case 'Y':
		queue = y_axis.queue;
		break;
	case 'z':
	case 'Z':
		queue = z_axis.queue;
		break;
	default:
		break;
	}
	return queue;
}

JSON_Value* telemetria_cmd(JSON_Value const *pars) {
	JSON_Value *ans = json_value_init_object();
	json_object_set_number(json_value_get_object(ans), "cuentas A",
			x_axis.pos_act / (float) x_axis.inches_to_counts_factor);
	json_object_set_number(json_value_get_object(ans), "cuentas B", 0);
	json_object_set_number(json_value_get_object(ans), "cuentas Z", 0);
	json_object_set_boolean(json_value_get_object(ans), "ZS x", x_axis.stop);

	//json_object_set_value(json_value_get_object(ans), "eje_x", mot_pap_json(&x_axis));

	return ans;

}

JSON_Value* logs_cmd(JSON_Value const *pars) {
	if (pars && json_value_get_type(pars) == JSONObject) {
		double quantity = json_object_get_number(json_value_get_object(pars),
				"quantity");

		JSON_Value *ans = json_value_init_object();
		JSON_Value *msg_array = json_value_init_array();

		int msgs_waiting = uxQueueMessagesWaiting(debug_queue);

		int extract = MIN(quantity, msgs_waiting);

		for (int x = 0; x < extract; x++) {
			char *dbg_msg = NULL;
			if (xQueueReceive(debug_queue, &dbg_msg, (TickType_t) 0) == pdPASS) {
				json_array_append_string(json_value_get_array(msg_array),
						dbg_msg);
				vPortFree(dbg_msg);
				dbg_msg = NULL;
			}
		}

		json_object_set_value(json_value_get_object(ans), "DEBUG_MSGS",
				msg_array);

		return ans;

	}
	return NULL;
}

JSON_Value* protocol_version_cmd(JSON_Value const *pars) {
	JSON_Value *ans = json_value_init_object();
	json_object_set_string(json_value_get_object(ans), "Version",
	PROTOCOL_VERSION);
	return ans;
}

JSON_Value* control_enable_cmd(JSON_Value const *pars) {
	JSON_Value *ans = json_value_init_object();

	if (json_object_has_value_of_type(json_value_get_object(pars), "enabled", JSONBoolean)) {
		bool enabled = json_object_get_boolean(json_value_get_object(pars),
				"enabled");
		rema_control_enabled_set(enabled);
	}
	json_object_set_boolean(json_value_get_object(ans), "STATUS",
				rema_control_enabled_get());
	return ans;
}

JSON_Value* stall_control_cmd(JSON_Value const *pars) {

	JSON_Value *ans = json_value_init_object();

	if (json_object_has_value_of_type(json_value_get_object(pars), "enabled", JSONBoolean)) {
		bool enabled = json_object_get_boolean(json_value_get_object(pars),
				"enabled");
		rema_stall_control_set(enabled);
	}
	json_object_set_boolean(json_value_get_object(ans), "STATUS",
				rema_stall_control_get());
	return ans;
}

JSON_Value* axis_closed_loop_cmd(JSON_Value const *pars) {
	if (pars && json_value_get_type(pars) == JSONObject) {

		char const *axis = json_object_get_string(json_value_get_object(pars),
				"axis");
		double setpoint = json_object_get_number(json_value_get_object(pars),
				"setpoint");

		struct mot_pap_msg *msg = (struct mot_pap_msg*) pvPortMalloc(
				sizeof(struct mot_pap_msg));

		msg->type = MOT_PAP_TYPE_CLOSED_LOOP;
		msg->closed_loop_setpoint = (float) setpoint;

		QueueHandle_t queue = get_queue(axis);

		if (queue && xQueueSend(queue, &msg, portMAX_DELAY) == pdPASS) {
			lDebug(Debug, " Comando enviado!");
		}
	}
	JSON_Value *ans = json_value_init_object();
	json_object_set_boolean(json_value_get_object(ans), "ACK", true);
	return ans;
}

JSON_Value* set_cal_point_cmd(JSON_Value const *pars) {
	if (pars && json_value_get_type(pars) == JSONObject) {

		double pos_x = json_object_get_number(json_value_get_object(pars),
				"position_x");
		double pos_y = json_object_get_number(json_value_get_object(pars),
				"position_y");

		mot_pap_set_position(&x_axis, pos_x);
		mot_pap_set_position(&y_axis, pos_y);
	}
	JSON_Value *ans = json_value_init_object();
	json_object_set_boolean(json_value_get_object(ans), "ACK", true);
	return ans;
}


JSON_Value* kp_set_tunings_cmd(JSON_Value const *pars) {
	JSON_Value *ans = json_value_init_object();

	if (pars && json_value_get_type(pars) == JSONObject) {

		char const *axis = json_object_get_string(json_value_get_object(pars),
				"axis");
		int kp = (int) json_object_get_number(json_value_get_object(pars),
				"kp");
		int update = (int) json_object_get_number(json_value_get_object(pars),
				"update");
		int min = (int) json_object_get_number(json_value_get_object(pars),
				"min");
		int max = (int) json_object_get_number(json_value_get_object(pars),
				"max");
		int abs_min = (int) json_object_get_number(json_value_get_object(pars),
				"abs_min");

		struct mot_pap *axis_ = NULL;

		switch (*axis) {
		case 'x':
		case 'X':
			axis_ = &x_axis;
			break;
//				case 'y':
//				case 'Y':
//						axis_ = &y_axis_;
//						break;
//				case 'z':
//				case 'Z':
//						msg->axis = &z_axis;
//					break;
		default:
			json_object_set_boolean(json_value_get_object(ans), "ACK", false);
			json_object_set_string(json_value_get_object(ans), "ERROR", "No axis specified");
			return ans;
			break;
		}

		kp_init(&(axis_->kp), kp, KP_DIRECT, update, min, max, abs_min);
		axis_->step_time = update;
		lDebug(Debug, "KP Settings set");
	}
	json_object_set_boolean(json_value_get_object(ans), "ACK", true);
	return ans;
}

JSON_Value* axis_free_run_cmd(JSON_Value const *pars) {
	if (pars && json_value_get_type(pars) == JSONObject) {

		char const *axis = json_object_get_string(json_value_get_object(pars),
				"axis");
		char const *dir = json_object_get_string(json_value_get_object(pars),
				"dir");
		double speed = json_object_get_number(json_value_get_object(pars),
				"speed");

		if (dir && speed != 0) {

			struct mot_pap_msg *msg = (struct mot_pap_msg*) pvPortMalloc(
					sizeof(struct mot_pap_msg));

			msg->type = MOT_PAP_TYPE_FREE_RUNNING;
			msg->free_run_direction = (
					strcmp(dir, "CW") == 0 ?
							MOT_PAP_DIRECTION_CW : MOT_PAP_DIRECTION_CCW);

			msg->free_run_speed = (int) speed;

			QueueHandle_t queue = get_queue(axis);

			if (queue && xQueueSend(queue, &msg, portMAX_DELAY) == pdPASS) {
				lDebug(Debug, " Comando enviado!");
			}

			lDebug(Info, "AXIS_FREE_RUN DIR: %s, SPEED: %d", dir, (int ) speed);
		}
		JSON_Value *ans = json_value_init_object();
		json_object_set_boolean(json_value_get_object(ans), "ACK", true);
		return ans;
	}
	return NULL;
}

JSON_Value* axis_stop_cmd(JSON_Value const *pars) {
	mot_pap_stop(&x_axis);
	JSON_Value *ans = json_value_init_object();
	json_object_set_boolean(json_value_get_object(ans), "ACK", true);
	return ans;
}

JSON_Value* axis_stop_all_cmd(JSON_Value const *pars) {
	mot_pap_stop(&x_axis);
//	mot_pap_stop(&y_axis);
//	mot_pap_stop(&z_axis);
	JSON_Value *ans = json_value_init_object();
	json_object_set_boolean(json_value_get_object(ans), "ACK", true);
	return ans;
}

JSON_Value* network_settings_cmd(JSON_Value const *pars) {
	if (pars && json_value_get_type(pars) == JSONObject) {
		char const *gw = json_object_get_string(json_value_get_object(pars),
				"gw");
		char const *ipaddr = json_object_get_string(json_value_get_object(pars),
				"ipaddr");
		char const *netmask = json_object_get_string(
				json_value_get_object(pars), "netmask");
		uint16_t port = (uint16_t) json_object_get_number(
				json_value_get_object(pars), "port");

		if (gw && ipaddr && netmask && port != 0) {
			lDebug(Info,
					"Received network settings: gw:%s, ipaddr:%s, netmask:%s, port:%d",
					gw, ipaddr, netmask, port);

			struct settings settings;

			unsigned char *gw_bytes = (unsigned char*) &(settings.gw.addr);
			if (sscanf(gw, "%hhu.%hhu.%hhu.%hhu", &gw_bytes[0], &gw_bytes[1],
					&gw_bytes[2], &gw_bytes[3]) == 4) {
			}

			unsigned char *ipaddr_bytes =
					(unsigned char*) &(settings.ipaddr.addr);
			if (sscanf(ipaddr, "%hhu.%hhu.%hhu.%hhu", &ipaddr_bytes[0],
					&ipaddr_bytes[1], &ipaddr_bytes[2], &ipaddr_bytes[3])
					== 4) {
			}

			unsigned char *netmask_bytes =
					(unsigned char*) &(settings.netmask.addr);
			if (sscanf(netmask, "%hhu.%hhu.%hhu.%hhu", &netmask_bytes[0],
					&netmask_bytes[1], &netmask_bytes[2], &netmask_bytes[3])
					== 4) {
			}

			settings.port = port;

			settings_save(settings);
			lDebug(Info, "Settings saved. Restarting...");

			Chip_UART_SendBlocking(DEBUG_UART, "\n\n", 2);

			Chip_RGU_TriggerReset(RGU_CORE_RST);
		}

		JSON_Value *ans = json_value_init_object();
		json_object_set_boolean(json_value_get_object(ans), "ACK", true);
		return ans;
	}
	return NULL;
}

JSON_Value* mem_info_cmd(JSON_Value const *pars) {
	JSON_Value *ans = json_value_init_object();
	json_object_set_number(json_value_get_object(ans), "MEM_TOTAL",
	configTOTAL_HEAP_SIZE);
	json_object_set_number(json_value_get_object(ans), "MEM_FREE",
			xPortGetFreeHeapSize());
	json_object_set_number(json_value_get_object(ans), "MEM_MIN_FREE",
			xPortGetMinimumEverFreeHeapSize());
	return ans;
}

JSON_Value* temperature_info_cmd(JSON_Value const *pars) {
	JSON_Value *ans = json_value_init_object();
	float temp1, temp2;
	temperature_ds18b20_get(0, &temp1);
	temperature_ds18b20_get(1, &temp2);
	json_object_set_number(json_value_get_object(ans), "TEMP1", (double) temp1);
	json_object_set_number(json_value_get_object(ans), "TEMP2", (double) temp2);
	return ans;
}

// @formatter:off
const cmd_entry cmds_table[] = {
		{
				"PROTOCOL_VERSION",		/* Command name */
				protocol_version_cmd,	/* Associated function */
		},
		{
				"CONTROL_ENABLE",
				control_enable_cmd,
		},
		{
				"STALL_CONTROL",
				stall_control_cmd,
		},
		{
				"AXIS_STOP",
				axis_stop_cmd,
		},
		{
				"AXIS_STOP_ALL",
				axis_stop_all_cmd,
		},
		{
				"AXIS_FREE_RUN",
				axis_free_run_cmd,
		},
		{
				"AXIS_CLOSED_LOOP",
				axis_closed_loop_cmd,
		},
		{
				"TELEMETRIA",
				telemetria_cmd,
		},
		{
				"LOGS",
				logs_cmd,
		},
		{
				"KP_SET_TUNINGS",
				kp_set_tunings_cmd,
		},
		{
				"NETWORK_SETTINGS",
				network_settings_cmd,
		},
		{
				"MEM_INFO",
				mem_info_cmd,
		},
		{
				"TEMPERATURE_INFO",
				temperature_info_cmd,
		},
		{
				"SET_CAL_POINT",
				set_cal_point_cmd,
		},
};
// @formatter:on

/**
 * @brief 	searchs for a matching command name in cmds_table[], passing the parameters
 * 			as a JSON object for the called function to parse them.
 * @param 	*cmd 	:name of the command to execute
 * @param   *pars   :JSON object containing the passed parameters to the called function
 */
JSON_Value* cmd_execute(char const *cmd, JSON_Value const *pars) {
	bool cmd_found = false;
	for (int i = 0; i < (sizeof(cmds_table) / sizeof(cmds_table[0])); i++) {
		if (!strcmp(cmd, cmds_table[i].cmd_name)) {
			return cmds_table[i].cmd_function(pars);
		}
	}
	if (!cmd_found) {
		lDebug(Error, "No matching command found");
	}
	return NULL;
}
