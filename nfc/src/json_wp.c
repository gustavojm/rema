#include <stdlib.h>
#include <string.h>
#include "debug.h"

#include "parson.h"
#include "json_wp.h"
#include "net_commands.h"

/**
 * @brief Defines a simple wire protocol base on JavaScript Object Notation (JSON)
 * @details Receives an JSON object containing an array of commands under the
 * "commands" key. Every command entry in this array must be a JSON object, containing
 * a "command" string specifying the name of the called command, and a nested JSON object
 * with the parameters for the called command under the "pars" key.
 * The called function must extract the parameters from the passed JSON object.
 *
 * Example of the received JSON object:
 *
 * {
 "commands" : [
 {"command": "AXIS_FREE_RUN",
 "pars": {
 "axis": "X",
 "dir": "CW",
 "speed": 8
 }
 },
 {"command": "LOGS",
 "pars": {
 "quantity": 10
 }
 }
 ]
 }

 * Every executed command has the chance of returning a JSON object that will be inserted
 * in the response JSON object under a key corresponding to the executed command name, or
 * NULL if no answer is expected.
 */

/**
 * @brief 	Parses the received JSON object looking for commands to execute and appends
 * 			the outputs of the called commands to the response buffer.
 * @param 	*rx_buff 	:pointer to the received buffer from the network
 * @param   **tx_buff	:pointer to pointer, will be set to the allocated return buffer
 * @returns	the length of the allocated response buffer
 */
int json_wp(char *rx_buff, char **tx_buff) {
	JSON_Value *rx_JSON_value = json_parse_string(rx_buff);
	JSON_Value *tx_JSON_value = json_value_init_object();
	*tx_buff = NULL;
	int buff_len = 0;

	lDebug(Info, "%s", rx_buff);

	if (!rx_JSON_value || (json_value_get_type(rx_JSON_value) != JSONObject)) {
		lDebug(Error, "Error json parse.");
	} else {
		JSON_Object *rx_JSON_object = json_value_get_object(rx_JSON_value);
		JSON_Array *commands = json_object_get_array(rx_JSON_object,
				"commands");

		if (commands != NULL) {
			for (int i = 0; i < json_array_get_count(commands); i++) {
				JSON_Object *command = json_array_get_object(commands, i);
				char const *command_name = json_object_get_string(command,
						"command");
				lDebug(Info, "Command Found: %s", command_name);
				JSON_Value *pars = json_object_get_value(command, "pars");

				JSON_Value *ans = cmd_execute(command_name, pars);
				if (ans) {
					json_object_set_value(json_value_get_object(tx_JSON_value),
							command_name, ans);
				}
			}
		}

		buff_len = json_serialization_size(tx_JSON_value); /* returns 0 on fail */
		*tx_buff = pvPortMalloc(buff_len);
		if (!(*tx_buff)) {
			lDebug(Error, "Out Of Memory");
			buff_len = 0;
		} else {
			json_serialize_to_buffer(tx_JSON_value, *tx_buff, buff_len);
		}
	}
	json_value_free(rx_JSON_value);
	json_value_free(tx_JSON_value);
	return buff_len;
}
