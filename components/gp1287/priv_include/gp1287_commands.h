/*
 * gp1287_commands.h
 *
 *  Created on: 2024-12-30
 *      Author: koshm
 *
 * SPDX-License-Identifier: GPL-3.0-only
 */

#pragma once

#ifndef COMPONENTS_GP1287_PRIV_INCLUDE_GP1287_COMMANDS_H_
#define COMPONENTS_GP1287_PRIV_INCLUDE_GP1287_COMMANDS_H_

#define GP1287_CMD_RESET            (0xAA)
#define GP1287_CMD_MEMCLR           (0x08)
#define GP1287_CMD_SET_OSC          (0x78)
#define GP1287_CMD_SET_DIMMING      (0xA0)
#define GP1287_CMD_SET_VFDMODE      (0xCC)
#define GP1287_CMD_SET_DISPMODE     (0x80)
#define GP1287_CMD_SET_DISPAREA     (0xE0)
#define GP1287_CMD_WRITE_GRAM       (0xF0)
#define GP1287_CMD_SET_SPEED        (0xB1)
#define GP1287_CMD_SET_INT          (0x08)
#define GP1287_CMD_STANDBY          (0x61)
#define GP1287_CMD_WAKEUP           (0x6D)

#endif /* COMPONENTS_GP1287_PRIV_INCLUDE_GP1287_COMMANDS_H_ */
