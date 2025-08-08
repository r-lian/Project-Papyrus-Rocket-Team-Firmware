#include "commands_tc1.h"
#include "papyrus_can.h"
#include "papyrus_hardware.h"
#include "papyrus_utils.h"
#include "stm32c0xx_hal_fdcan.h"
#include "tc_amplifier_max31855.h"
#include "tc_controller.h"
#include <stdio.h>
#include <stdlib.h>

const uint8_t TC_READ_FORMAT_SIZE[] = {1, 2, 2, 4};

void load_ctable_tc1(CommandRunner *runs, CommandResponder *resps,
                     CANMsgLen *lens) {
  runs[TCCMD_READ] = run_tc1_read;
  resps[TCCMD_READ] = resp_tc1_read;
  lens[TCCMD_READ] = MSGLEN_SHORT;
  runs[TCCMD_STREAM] = run_tc1_stream;
  resps[TCCMD_STREAM] = resp_tc1_stream;
  lens[TCCMD_STREAM] = MSGLEN_SHORT;
  runs[TCCMD_STATUS] = run_tc1_status;
  resps[TCCMD_STATUS] = resp_tc1_status;
  lens[TCCMD_STATUS] = MSGLEN_SHORT;
  runs[TCCMD_FMT] = run_tc1_fmt;
  resps[TCCMD_FMT] = resp_tc1_fmt;
  lens[TCCMD_FMT] = MSGLEN_SHORT;
  runs[TCCMD_COUNT] = run_tc1_count;
  resps[TCCMD_COUNT] = resp_tc1_count;
  lens[TCCMD_COUNT] = MSGLEN_SHORT;
  runs[TCCMD_TYPE] = run_tc1_type;
  resps[TCCMD_TYPE] = resp_tc1_type;
  lens[TCCMD_TYPE] = MSGLEN_SHORT;
  runs[TCCMD_ALARM] = run_tc1_alarm;
  resps[TCCMD_ALARM] = resp_tc1_alarm;
  lens[TCCMD_ALARM] = MSGLEN_SHORT;
}

PapyrusStatus run_tc1_read(CANMessage *msg, ControllerBase *controller,
                           ErrorEntry *err) {
  UNUSED(msg);
  UNUSED(controller);
  err->err = ERROR_NONE;
  err->target = 0;
  return PAPYRUS_OK;
}
PapyrusStatus read_indiv_tc(TCController *tcc, uint8_t idx, TCTempRead *out,
                            ErrorEntry *err, bool read_cjc) {
  if (idx > tcc->tc_config.num_tcs_active) {
    err->err = ERROR_PARAM_RANGE;
    err->target = 0;
    return PAPYRUS_ERROR_PARAM;
  }
  idx--;
  MAX31855Output tc_out;
  PapyrusStatus spi_stat = max31855_read_tc(&tcc->tc_spis[idx], &tc_out);
  tcc->last_tc_read[idx] = tc_out.tc_fixed;
  tcc->last_cjc_read[idx] = tc_out.cjc_fixed;
  tcc->last_err_flags[idx] = tc_out.err_flags;
  if (spi_stat != PAPYRUS_OK) {
    if (tc_out.err_flags & TC_FAULT_SPI_FAILED) {
      err->err = ERROR_INTERFACE;
    } else {
      err->err = ERROR_HARDWARE;
    }
    err->target = idx;
    return PAPYRUS_ERROR_HARDWARE;
  }
  if (out != NULL) {
    if (read_cjc) {
      *out = tc_out.cjc_fixed;
    } else {
      *out = tc_out.tc_fixed;
    }
  }
  return PAPYRUS_OK;
}
PapyrusStatus resp_tc1_read(CANMessage *msg, CANMessage *resp,
                            ControllerBase *controller, ErrorEntry *err) {
  papyrus_prep_theader(&resp->tHeader);
  resp->tHeader.Identifier =
      CAN_GENERATE_ID(controller->board_id, MSG_TYPE_RESPONSE);
  TCController *tc = (TCController *)controller;
  uint8_t data_buf[32] = {0};
  uint8_t *data_ptr = data_buf;
  bool read_cjc = (msg->msg.short_args[0] & 0x80) != 0;
  uint8_t rd_idx = msg->msg.short_args[0] & 0x7F;
  bool read_all = rd_idx == 0;
  if (read_all)
    rd_idx++;
  bool any_failed = false;
  while (rd_idx <= tc->tc_config.num_tcs_active) {
    TCTempRead last;
    PapyrusStatus read_stat = read_indiv_tc(tc, rd_idx, &last, err, read_cjc);
    if (read_stat != PAPYRUS_OK) {
      any_failed = true;
      last = 0xFFFFFFFF;
    }
    TCReadFormat fmt = tc->tc_config.tc_read_format[rd_idx - 1];
    uint32_t cvt = to_tc_format(last, fmt);
    for (uint8_t i = 0; i < TC_READ_FORMAT_SIZE[fmt]; i++) {
      *(data_ptr++) = cvt & 0xFF;
      cvt >>= 8;
    }
    if (!read_all)
      break;
    rd_idx++;
  }
  ptrdiff_t buf_len = data_ptr - data_buf;
  for (uint8_t i = 0; i < 7; i++) {
    resp->msg.long_data[i] = data_buf[i];
  }
  resp->msg.arg_block_id = 0;
  resp->tHeader.DataLength = MIN(8, buf_len + 1);
  if (buf_len > 7) {
    CANMessage *child = malloc(sizeof(CANMessage));
    if (child == NULL) {
      err->err = ERROR_MEMORY;
      err->target = 0;
      return PAPYRUS_ERROR_MEMORY;
    }
    papyrus_prep_theader(&child->tHeader);
    child->tHeader.Identifier =
        CAN_GENERATE_ID(controller->board_id, MSG_TYPE_RESPONSE);
    child->tHeader.DataLength = buf_len - 6;
    for (uint8_t i = 7; i < buf_len - 7; i++) {
      child->msg.long_data[i - 7] = data_buf[i];
    }
    resp->next = child;
    child->next = nullptr;
    child->msg.arg_block_id = 0x81;
  } else {
    resp->msg.arg_block_id |= 0x80;
  }
  if (any_failed) {
    if (err->err == ERROR_PARAM_RANGE)
      return PAPYRUS_ERROR_PARAM;
    return PAPYRUS_ERROR_HARDWARE;
  }
  return PAPYRUS_OK;
}

PapyrusStatus run_tc1_stream(CANMessage *msg, ControllerBase *controller,
                             ErrorEntry *err) {
  TCController *tc = (TCController *)controller;
  uint8_t op_idx = msg->msg.short_args[0];
  if (op_idx > tc->tc_config.num_tcs_active) {
    err->err = ERROR_PARAM_RANGE;
    err->target = 0;
    return PAPYRUS_ERROR_PARAM;
  }
  bool read_all = op_idx == 0;
  if (!read_all)
    op_idx--;
  uint16_t delay_ms = msg->msg.short_args[1] | (msg->msg.short_args[2] << 8);
  while (op_idx < tc->tc_config.num_tcs_active) {
    tc->tc_config.stream_enabled[op_idx] = delay_ms != 0;
    tc->tc_config.stream_reload[op_idx] = delay_ms;
    tc->tc_config.stream_timer[op_idx] = delay_ms;
    if (!read_all)
      break;
    op_idx++;
  }
  err->err = ERROR_NONE;
  err->target = 0;
  return PAPYRUS_OK;
}
PapyrusStatus resp_tc1_stream(CANMessage *msg, CANMessage *resp,
                              ControllerBase *controller, ErrorEntry *err) {
  UNUSED(msg);
  papyrus_prep_theader(&resp->tHeader);
  resp->tHeader.DataLength = 0;
  resp->tHeader.Identifier =
      CAN_GENERATE_ID(controller->board_id, MSG_TYPE_RESPONSE);
  UNUSED(err);
  return PAPYRUS_OK;
}
void tc_controller_mstick(TCController *tc_ctrl) {
  uint8_t stream_buf[8];
  for (uint8_t i = 0; i < tc_ctrl->tc_config.num_tcs_active; i++) {
    if ((int)tc_ctrl->tc_config.stream_enabled[i] &&
        tc_ctrl->tc_config.stream_timer[i] == 0) {
      stream_buf[0] = i + 1;
      tc_ctrl->tc_config.stream_timer[i] = tc_ctrl->tc_config.stream_reload[i];
      TCTempRead out;
      ErrorEntry err;
      err.err = ERROR_NONE;
      err.target = 0;
      PapyrusStatus stat = read_indiv_tc(tc_ctrl, i + 1, &out, &err, false);
      if (stat == PAPYRUS_OK) {
        TCReadFormat fmt = tc_ctrl->tc_config.tc_read_format[i];
        uint32_t cvt = to_tc_format(out, fmt);
        for (uint8_t j = 0; j < TC_READ_FORMAT_SIZE[fmt]; j++) {
          stream_buf[j + 1] = cvt & 0xFF;
          cvt >>= 8;
        }
        CANMessage stream;
        papyrus_prep_theader(&stream.tHeader);
        stream.tHeader.Identifier =
            CAN_GENERATE_ID(tc_ctrl->base.board_id, MSG_TYPE_STREAM);
        stream.tHeader.DataLength = TC_READ_FORMAT_SIZE[fmt] + 1;
        if (HAL_FDCAN_AddMessageToTxFifoQ(&tc_ctrl->base.can.handle,
                                          &stream.tHeader,
                                          &stream_buf[0]) != HAL_OK) {
          err.err = ERROR_HARDWARE;
          err.target = 0;
        }
      }
    } else if (tc_ctrl->tc_config.stream_enabled[i]) {
      tc_ctrl->tc_config.stream_timer[i]--;
    }
  }
}
PapyrusStatus run_tc1_status(CANMessage *msg, ControllerBase *controller,
                             ErrorEntry *err) {
  UNUSED(msg);
  UNUSED(controller);
  err->err = ERROR_NONE;
  err->target = 0;
  return PAPYRUS_OK;
}
PapyrusStatus resp_tc1_status(CANMessage *msg, CANMessage *resp,
                              ControllerBase *controller, ErrorEntry *err) {
  papyrus_prep_theader(&resp->tHeader);
  resp->tHeader.Identifier =
      CAN_GENERATE_ID(controller->board_id, MSG_TYPE_RESPONSE);
  TCController *tc = (TCController *)controller;
  uint8_t rd_idx = msg->msg.short_args[0];
  bool read_all = rd_idx == 0;
  if (read_all)
    rd_idx++;
  uint8_t data_buf[8] = {0};
  uint8_t *data_ptr = data_buf;
  while (rd_idx <= tc->tc_config.num_tcs_active) {
    read_indiv_tc(tc, rd_idx, nullptr, err, false);
    *(data_ptr++) = tc->last_err_flags[rd_idx - 1];
    if (!read_all)
      break;
    rd_idx++;
  }
  ptrdiff_t buf_len = data_ptr - data_buf;
  for (uint8_t i = 0; i < 7; i++) {
    resp->msg.short_data[i] = data_buf[i];
  }
  resp->tHeader.DataLength = buf_len;
  return PAPYRUS_OK;
}
PapyrusStatus run_tc1_fmt(CANMessage *msg, ControllerBase *controller,
                          ErrorEntry *err) {
  TCController *tc = (TCController *)controller;
  uint8_t op_idx = msg->msg.short_args[0];
  TCReadFormat fmt = msg->msg.short_args[1];
  if (op_idx > tc->tc_config.num_tcs_active || fmt > TC_FMT_FIXED1616) {
    err->err = ERROR_PARAM_RANGE;
    err->target = 0;
    return PAPYRUS_ERROR_PARAM;
  }
  bool read_all = op_idx == 0;
  if (!read_all)
    op_idx--;
  while (op_idx < tc->tc_config.num_tcs_active) {
    tc->tc_config.tc_read_format[op_idx] = fmt;
    if (!read_all)
      break;
    op_idx++;
  }
  err->err = ERROR_NONE;
  err->target = 0;
  return PAPYRUS_OK;
}
PapyrusStatus resp_tc1_fmt(CANMessage *msg, CANMessage *resp,
                           ControllerBase *controller, ErrorEntry *err) {
  UNUSED(msg);
  papyrus_prep_theader(&resp->tHeader);
  resp->tHeader.DataLength = 0;
  resp->tHeader.Identifier =
      CAN_GENERATE_ID(controller->board_id, MSG_TYPE_RESPONSE);
  UNUSED(err);
  return PAPYRUS_OK;
}
PapyrusStatus run_tc1_count(CANMessage *msg, ControllerBase *controller,
                            ErrorEntry *err) {
  TCController *tc = (TCController *)controller;
  uint8_t set_to = msg->msg.short_args[0];
  if (set_to != 0xFF) {
    if (set_to > TC_MAX_CHANNELS) {
      err->err = ERROR_PARAM_RANGE;
      err->target = 0;
      return PAPYRUS_ERROR_PARAM;
    }
    tc->tc_config.num_tcs_active = set_to;
  }
  err->err = ERROR_NONE;
  err->target = 0;
  return PAPYRUS_OK;
}
PapyrusStatus resp_tc1_count(CANMessage *msg, CANMessage *resp,
                             ControllerBase *controller, ErrorEntry *err) {
  TCController *tc = (TCController *)controller;
  uint8_t real_count = 0;
  for (uint8_t i = 0; i < TC_MAX_CHANNELS; i++) {
    if (read_indiv_tc(tc, i + 1, nullptr, err, false) != PAPYRUS_OK) {
      break;
    }
    real_count++;
  }
  UNUSED(msg);
  papyrus_prep_theader(&resp->tHeader);
  resp->tHeader.DataLength = 1;
  resp->msg.short_data[0] = real_count;
  resp->tHeader.Identifier =
      CAN_GENERATE_ID(controller->board_id, MSG_TYPE_RESPONSE);
  UNUSED(err);
  return PAPYRUS_OK;
}
PapyrusStatus run_tc1_type(CANMessage *msg, ControllerBase *controller,
                           ErrorEntry *err) {
  TCController *tc = (TCController *)controller;
  uint8_t arg = msg->msg.short_args[0];
  uint8_t tc_id = arg & 0x7F;
  uint8_t set_to = msg->msg.short_args[1];
  if ((arg & 0x80) == 0) {
    if (tc_id >= TC_MAX_CHANNELS || (set_to > TC_TYPE_B)) {
      err->err = ERROR_PARAM_RANGE;
      err->target = 0;
      return PAPYRUS_ERROR_PARAM;
    }
    if (tc_id == 0) {
      for (uint8_t i = 0; i < tc->tc_config.num_tcs_active; i++) {
        tc->tc_config.tc_type[i] = set_to;
      }
    } else {
      tc->tc_config.tc_type[tc_id - 1] = set_to;
    }
  }
  err->err = ERROR_NONE;
  err->target = 0;
  return PAPYRUS_OK;
}
PapyrusStatus resp_tc1_type(CANMessage *msg, CANMessage *resp,
                            ControllerBase *controller, ErrorEntry *err) {
  TCController *tc = (TCController *)controller;
  uint8_t arg = msg->msg.short_args[0];
  uint8_t tc_id = arg & 0x7F;
  uint8_t data_buf[8] = {0};
  uint8_t *data_ptr = data_buf;
  if ((arg & 0x80) == 0x80) {
    if (tc_id >= TC_MAX_CHANNELS) {
      err->err = ERROR_PARAM_RANGE;
      err->target = 0;
      return PAPYRUS_ERROR_PARAM;
    }
    if (tc_id == 0) {
      for (uint8_t i = 0; i < tc->tc_config.num_tcs_active; i++) {
        *(data_ptr++) = tc->tc_config.tc_type[i];
      }
    } else {
      *(data_ptr++) = tc->tc_config.tc_type[tc_id - 1];
    }
  }
  papyrus_prep_theader(&resp->tHeader);
  ptrdiff_t buf_len = data_ptr - data_buf;
  for (uint8_t i = 0; i < buf_len; i++) {
    resp->msg.short_data[i] = data_buf[i];
  }
  resp->tHeader.DataLength = buf_len;
  resp->tHeader.Identifier =
      CAN_GENERATE_ID(controller->board_id, MSG_TYPE_RESPONSE);
  return PAPYRUS_OK;
}
PapyrusStatus run_tc1_alarm(CANMessage *msg, ControllerBase *controller,
                            ErrorEntry *err) {
  TCController *tc = (TCController *)controller;
  uint8_t arg = msg->msg.short_args[0];
  uint8_t tc_id = arg & 0x3F;
  uint8_t alm_type = arg >> 6;
  if (alm_type == 2)
    tc_id = 0;
  if (tc_id >= TC_MAX_CHANNELS) {
    err->err = ERROR_PARAM_RANGE;
    err->target = 0;
    return PAPYRUS_ERROR_PARAM;
  }
  uint32_t fmtted = msg->msg.short_args[1];
  fmtted |= msg->msg.short_args[2] << 8;
  fmtted |= msg->msg.short_args[3] << 16;
  fmtted |= msg->msg.short_args[4] << 24;
  TCTempRead alm_val = from_tc_format(
      fmtted, (tc_id == 0) ? TC_FMT_FIXED1616
                           : tc->tc_config.tc_read_format[tc_id - 1]);
  if (alm_type == 2) {
    tc->tc_config.cjc_diff_alarm = alm_val;
  } else {
    bool read_all = tc_id == 0;
    uint8_t op_idx = tc_id;
    if (!read_all)
      op_idx--;
    while (op_idx < tc->tc_config.num_tcs_active) {
      if (alm_type == 1) {
        tc->tc_config.temp_alarm_high_c[op_idx] = alm_val;
      } else {
        tc->tc_config.temp_alarm_low_c[op_idx] = alm_val;
      }
      if (!read_all)
        break;
      op_idx++;
    }
  }
  return PAPYRUS_OK;
}
PapyrusStatus resp_tc1_alarm(CANMessage *msg, CANMessage *resp,
                             ControllerBase *controller, ErrorEntry *err) {
  UNUSED(msg);
  papyrus_prep_theader(&resp->tHeader);
  resp->tHeader.DataLength = 0;
  resp->tHeader.Identifier =
      CAN_GENERATE_ID(controller->board_id, MSG_TYPE_RESPONSE);
  UNUSED(err);
  return PAPYRUS_OK;
}
