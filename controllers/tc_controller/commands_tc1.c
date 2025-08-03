#include "commands_tc1.h"
#include "papyrus_can.h"
#include "papyrus_hardware.h"
#include "papyrus_utils.h"
#include "tc_amplifier_max31855.h"
#include "tc_controller.h"
#include <stdlib.h>

const uint8_t TC_READ_FORMAT_SIZE[] = {1, 2, 2, 4};

void load_ctable_tc1(CommandRunner *runs, CommandResponder *resps,
                     CANMsgLen *lens) {
  runs[TCCMD_READ] = run_tc1_read;
  resps[TCCMD_READ] = resp_tc1_read;
  lens[TCCMD_READ] = MSGLEN_SHORT;
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
  if (read_cjc) {
    *out = tc_out.cjc_fixed;
  } else {
    *out = tc_out.tc_fixed;
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
    TCReadFormat fmt = tc->tc_config.tc_read_format[rd_idx];
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
