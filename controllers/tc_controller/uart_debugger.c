#include "commands_sys0.h"
#include "commands_tc1.h"
#include "papyrus_can.h"
#include "papyrus_hardware.h"
#include "papyrus_utils.h"
#include "tc_controller.h"
#include <ctype.h>
#include <iso646.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef PapyrusStatus (*CommandStringParser)(const char *, CANMessage *);

const int NUM_COMMAND_GROUPS = 2;
char *cmd_prefixes[] = {"SYS", "TC"};
uint8_t cmd_group_bases[] = {0x00, 0x10};

char *sys0_commandnames[] = {"PING",
                             "QUERY_TYPE",
                             "REASSIGN_ID",
                             "RAW_IO",
                             "RESET",
                             "NAME",
                             "NOTIFICATION",
                             "SUPPORTED",
                             "GET_ERROR",
                             "STATE",
                             "_",
                             "_",
                             "_",
                             "_",
                             "_",
                             "_"};
char *tc1_commandnames[] = {"READ", "STREAM", "STATUS", "FMT",  "_",     "_",
                            "_",    "_",      "COUNT",  "TYPE", "ALARM", "_",
                            "_",    "_",      "_",      "_"};

bool is_prefix(const char *base, char *pref) {
  return memcmp(base, pref, strlen(pref)) == 0;
}
PapyrusStatus parse_sys0_ping(const char *fmt, CANMessage *out) {
  uint8_t data_byte = atoi(fmt);
  papyrus_prep_theader(&out->tHeader);
  out->tHeader.DataLength = 2;
  out->msg.command_id = SYSCMD_PING;
  out->msg.short_args[0] = data_byte;
  return PAPYRUS_OK;
}
PapyrusStatus parse_sys0_query_type(const char *fmt, CANMessage *out) {
  UNUSED(fmt);
  papyrus_prep_theader(&out->tHeader);
  out->tHeader.DataLength = 1;
  out->msg.command_id = SYSCMD_QUERY_TYPE;
  return PAPYRUS_OK;
}
void skip_word_ws(const char **base, char *word) {
  *base += strlen(word);
  while (isspace((int)**base)) {
    (*base)++;
  }
}
void until_next_word(const char **base) {
  while (!isspace((int)**base)) {
    (*base)++;
  }
  while (isspace((int)**base)) {
    (*base)++;
  }
}
PapyrusStatus parse_tc1_read(const char *fmt, CANMessage *out) {
  bool use_cjc = false;
  if (is_prefix(fmt, "cjc")) {
    use_cjc = true;
    skip_word_ws(&fmt, "cjc");
  }
  uint8_t tc_id;
  if (is_prefix(fmt, "all")) {
    tc_id = 0;
  } else {
    tc_id = atoi(fmt);
    if (tc_id == 0) {
      return PAPYRUS_ERROR_PARAM;
    }
  }
  papyrus_prep_theader(&out->tHeader);
  out->tHeader.DataLength = 2;
  out->msg.command_id = TCCMD_READ;
  out->msg.short_args[0] = ((int)use_cjc ? 0x80 : 0) | tc_id;
  return PAPYRUS_OK;
}
PapyrusStatus parse_tc1_stream(const char *fmt, CANMessage *out) {
  uint8_t tc_id;
  if (is_prefix(fmt, "all")) {
    tc_id = 0;
  } else {
    tc_id = atoi(fmt);
    if (tc_id == 0) {
      return PAPYRUS_ERROR_PARAM;
    }
  }
  until_next_word(&fmt);
  uint16_t timer;
  if (is_prefix(fmt, "off")) {
    timer = 0;
  } else {
    timer = atoi(fmt);
  }
  papyrus_prep_theader(&out->tHeader);
  out->tHeader.DataLength = 4;
  out->msg.command_id = TCCMD_STREAM;
  out->msg.short_args[0] = tc_id;
  out->msg.short_args[1] = timer & 0xFF;
  out->msg.short_args[2] = timer >> 8;
  return PAPYRUS_OK;
}
PapyrusStatus parse_tc1_status(const char *fmt, CANMessage *out) {
  uint8_t tc_id;
  if (is_prefix(fmt, "all")) {
    tc_id = 0;
  } else {
    tc_id = atoi(fmt);
    if (tc_id == 0) {
      return PAPYRUS_ERROR_PARAM;
    }
  }
  papyrus_prep_theader(&out->tHeader);
  out->tHeader.DataLength = 2;
  out->msg.command_id = TCCMD_STATUS;
  out->msg.short_args[0] = tc_id;
  return PAPYRUS_OK;
}
PapyrusStatus parse_tc1_fmt(const char *fmt, CANMessage *out) {
  uint8_t tc_id;
  if (is_prefix(fmt, "all")) {
    tc_id = 0;
  } else {
    tc_id = atoi(fmt);
    if (tc_id == 0) {
      return PAPYRUS_ERROR_PARAM;
    }
  }
  until_next_word(&fmt);
  TCReadFormat rfmt;
  if (is_prefix(fmt, "int8s")) {
    rfmt = TC_FMT_INT8_SHIFT;
  } else if (is_prefix(fmt, "int16")) {
    rfmt = TC_FMT_INT16;
  } else if (is_prefix(fmt, "fixed88")) {
    rfmt = TC_FMT_FIXED88;
  } else if (is_prefix(fmt, "fixed1616")) {
    rfmt = TC_FMT_FIXED1616;
  } else {
    return PAPYRUS_ERROR_PARAM;
  }
  papyrus_prep_theader(&out->tHeader);
  out->tHeader.DataLength = 3;
  out->msg.command_id = TCCMD_FMT;
  out->msg.short_args[0] = tc_id;
  out->msg.short_args[1] = rfmt;
  return PAPYRUS_OK;
}
PapyrusStatus parse_tc1_count(const char *fmt, CANMessage *out) {
  uint8_t tc_count;
  if (isdigit((int)*fmt)) {
    tc_count = atoi(fmt);
  } else {
    tc_count = 0xFF;
  }
  papyrus_prep_theader(&out->tHeader);
  out->tHeader.DataLength = 2;
  out->msg.command_id = TCCMD_COUNT;
  out->msg.short_args[0] = tc_count;
  return PAPYRUS_OK;
}
PapyrusStatus parse_tc1_type(const char *fmt, CANMessage *out) {
  bool do_set;
  if (is_prefix(fmt, "set")) {
    do_set = true;
  } else if (is_prefix(fmt, "get")) {
    do_set = false;
  } else {
    return PAPYRUS_ERROR_PARAM;
  }
  until_next_word(&fmt);
  uint8_t tc_id;
  if (is_prefix(fmt, "all")) {
    tc_id = 0;
  } else {
    tc_id = atoi(fmt);
    if (tc_id == 0) {
      return PAPYRUS_ERROR_PARAM;
    }
  }
  TCType set_to = 0;
  if (do_set) {
    until_next_word(&fmt);
    if (is_prefix(fmt, "K"))
      set_to = TC_TYPE_K;
    else if (is_prefix(fmt, "J"))
      set_to = TC_TYPE_J;
    else if (is_prefix(fmt, "T"))
      set_to = TC_TYPE_T;
    else if (is_prefix(fmt, "E"))
      set_to = TC_TYPE_E;
    else if (is_prefix(fmt, "N"))
      set_to = TC_TYPE_N;
    else if (is_prefix(fmt, "S"))
      set_to = TC_TYPE_S;
    else if (is_prefix(fmt, "R"))
      set_to = TC_TYPE_R;
    else if (is_prefix(fmt, "B"))
      set_to = TC_TYPE_B;
    else
      return PAPYRUS_ERROR_PARAM;
  }
  papyrus_prep_theader(&out->tHeader);
  out->tHeader.DataLength = (int)do_set ? 3 : 2;
  out->msg.command_id = TCCMD_TYPE;
  out->msg.short_args[0] = ((int)do_set ? 0x00 : 0x80) | tc_id;
  out->msg.short_args[1] = set_to;
  return PAPYRUS_OK;
}
PapyrusStatus parse_tc1_alarm(const char *fmt, CANMessage *out) {
  uint8_t alm_type;
  if (is_prefix(fmt, "low"))
    alm_type = 0;
  else if (is_prefix(fmt, "high"))
    alm_type = 1;
  else if (is_prefix(fmt, "cjc"))
    alm_type = 2;
  else
    return PAPYRUS_ERROR_PARAM;
  until_next_word(&fmt);
  uint8_t tc_id = 0;
  if (alm_type != 2) {
    if (is_prefix(fmt, "all")) {
      tc_id = 0;
    } else {
      tc_id = atoi(fmt);
      if (tc_id == 0) {
        return PAPYRUS_ERROR_PARAM;
      }
    }
  }
  until_next_word(&fmt);
  float alm_val = atof(fmt);
  fixed32 alm_fixed = float_to_fixed(alm_val);
  papyrus_prep_theader(&out->tHeader);
  out->tHeader.DataLength = 6;
  out->msg.command_id = TCCMD_ALARM;
  out->msg.short_args[0] = (alm_type << 6) | tc_id;
  for (uint8_t i = 0; i < 4; i++) {
    out->msg.short_args[1 + i] = alm_fixed & 0xFF;
    alm_fixed >>= 8;
  }
  return PAPYRUS_OK;
}
CommandStringParser sys0_parsers[] = {parse_sys0_ping, parse_sys0_query_type};
CommandStringParser tc1_parsers[] = {
    parse_tc1_read,  parse_tc1_stream, parse_tc1_status, parse_tc1_fmt,
    nullptr,         nullptr,          nullptr,          nullptr,
    parse_tc1_count, parse_tc1_type,   parse_tc1_alarm};
CommandStringParser *cmd_group_parsers[] = {sys0_parsers, tc1_parsers};
char **cmd_group_names[] = {sys0_commandnames, tc1_commandnames};

char read_buf[128];
char read_buf_raw[128];

void papyrus_process_can_other(CANMessage *msg, ControllerBase *base,
                               ErrorEntry *err) {

  UNUSED(base);
  UNUSED(err);
  MsgType mtype = CAN_MESSAGE_TYPE(msg->rHeader.Identifier);
  switch (mtype) {
  case MSG_TYPE_RESPONSE:
    printf("Received response: ");
    for (uint32_t i = 0; i < msg->rHeader.DataLength; i++) {
      printf("%02x ", msg->msg.raw_data[i]);
    }
    printf("\r\n");
    break;
  case MSG_TYPE_BULKDATA:
    break;
  case MSG_TYPE_STREAM:
    printf("Stream data: ");
    for (uint32_t i = 0; i < msg->rHeader.DataLength; i++) {
      printf("%02x ", msg->msg.raw_data[i]);
    }
    printf("\r\n");
    break;
  case MSG_TYPE_NOTIF:
    break;
  case MSG_TYPE_ERROR:
    break;
  default:
    break;
  }
}

int isterm(char c) { return c == 0 || isspace(c); }
bool decode_command(char **cmd, uint8_t *dec_id, CommandStringParser *parse) {
  for (uint8_t i = 0; i < NUM_COMMAND_GROUPS; i++) {
    if ((int)is_prefix(*cmd, cmd_prefixes[i]) &&
        (*cmd)[strlen(cmd_prefixes[i])] == '/') {
      *cmd += strlen(cmd_prefixes[i]) + 1;
      char **my_grp = cmd_group_names[i];
      for (uint8_t j = 0; j < 16; j++) {
        if ((int)is_prefix(*cmd, my_grp[j]) &&
            isterm((*cmd)[strlen(my_grp[j])])) {
          *dec_id = cmd_group_bases[i] | j;
          *parse = cmd_group_parsers[i][j];
          *cmd += strlen(my_grp[j]) + 1;
          return true;
        }
      }
    }
  }
  return false;
}

void uart_debugger(TCController *this) {
  setvbuf(stdin, nullptr, _IONBF, 0);
  UNUSED(this);
  HAL_Delay(1200);
  printf("Welcome to UART DEBUGGER\r\n");
  for (;;) {
    fgets(read_buf_raw, 126, stdin);
    int j = 0;
    for (int i = 0; i < 128; i++) {
      if (read_buf_raw[i] && !isprint((int)read_buf_raw[i]))
        continue;
      read_buf[j] = read_buf_raw[i];
      j++;
    }
    PapyrusStatus stat;
    CANMessage tmpmsg;
    for (char *ptr = read_buf; !isterm(*ptr) && *ptr; ptr++) {
      *ptr = toupper(*ptr);
    }
    printf("> %s\r\n", read_buf);
    uint8_t dec_id;
    CommandStringParser dec_parser;
    char *read_ptr = read_buf;
    if (decode_command(&read_ptr, &dec_id, &dec_parser)) {
      stat = dec_parser(read_ptr, &tmpmsg);
      if (stat == PAPYRUS_OK) {
        printf("Sending a message: ");
        for (uint32_t i = 0; i < tmpmsg.tHeader.DataLength; i++) {
          printf("%02x ", tmpmsg.msg.raw_data[i]);
        }
        printf("\r\n");
        if (HAL_FDCAN_AddMessageToTxFifoQ(&this->base.can.handle,
                                          &tmpmsg.tHeader,
                                          tmpmsg.msg.raw_data) != HAL_OK) {
          printf("Send failed!\r\n");
        }
      } else {
        printf("Parse/generate error\r\n");
      }
    } else {
      printf("No such command\r\n");
    }
  }
}
