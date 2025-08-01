#include "commands_sys0.h"
#include "papyrus_can.h"
#include "papyrus_utils.h"
#include "tc_controller.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef PapyrusStatus (*CommandStringParser)(const char *, CANMessage *);

char *sys0_commandnames[] = {
    "PING", "QUERY_TYPE",   "REASSIGN_ID", "RAW_IO",    "RESET",
    "NAME", "NOTIFICATION", "SUPPORTED",   "GET_ERROR", "STATE"};
PapyrusStatus parse_sys0_ping(const char *fmt, CANMessage *out) {
  UNUSED(fmt);
  fmt += 4;
  while (isspace((int)*fmt))
    fmt++;
  uint8_t data_byte = atoi(fmt);
  return gen_sys0_ping(out, data_byte);
}
PapyrusStatus parse_sys0_query_type(const char *fmt, CANMessage *out) {
  UNUSED(fmt);
  return gen_sys0_query_type(out);
}
CommandStringParser sys0_parsers[] = {parse_sys0_ping, parse_sys0_query_type};

char read_buf[128];

void uart_debugger(TCController *this) {
  setvbuf(stdin, nullptr, _IONBF, 0);
  UNUSED(this);
  HAL_Delay(1200);
  printf("Welcome to UART DEBUGGER\r\n");
  for (;;) {
    fgets(read_buf, 127, stdin);
    printf("> %s\r\n", read_buf);
    PapyrusStatus stat;
    CANMessage tmpmsg;
    for (int i = 0; i < 10; i++) {
      if (!memcmp(sys0_commandnames[i], read_buf,
                  strlen(sys0_commandnames[i])) &&
          isspace((int)read_buf[strlen(sys0_commandnames[i])])) {
        stat = sys0_parsers[i](read_buf, &tmpmsg);
        if (stat == PAPYRUS_OK) {
          printf("Sending a message: ");
          for (uint32_t i = 0; i < tmpmsg.tHeader.DataLength; i++) {
            printf("%02x ", tmpmsg.msg.raw_data[i]);
          }
          printf("\n");
        } else {
          printf("Parse/generate error\r\n");
        }
      }
    }
  }
}
