#include "controller_base.h"
papyrus_status_t controller_base_init(controller_base_t *controller,
                                      board_id_t board_id,
                                      controller_type_t controller_type) {
  controller->status.state = CONTROLLER_STATE_INIT;
  controller->config.board_id = board_id;
  controller->config.controller_type = controller_type;
  return PAPYRUS_OK;
}
