#include "hal_internal.h"

static socket_drv_t *g_link;

void hal_link_set(socket_drv_t *drv) { g_link = drv; }

socket_drv_t *hal_link_get(void) { return g_link; }
