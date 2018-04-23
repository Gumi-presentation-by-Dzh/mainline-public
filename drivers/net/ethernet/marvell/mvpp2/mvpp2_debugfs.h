#ifndef _MVPP2_DEBUGFS_H
#define _MVPP2_DEBUGFS_H

#include "mvpp2.h"

#ifdef CONFIG_DEBUG_FS

int mvpp2_dbgfs_init(struct mvpp2 *priv, const char *name);

void mvpp2_dbgfs_cleanup(struct mvpp2 *priv);

#else

static int mvpp2_dbgfs_init(struct mvpp2 *priv, const char *name){ return 0; }

static void mvpp2_dbgfs_cleanup(struct mvpp2 *priv){}

#endif

#endif
