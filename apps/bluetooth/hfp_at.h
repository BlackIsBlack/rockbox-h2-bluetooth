#ifndef HFP_AT_H
#define HFP_AT_H

#include <stdbool.h>

/* Handle an unknown HFP AT command from the headphones (see hfp_at.c).
 * Returns the battery percentage it reported, or -1. Sets *reply to a line
 * to send before OK (or NULL); *ok == false means answer ERROR. */
int hfp_at_handle(const char *cmd, const char **reply, bool *ok);

#endif
