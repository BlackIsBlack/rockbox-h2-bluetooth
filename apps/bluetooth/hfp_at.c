/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * Copyright (C) 2026 Cameron
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This software is distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY
 * KIND, either express or implied.
 *
 ****************************************************************************/

/* Headphone battery over HFP. Bluedroid's AG hands us the AT commands it
 * doesn't know; these are the ways headphones report battery:
 *   Apple:  AT+XAPL=<id>,<features> then AT+IPHONEACCEV=<n>,1,<0-9>,...
 *   HFP 1.7 HF indicators: AT+BIND=..., AT+BIEV=2,<0-100>
 *   Plantronics: AT+XEVENT=BATTERY,<level>,<levels>,...
 * Self-test: cc -DHFP_AT_TEST hfp_at.c && ./a.out */

#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include "hfp_at.h"

static bool starts_with(const char *s, const char *prefix)
{
    for (; *prefix; s++, prefix++) {
        char c = *s;
        if (c >= 'a' && c <= 'z')
            c -= 'a' - 'A';
        if (c != *prefix)
            return false;
    }
    return true;
}

/* Parse up to max comma separated non-negative ints; returns count. */
static int parse_ints(const char *s, int *out, int max)
{
    int n = 0;
    while (n < max) {
        if (*s < '0' || *s > '9')
            break;
        int v = 0;
        while (*s >= '0' && *s <= '9')
            v = v * 10 + (*s++ - '0');
        out[n++] = v;
        if (*s != ',')
            break;
        s++;
    }
    return n;
}

static int clamp_pct(int v)
{
    return v < 0 ? 0 : v > 100 ? 100 : v;
}

int hfp_at_handle(const char *cmd, const char **reply, bool *ok)
{
    int v[16];
    int n;

    *reply = NULL;
    *ok = true;
    if (starts_with(cmd, "AT"))
        cmd += 2;

    if (starts_with(cmd, "+XAPL=")) {
        *reply = "+XAPL=iPhone,2"; /* 2 = battery reporting */
        return -1;
    }
    if (starts_with(cmd, "+IPHONEACCEV=")) {
        n = parse_ints(cmd + 13, v, 16);
        for (int i = 1; i + 1 < n && i < 1 + 2 * v[0]; i += 2)
            if (v[i] == 1)                    /* key 1: battery 0-9 */
                return clamp_pct((v[i + 1] + 1) * 10);
        return -1;
    }
    if (starts_with(cmd, "+BIND=?")) {
        *reply = "+BIND: (2)";               /* we support indicator 2 */
        return -1;
    }
    if (starts_with(cmd, "+BIND?")) {
        *reply = "+BIND: 2,1";               /* ...and it's enabled */
        return -1;
    }
    if (starts_with(cmd, "+BIND="))
        return -1;
    if (starts_with(cmd, "+BIEV=")) {
        n = parse_ints(cmd + 6, v, 2);
        return (n == 2 && v[0] == 2) ? clamp_pct(v[1]) : -1;
    }
    if (starts_with(cmd, "+XEVENT=BATTERY,")) {
        n = parse_ints(cmd + 16, v, 2);
        return (n == 2 && v[1] > 1) ? clamp_pct(100 * v[0] / (v[1] - 1)) : -1;
    }

    *ok = false;
    return -1;
}

#ifdef HFP_AT_TEST
#include <assert.h>
#include <stdio.h>
int main(void)
{
    const char *r;
    bool ok;
    assert(hfp_at_handle("+XAPL=0A5C-1234-0100,10", &r, &ok) == -1 && ok && !strcmp(r, "+XAPL=iPhone,2"));
    assert(hfp_at_handle("+IPHONEACCEV=2,1,7,2,0", &r, &ok) == 80 && ok && !r);
    assert(hfp_at_handle("AT+IPHONEACCEV=1,1,9", &r, &ok) == 100);
    assert(hfp_at_handle("+iphoneaccev=2,2,0,1,0", &r, &ok) == 10);  /* battery second */
    assert(hfp_at_handle("+IPHONEACCEV=1,2,1", &r, &ok) == -1 && ok); /* dock only */
    assert(hfp_at_handle("+BIND=1,2", &r, &ok) == -1 && ok && !r);
    assert(hfp_at_handle("+BIND=?", &r, &ok) == -1 && !strcmp(r, "+BIND: (2)"));
    assert(hfp_at_handle("+BIND?", &r, &ok) == -1 && !strcmp(r, "+BIND: 2,1"));
    assert(hfp_at_handle("+BIEV=2,57", &r, &ok) == 57 && ok);
    assert(hfp_at_handle("+BIEV=1,1", &r, &ok) == -1 && ok);         /* safety ind. */
    assert(hfp_at_handle("+BIEV=2,250", &r, &ok) == 100);
    assert(hfp_at_handle("+XEVENT=BATTERY,3,5,0,0", &r, &ok) == 75);
    assert(hfp_at_handle("+CSRSF=0,0,0,1,0,0,0", &r, &ok) == -1 && !ok);
    puts("hfp_at OK");
    return 0;
}
#endif
