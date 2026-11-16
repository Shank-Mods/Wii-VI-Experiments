#ifndef AVEPROBE_VIMODE_H
#define AVEPROBE_VIMODE_H

#include <gccore.h>

bool vim_select(const char *name);
const char *vim_names(void);
bool vim_selected(void);
const char *vim_desc(void);
u16 vim_width(void);
u16 vim_height(void);
u32 vim_clock_khz(void);
void vim_enter(void *fb);
void vim_leave(GXRModeObj *rmode, void *safe_fb);

#endif
