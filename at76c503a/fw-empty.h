/* $Id: fw-empty.h,v 1.2 2003/12/25 22:40:26 jal2 Exp $ */

/* a dummy struct to use if at76c503-*.o shall load the firmware via hotplug */
static struct firmware static_fw = {0,NULL};
