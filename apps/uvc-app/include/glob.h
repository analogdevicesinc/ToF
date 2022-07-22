/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * glob() compatibility
 *
 * Copyright (C) 2018 Laurent Pinchart
 *
 * Contact: Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 */
#ifndef __UVC_GADGET_GLOB_H__
#define __UVC_GADGET_GLOB_H__

#include "config.h"

#ifdef HAVE_GLOB
#include_next <glob.h>
#else
#include "compat/glob.h"
#endif

#endif /* __UVC_GADGET_GLOB_H__ */
