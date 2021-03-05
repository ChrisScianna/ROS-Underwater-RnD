/*  QinetiQ North America (QNA)
 *  350 Second Avenue
 *  Waltham, MA 02451
 *
 *  Proprietary Licensed Information.
 *  Not to be duplicated, used, or disclosed,
 *  except under terms of the license.
 *
 *  Copyright © 2018 QinetiQ North America  All rights reserved.
 */

#ifndef IXBLUE_C3_INS_IO_HELPERS_H
#define IXBLUE_C3_INS_IO_HELPERS_H

#include <stddef.h>
#include <sys/time.h>


namespace ixblue_c3_ins
{
namespace io_helpers
{

int openUDPSocket(const char *iface, unsigned short port);

size_t readFor(int fd, void * buffer, size_t size, struct timeval timeout);

}  // namespace io_helpers
}  // namespace ixblue_c3_ins

#endif  // IXBLUE_C3_INS_IO_HELPERS_H
