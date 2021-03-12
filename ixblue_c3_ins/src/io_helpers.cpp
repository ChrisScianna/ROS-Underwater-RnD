/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, QinetiQ, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of QinetiQ nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <stdint.h>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <system_error>  // NOLINT(build/c++11)


namespace ixblue_c3_ins
{
namespace io_helpers
{

int openUDPSocket(const char *iface, uint16_t port)
{
  int fd;

  if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
  {
    throw std::system_error(errno, std::system_category(), strerror(errno));
  }

  struct sockaddr_in saddr;
  memset(&saddr, 0, sizeof(saddr));
  saddr.sin_family = AF_INET;
  saddr.sin_addr.s_addr = inet_addr(iface);
  saddr.sin_port = htons(port);

  /* UDP socket, we only need to bind, not listen, connect or accept */
  if (bind(fd, (struct sockaddr *)&saddr, sizeof(saddr)) < 0)
  {
    int ec = errno;
    close(fd);
    throw std::system_error(ec, std::system_category(), strerror(ec));
  }

  if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &saddr, sizeof(saddr)) < 0)
  {
    int ec = errno;
    close(fd);
    throw std::system_error(ec, std::system_category(), strerror(ec));
  }

  return fd;
}

size_t readFor(int fd, void * buffer, size_t size, struct timeval timeout)
{
  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(fd, &fds);

  int rc = select(fd + 1, &fds, NULL, NULL, &timeout);

  if (-1 == rc)
  {
    throw std::system_error(errno, std::system_category(), strerror(errno));
  }

  if (0 == rc)
  {
    return 0;
  }

  size_t nbytes = read(fd, buffer, size);
  if (nbytes < 0)
  {
    throw std::system_error(errno, std::system_category(), strerror(errno));
  }

  return nbytes;
}

}  // namespace io_helpers
}  // namespace ixblue_c3_ins
