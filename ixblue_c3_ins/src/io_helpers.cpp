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

