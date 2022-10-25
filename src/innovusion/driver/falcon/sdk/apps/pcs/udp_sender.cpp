  /**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include "pcs/udp_sender.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <sys/types.h>

#ifndef __MINGW64__
#include <netdb.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#else
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <iphlpapi.h>
#include <boost/asio/detail/socket_ops.hpp>

#endif

#include <string>

#include "src/utils/inno_lidar_log.h"

namespace innovusion {
UdpSender::UdpSender(const std::string &ip, uint16_t port) {
#ifdef __MINGW64__
  WSADATA wsaData;
  int res = WSAStartup(MAKEWORD(2, 2), &wsaData);
  if (res != NO_ERROR) {
      printf("WSAStartup failed with error %d\n", res);
      exit(1);
  }
#endif
  memset(reinterpret_cast<char *>(&sockaddr_), 0,
         sizeof(sockaddr_));
  multicast_ = false;
  fd_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (fd_ < 0) {
    inno_log_error_errno("cannot open udp fd %s", ip.c_str());
    exit(1);
  }

  const char *udp_ip;
#ifndef __MINGW64__
  char host_broadcast[256];
  if (ip.size() > 0 &&
      isdigit(static_cast<u_char>(ip.c_str()[0])) == 0) {
    int sock;
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
      inno_log_error_errno("cannot open sock %s", ip.c_str());
      exit(1);
    }
    struct ifreq ifreq;
    memset(&ifreq, 0, sizeof ifreq);
    strncpy(ifreq.ifr_name, ip.c_str(), IFNAMSIZ);

    if (ioctl(sock, SIOCGIFBRDADDR, &ifreq) != 0) {
      inno_log_error("Could not find interface named %s", ip.c_str());
      exit(1);
    }
    if (0 != getnameinfo(&ifreq.ifr_broadaddr, sizeof(ifreq.ifr_broadaddr),
                         host_broadcast, sizeof(host_broadcast),
                         0, 0, NI_NUMERICHOST)) {
      inno_log_error_errno("Could getnameinfo %s", ip.c_str());
      exit(1);
    }
    udp_ip = host_broadcast;
    close(sock);
    inno_log_info("Interface %s broadcast ip is %s",
                  ip.c_str(), udp_ip);
  } else {
    udp_ip = ip.c_str();
  }
#else
  udp_ip = ip.c_str();
#endif
  if (strlen(udp_ip) > 4 &&
      strcmp(udp_ip + strlen(udp_ip) - 4, ".255") == 0) {
    int broadcastEnable = 1;
#ifndef __MINGW64__
    int ret = setsockopt(fd_, SOL_SOCKET, SO_BROADCAST,
                         &broadcastEnable, sizeof(broadcastEnable));
#else
    int ret =
        setsockopt(fd_, SOL_SOCKET, SO_BROADCAST,
                   (const char *)&broadcastEnable, sizeof(broadcastEnable));
#endif
    if (ret) {
      inno_log_error_errno("Could not open set socket to broadcast mode %s",
                           udp_ip);
      exit(1);
    } else {
      inno_log_info("%s is broadcast address, set broadcast socket enable",
                    udp_ip);
    }
  }
  sockaddr_.sin_family = AF_INET;
  sockaddr_.sin_port = htons(port);
#ifndef __MINGW64__
  if (inet_aton(udp_ip,
                &sockaddr_.sin_addr) == 0) {
#else
  unsigned long scope_id; //NOLINT
  boost::system::error_code ec;
  if (boost::asio::detail::socket_ops::inet_pton(
          AF_INET, udp_ip, &sockaddr_.sin_addr, &scope_id, ec) == 0) {
#endif
    inno_log_panic("%s is not a valid ip",
                   udp_ip);
  }

  char *ip_str = reinterpret_cast<char *>(&sockaddr_.sin_addr);
  int i = ip_str[0] & 0xFF;
  // we will check only first byte of IP
  // and if it from 224 to 239, then it can
  // represent multicast IP.
  if (i >= 224 && i <= 239) {
    unsigned char ttl = 2;
#ifndef __MINGW64__
    int ret = setsockopt(fd_, IPPROTO_IP,
                         IP_MULTICAST_TTL, &ttl, sizeof(ttl));
#else
    int ret = setsockopt(fd_, IPPROTO_IP, IP_MULTICAST_TTL, (const char *)&ttl,
                         sizeof(ttl));
#endif
    if (ret) {
      inno_log_error_errno("Could not open set socket to multicast mode %s",
                           udp_ip);
      exit(1);
    }

    // disable loopback
    unsigned char loop = 0;
#ifndef __MINGW64__
    setsockopt(fd_, IPPROTO_IP, IP_MULTICAST_LOOP, &loop, sizeof(loop));
#else
    setsockopt(fd_, IPPROTO_IP, IP_MULTICAST_LOOP, (const char *)&loop,
               sizeof(loop));
#endif

    multicast_ = true;
    inno_log_info("%s is multicast address", udp_ip);
  } else {
    inno_log_info("%s is not multicast address", udp_ip);
  }

  udp_ip_str_ = udp_ip;
  inno_log_info("udp sender setup success ip:port=%s:%hu", udp_ip, port);
}

UdpSender::~UdpSender() {
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
#ifdef __MINGW64__
  WSACleanup();
#endif
}

ssize_t UdpSender::write(const void *buffer, size_t size, bool blocking) {
  int flags = 0;
#ifndef __MINGW64__
  //! @Yahui, only for falcon NIC driver bug fix
  if (!blocking) {
    flags |= MSG_DONTWAIT;
  }
#endif
  ssize_t written = sendto(fd_, buffer, size, flags,
                           (struct sockaddr *)&sockaddr_,
                           sizeof(sockaddr_));
  return written;
}

//
// udp with mutil msg
//
ssize_t UdpSender::write_raw(const void *header, size_t header_size,
                             const void *body, size_t body_size) {
  ssize_t written = 0;

  {
#if (defined(__APPLE__) || defined(__MINGW64__))
#ifndef MSG_MORE
# define MSG_MORE 0
#endif
#endif
    ssize_t ret = sendto(fd_, header, header_size, MSG_MORE,
                      (struct sockaddr *)&sockaddr_, sizeof(sockaddr_));
    if (ret == -1) {
      inno_log_warning("sendto faild, errno: %d", errno);
    } else {
      written += ret;
    }
  }

  {
    ssize_t ret = sendto(fd_, body, body_size, 0, (struct sockaddr *)&sockaddr_,
                      sizeof(sockaddr_));
    if (ret == -1) {
      inno_log_warning("sendto faild, errno: %d", errno);
    } else {
      written += ret;
    }
  }

  return written;
}

}  // namespace innovusion
