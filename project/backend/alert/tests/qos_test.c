#define _POSIX_C_SOURCE 200809L
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>

#include "../alert_server.h"

// Simple assert macro
#define ASSERT_TRUE(cond, msg) do { \
  if (!(cond)) { \
    fprintf(stderr, "[FAIL] %s\n", msg); \
    exit(EXIT_FAILURE); \
  } else { \
    fprintf(stdout, "[PASS] %s\n", msg); \
  } \
} while (0)

static int create_udp_socket(struct addrinfo **ai_out) {
  struct addrinfo hints = {0}, *res = NULL;
  hints.ai_family = AF_INET6; // match server AF
  hints.ai_socktype = SOCK_DGRAM;
  int rc = getaddrinfo("localhost", UDP_PORT, &hints, &res);
  if (rc != 0) {
    fprintf(stderr, "getaddrinfo failed: %s\n", gai_strerror(rc));
    return -1;
  }
  int s = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
  if (s < 0) {
    perror("socket");
    freeaddrinfo(res);
    return -1;
  }
  // Set recv timeout to 800ms
  struct timeval tv; tv.tv_sec = 0; tv.tv_usec = 800000; // 800ms
  if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
    perror("setsockopt");
    close(s); freeaddrinfo(res); return -1;
  }
  *ai_out = res;
  return s;
}

static int send_packet_and_maybe_recv_ack(int sock, struct addrinfo *ai, uint32_t seq, uint8_t qos, uint8_t flags, const char *json, char *ackBuf, size_t ackBufSz) {
  // Build header + payload buffer
  size_t json_len = strlen(json);
  UdpQoSHeader hdr;
  hdr.seq = seq;
  hdr.qos = qos;
  hdr.flags = flags;

  size_t pkt_len = sizeof(UdpQoSHeader) + json_len;
  char *pkt = (char*)malloc(pkt_len);
  memcpy(pkt, &hdr, sizeof(UdpQoSHeader));
  memcpy(pkt + sizeof(UdpQoSHeader), json, json_len);

  int sent = sendto(sock, pkt, (int)pkt_len, 0, ai->ai_addr, ai->ai_addrlen);
  free(pkt);
  if (sent < 0) {
    perror("sendto");
    return -1;
  }

  // Try to receive ACK
  struct sockaddr_storage src; socklen_t srclen = sizeof(src);
  int n = recvfrom(sock, ackBuf, (int)(ackBufSz - 1), 0, (struct sockaddr*)&src, &srclen);
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return 0; // no data
    }
    perror("recvfrom");
    return -1;
  }
  ackBuf[n] = '\0';
  return n;
}

int main(void) {
  fprintf(stdout, "=== QoS UDP Test ===\n");
  struct addrinfo *ai = NULL;
  int sock = create_udp_socket(&ai);
  ASSERT_TRUE(sock >= 0, "socket created");

  const char *json = "{\"id\":\"urn:ngsi-ld:WeatherObserved:TEST\",\"type\":\"WeatherObserved\",\"dateObserved\":\"2025-11-30T12:00:00Z\",\"temperature\":23.4,\"humidity\":50.1,\"averageTemperature\":23.4,\"averageHumidity\":50.1}";

  char ack[64];

  // Test 1: Best-effort, expect NO ACK
  int r = send_packet_and_maybe_recv_ack(sock, ai, 1, UDP_QOS_BEST_EFFORT, 0, json, ack, sizeof(ack));
  ASSERT_TRUE(r == 0, "best-effort: no ACK received");

  // Test 2: Guaranteed, expect ACK with matching seq
  r = send_packet_and_maybe_recv_ack(sock, ai, 2, UDP_QOS_GUARANTEED, 0, json, ack, sizeof(ack));
  ASSERT_TRUE(r > 0, "guaranteed: ACK received");
  ASSERT_TRUE(strncmp(ack, "ACK ", 4) == 0, "guaranteed: ACK format correct");
  uint32_t ackSeq = (uint32_t)strtoul(ack + 4, NULL, 10);
  ASSERT_TRUE(ackSeq == 2, "guaranteed: ACK seq matches");

  // Test 3: Guaranteed with retransmission flag, still ACK
  r = send_packet_and_maybe_recv_ack(sock, ai, 3, UDP_QOS_GUARANTEED, 0x01, json, ack, sizeof(ack));
  ASSERT_TRUE(r > 0, "guaranteed(retrans): ACK received");
  ASSERT_TRUE(strncmp(ack, "ACK ", 4) == 0, "guaranteed(retrans): ACK format correct");
  ackSeq = (uint32_t)strtoul(ack + 4, NULL, 10);
  ASSERT_TRUE(ackSeq == 3, "guaranteed(retrans): ACK seq matches");

  freeaddrinfo(ai);
  close(sock);
  fprintf(stdout, "=== QoS UDP Test Completed ===\n");
  return 0;
}
