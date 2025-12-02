# Makefile to build the alert server

CC = gcc
CFLAGS = -Wall -Wextra -pthread
LDFLAGS = -pthread -lm
USE_MQTT ?= 1
ifeq ($(USE_MQTT),1)
  CFLAGS += -DUSE_PAHO_MQTT
  LDFLAGS += -lpaho-mqtt3cs
  $(info Building with Paho MQTT support)
else
  $(info Building WITHOUT MQTT support)
endif

# Source files required by alert_server.c
SRCS = alert_server.c utils/json_parser.c utils/kvstore.c lib/cJSON.c lib/mqtt_client.c
OBJS = $(SRCS:.c=.o)
TARGET = alert

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(OBJS) -o $@ $(LDFLAGS)
	@echo "Built executable: $@"
	@echo "Removing object files (post-build cleanup)"
	rm -f $(OBJS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)
	@echo "Cleaned build artifacts"

# =============================
# Test client build and runner
# =============================

TEST_TARGET = tests/qos_test
TEST_SRCS = tests/qos_test.c

.PHONY: test test-build

test-build: $(TARGET) $(TEST_TARGET)

$(TEST_TARGET): $(TEST_SRCS)
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

# Run tests: start server, run client, then stop server
test: test-build
	./$(TARGET) & echo $$! > alert.pid; \
	sleep 1; \
	./$(TEST_TARGET); \
	kill `cat alert.pid`; rm -f alert.pid
