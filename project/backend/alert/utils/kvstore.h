/* SPDX-License-Identifier: MIT */
/*
 * KVStore - Simple key-value store library
 * Copyright (C) 2025 Vítor Santos
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef KVSTORE_H
#define KVSTORE_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include "../alert.h"

/* Hash table configuration */
#define KV_HASH_BITS 3
#define KV_HASH_SIZE (1 << KV_HASH_BITS)

/**
 * struct kv_node - hash table node (opaque in public API)
 * @next: next node in collision chain
 * @key: key string (sensor_id)
 * @data: latest sensor data payload
 */
struct kv_node {
  char *key;
  SensorData data;
  struct kv_node *next;
};

/**
 * struct kv_store - hash table store
 * @buckets: array of bucket heads
 * @count: number of stored entries
 */
struct kv_store {
  struct kv_node *buckets[KV_HASH_SIZE];
  int count;
};

/* Public API */
int kv_store_init(struct kv_store *store);
void kv_store_destroy(struct kv_store *store);
int kv_store_put(struct kv_store *store, const char *key, const SensorData *data);
int kv_store_get(struct kv_store *store, const char *key, SensorData *value);
int kv_store_delete(struct kv_store *store, const char *key);
bool kv_store_exists(struct kv_store *store, const char *key);
void kv_store_dump(struct kv_store *store);

#endif /* KVSTORE_H */
